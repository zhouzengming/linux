// SPDX-License-Identifier: GPL-2.0-only
/* FV01 vendor sysfs/thermal ABI on the upstream PWM framework.
 * Derived from observable V02.07 ABI and hardware validation.
 * This driver must not claim the generic "pwm-fan" compatible.
 */
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

struct fv01_fan {
	struct mutex lock;
	struct pwm_device *pwm;
	struct gpio_desc *power;
	u64 period;
	unsigned int level;
	unsigned long cooling_state;
	bool power_on;
};

static int fv01_fan_apply(struct fv01_fan *fan, unsigned long level)
{
	struct pwm_state state;
	unsigned long effective = max(level, 1UL);
	int ret;

	/* V02.07 maps both 0 and 1 to one third. Power is independent. */
	pwm_init_state(fan->pwm, &state);
	state.period = fan->period;
	state.duty_cycle = DIV_ROUND_UP_ULL((fan->period - 1) * effective, 3);
	state.polarity = PWM_POLARITY_NORMAL;
	state.enabled = true;
	ret = pwm_apply_might_sleep(fan->pwm, &state);
	if (!ret) {
		fan->level = effective;
		fan->cooling_state = level;
	}
	return ret;
}

static ssize_t fan_power_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	int value;

	mutex_lock(&fan->lock);
	value = gpiod_get_raw_value_cansleep(fan->power);
	mutex_unlock(&fan->lock);
	return value < 0 ? value : sysfs_emit(buf, "%d\n", !!value);
}

static ssize_t fan_power_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 0, &value);
	if (ret || value > 1)
		return -EINVAL;
	mutex_lock(&fan->lock);
	ret = gpiod_direction_output_raw(fan->power, value);
	if (!ret)
		fan->power_on = !!value;
	mutex_unlock(&fan->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(fan_power);

static ssize_t pwm_duty_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	unsigned int level;

	mutex_lock(&fan->lock);
	level = fan->level;
	mutex_unlock(&fan->lock);
	return sysfs_emit(buf, "%u\n", level);
}

static ssize_t pwm_duty_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	unsigned long level;
	int ret;

	ret = kstrtoul(buf, 10, &level);
	if (ret || level > 3)
		return -EINVAL;
	mutex_lock(&fan->lock);
	ret = fv01_fan_apply(fan, level);
	mutex_unlock(&fan->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pwm_duty);

static struct attribute *fv01_fan_attrs[] = {
	&dev_attr_fan_power.attr,
	&dev_attr_pwm_duty.attr,
	NULL,
};
static const struct attribute_group fv01_fan_group = {
	.name = "fan",
	.attrs = fv01_fan_attrs,
};

static int fv01_get_max_state(struct thermal_cooling_device *cdev,
			      unsigned long *state)
{
	*state = 3;
	return 0;
}

static int fv01_get_cur_state(struct thermal_cooling_device *cdev,
			      unsigned long *state)
{
	struct fv01_fan *fan = cdev->devdata;

	mutex_lock(&fan->lock);
	*state = fan->cooling_state;
	mutex_unlock(&fan->lock);
	return 0;
}

static int fv01_set_cur_state(struct thermal_cooling_device *cdev,
			      unsigned long state)
{
	struct fv01_fan *fan = cdev->devdata;
	int ret;

	if (state > 3)
		return -EINVAL;
	mutex_lock(&fan->lock);
	ret = fv01_fan_apply(fan, state);
	mutex_unlock(&fan->lock);
	return ret;
}

static const struct thermal_cooling_device_ops fv01_cooling_ops = {
	.get_max_state = fv01_get_max_state,
	.get_cur_state = fv01_get_cur_state,
	.set_cur_state = fv01_set_cur_state,
};

static int fv01_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_cooling_device *cdev;
	struct pwm_args args;
	struct fv01_fan *fan;
	int ret;

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;
	mutex_init(&fan->lock);
	platform_set_drvdata(pdev, fan);
	fan->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(fan->pwm))
		return dev_err_probe(dev, PTR_ERR(fan->pwm), "PWM unavailable\n");
	pwm_get_args(fan->pwm, &args);
	if (args.period < 3 || args.period > NSEC_PER_SEC)
		return -EINVAL;
	fan->period = args.period;
	fan->power = devm_gpiod_get(dev, "power", GPIOD_ASIS);
	if (IS_ERR(fan->power))
		return dev_err_probe(dev, PTR_ERR(fan->power), "power GPIO unavailable\n");

	/*
	 * Match Factory 5.4.259 fan_driver_probe: enable PWM at initial 1/3
	 * duty and assert GPIO82 fan_power HIGH at boot so active cooling runs
	 * immediately, while allowing userspace sysfs to turn fan_power off.
	 */
	ret = fv01_fan_apply(fan, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize fan PWM\n");

	ret = gpiod_direction_output_raw(fan->power, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable fan power GPIO\n");
	fan->power_on = true;

	cdev = devm_thermal_of_cooling_device_register(dev, 0, "fv01-fan", fan,
						       &fv01_cooling_ops);
	if (IS_ERR(cdev))
		return PTR_ERR(cdev);

	ret = devm_device_add_group(dev, &fv01_fan_group);
	if (ret)
		return ret;

	dev_info(dev, "FV01 fan enabled (period=%llu ns, level=%u, state=%lu)\n",
		 fan->period, fan->level, fan->cooling_state);
	return 0;
}

static int fv01_fan_suspend(struct device *dev)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	struct pwm_state state;

	mutex_lock(&fan->lock);
	gpiod_direction_output_raw(fan->power, 0);
	pwm_get_state(fan->pwm, &state);
	state.enabled = false;
	pwm_apply_might_sleep(fan->pwm, &state);
	mutex_unlock(&fan->lock);
	return 0;
}

static int fv01_fan_resume(struct device *dev)
{
	struct fv01_fan *fan = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&fan->lock);
	ret = fv01_fan_apply(fan, fan->cooling_state);
	if (fan->power_on)
		gpiod_direction_output_raw(fan->power, 1);
	mutex_unlock(&fan->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fv01_fan_pm_ops, fv01_fan_suspend, fv01_fan_resume);

static const struct of_device_id fv01_fan_match[] = {
	{ .compatible = "faiot,fv01-fan" },
	{ }
};
MODULE_DEVICE_TABLE(of, fv01_fan_match);

static struct platform_driver fv01_fan_driver = {
	.probe = fv01_fan_probe,
	.driver = {
		.name = "fv01-fan",
		.of_match_table = fv01_fan_match,
		.pm = pm_sleep_ptr(&fv01_fan_pm_ops),
	},
};
module_platform_driver(fv01_fan_driver);
MODULE_DESCRIPTION("FV01 legacy fan ABI using upstream Qualcomm LPG PWM");
MODULE_LICENSE("GPL");
