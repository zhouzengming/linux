// SPDX-License-Identifier: GPL-2.0-only
/* Experimental FV01 board supply group for the TC9563 PCIe hierarchy.
 * GPIO ordering and RAW levels follow the saved factory qps615_power_probe.
 * This is an always-on bring-up supply; runtime PM and suspend are untested.
 * Host PERST remains owned solely by the Qualcomm PCIe driver.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

struct fv01_pcie_power {
	struct device *dev;
	struct gpio_desc *dc, *core, *io, *ethernet, *usb_reset;
	bool enabled;
	struct regulator_desc desc;
};

static int fv01_pcie_enable(struct regulator_dev *rdev)
{
	struct fv01_pcie_power *p = rdev_get_drvdata(rdev);
	int ret;

	if (p->enabled)
		return 0;
	ret = gpiod_direction_output_raw(p->dc, 1);
	if (ret)
		return ret;
	ret = gpiod_direction_output_raw(p->core, 1);
	if (ret)
		return ret;
	ret = gpiod_direction_output_raw(p->usb_reset, 0);
	if (ret)
		return ret;
	ret = gpiod_direction_output_raw(p->ethernet, 1);
	if (ret)
		return ret;
	/* Factory waits ten 1 ms delays before enabling the 1.8 V group. */
	msleep(10);
	ret = gpiod_direction_output_raw(p->io, 1);
	if (ret)
		return ret;
	msleep(20);
	p->enabled = true;
	dev_info(p->dev, "factory power GPIO sequence complete; USB reset held raw LOW\n");
	return 0;
}

static int fv01_pcie_is_enabled(struct regulator_dev *rdev)
{
	struct fv01_pcie_power *p = rdev_get_drvdata(rdev);

	/* Avoid unnecessary register reads with the vendor secure firmware. */
	return p->enabled;
}

static const struct regulator_ops fv01_pcie_ops = {
	.enable = fv01_pcie_enable,
	.is_enabled = fv01_pcie_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
};

static int fv01_pcie_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = { .dev = dev, .of_node = dev->of_node };
	struct regulator_dev *rdev;
	struct fv01_pcie_power *p;

	if (!of_machine_is_compatible("faiot,fv01") &&
	    !of_machine_is_compatible("qcom,yupikp-iot"))
		return -ENODEV;
	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	/* ASIS preserves firmware states until the explicit enable sequence. */
#define FV01_GET_GPIO(member, con) do { \
	p->member = devm_gpiod_get(dev, con, GPIOD_ASIS); \
	if (IS_ERR(p->member)) \
		return dev_err_probe(dev, PTR_ERR(p->member), "failed to get " con " GPIO\n"); \
} while (0)
	FV01_GET_GPIO(dc, "dc-in");
	FV01_GET_GPIO(core, "core");
	FV01_GET_GPIO(io, "io");
	FV01_GET_GPIO(ethernet, "ethernet-power");
	FV01_GET_GPIO(usb_reset, "usb-reset");
#undef FV01_GET_GPIO
	p->desc.name = "fv01-pcie-supply-group";
	p->desc.type = REGULATOR_VOLTAGE;
	p->desc.owner = THIS_MODULE;
	p->desc.ops = &fv01_pcie_ops;
	p->desc.n_voltages = 1;
	p->desc.min_uV = 3300000;
	p->desc.fixed_uV = 3300000;
	cfg.driver_data = p;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &p->desc);
	if (!cfg.init_data)
		return -EINVAL;
	rdev = devm_regulator_register(dev, &p->desc, &cfg);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id fv01_pcie_power_match[] = {
	{ .compatible = "faiot,fv01-pcie-supply-group" },
	{}
};
MODULE_DEVICE_TABLE(of, fv01_pcie_power_match);

static struct platform_driver fv01_pcie_power_driver = {
	.probe = fv01_pcie_power_probe,
	.driver = {
		.name = "fv01-pcie-power",
		.of_match_table = fv01_pcie_power_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(fv01_pcie_power_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FV01 factory GPIO sequence for PCIe1 bring-up");
