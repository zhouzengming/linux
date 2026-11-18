// SPDX-License-Identifier: GPL-2.0-only
/*
 * FV01 keepalive, Linux watchdog_device, and PM suspend/resume support
 * for the Qualcomm Haven hypervisor virtual watchdog (qcom,hh-watchdog).
 */
#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/timer.h>
#include <linux/watchdog.h>

#define FV01_HH_WDT_CTRL       0x86000005UL
#define FV01_HH_WDT_STATUS     0x86000006UL
#define FV01_HH_WDT_PET        0x86000007UL
#define FV01_HH_WDT_SET_TIME   0x86000008UL
#define FV01_HH_WDT_NO_CHANGE  0xffffUL
#define FV01_HH_WDT_BARK_MS    11000U
#define FV01_HH_WDT_BITE_MS    14000U

static struct timer_list fv01_hh_wdt_timer;
static bool fv01_hh_wdt_present;
static bool fv01_hh_wdt_armed;
static bool fv01_hh_wdt_suspended;
static bool fv01_hh_wdt_wdd_registered;
static unsigned int fv01_hh_wdt_count;

static struct arm_smccc_res fv01_hh_wdt_smc(unsigned long function,
					    unsigned long arg1,
					    unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function, arg1, arg2, 0, 0, 0, 0, 0, &res);
	return res;
}

static void fv01_hh_wdt_log_status(const char *tag)
{
	struct arm_smccc_res res = fv01_hh_wdt_smc(FV01_HH_WDT_STATUS, 0, 0);

	pr_info("fv01-hh-wdt: %s a0=0x%lx ctl=0x%lx age=0x%lx extra=0x%lx\n",
		tag, (unsigned long)res.a0, (unsigned long)res.a1,
		(unsigned long)res.a2, (unsigned long)res.a3);
}

static int fv01_hh_wdt_set(unsigned long bark, unsigned long bite)
{
	struct arm_smccc_res res;

	res = fv01_hh_wdt_smc(FV01_HH_WDT_SET_TIME, bark, bite);
	pr_debug("fv01-hh-wdt: set bark=0x%lx bite=0x%lx result=0x%lx\n",
		 bark, bite, (unsigned long)res.a0);
	return res.a0 ? -EIO : 0;
}

static void fv01_hh_wdt_pet_fn(struct timer_list *timer)
{
	struct arm_smccc_res res;

	if (READ_ONCE(fv01_hh_wdt_suspended) || READ_ONCE(fv01_hh_wdt_wdd_registered))
		return;

	res = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	fv01_hh_wdt_count++;
	if (fv01_hh_wdt_count <= 2) {
		pr_info("fv01-hh-wdt: early pet=%u result=0x%lx\n",
			fv01_hh_wdt_count, (unsigned long)res.a0);
	}
	mod_timer(timer, jiffies + msecs_to_jiffies(1000));
}

static int fv01_hh_wdd_start(struct watchdog_device *wdd)
{
	struct arm_smccc_res ctrl, pet;

	ctrl = fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 3, 0);
	pet = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	WRITE_ONCE(fv01_hh_wdt_armed, true);
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	return (ctrl.a0 || pet.a0) ? -EIO : 0;
}

static int fv01_hh_wdd_stop(struct watchdog_device *wdd)
{
	struct arm_smccc_res ctrl;

	fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	ctrl = fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 0, 0);
	WRITE_ONCE(fv01_hh_wdt_armed, false);
	clear_bit(WDOG_HW_RUNNING, &wdd->status);
	return ctrl.a0 ? -EIO : 0;
}

static int fv01_hh_wdd_ping(struct watchdog_device *wdd)
{
	struct arm_smccc_res res;

	if (READ_ONCE(fv01_hh_wdt_suspended))
		return 0;

	res = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	fv01_hh_wdt_count++;
	return res.a0 ? -EIO : 0;
}

static int fv01_hh_wdd_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	unsigned long bite_ms = (unsigned long)timeout * 1000UL;
	unsigned long bark_ms = (timeout > 3 ? (unsigned long)(timeout - 3) : (unsigned long)(timeout - 1)) * 1000UL;
	int ret;

	ret = fv01_hh_wdt_set(bark_ms, FV01_HH_WDT_NO_CHANGE);
	if (ret)
		return ret;
	ret = fv01_hh_wdt_set(FV01_HH_WDT_NO_CHANGE, bite_ms);
	if (ret)
		return ret;

	wdd->timeout = timeout;
	return 0;
}

static const struct watchdog_info fv01_hh_wdd_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.identity = "QCOM Haven Hypervisor WDT",
};

static const struct watchdog_ops fv01_hh_wdd_ops = {
	.owner = THIS_MODULE,
	.start = fv01_hh_wdd_start,
	.stop = fv01_hh_wdd_stop,
	.ping = fv01_hh_wdd_ping,
	.set_timeout = fv01_hh_wdd_set_timeout,
};

static struct watchdog_device fv01_hh_wdd = {
	.info = &fv01_hh_wdd_info,
	.ops = &fv01_hh_wdd_ops,
	.min_timeout = 4,
	.max_timeout = 30,
	.timeout = FV01_HH_WDT_BITE_MS / 1000U,
	.max_hw_heartbeat_ms = 4000,
};

static int fv01_hh_wdt_pm_notify(struct notifier_block *nb,
				 unsigned long mode, void *_unused)
{
	struct arm_smccc_res pet, ctrl;

	if (!READ_ONCE(fv01_hh_wdt_armed))
		return NOTIFY_DONE;

	switch (mode) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		WRITE_ONCE(fv01_hh_wdt_suspended, true);
		if (!READ_ONCE(fv01_hh_wdt_wdd_registered))
			timer_delete_sync(&fv01_hh_wdt_timer);
		pet = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
		ctrl = fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 0, 0);
		pr_info("fv01-hh-wdt: paused for suspend pet=0x%lx disable=0x%lx\n",
			(unsigned long)pet.a0, (unsigned long)ctrl.a0);
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		pet = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
		ctrl = fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 3, 0);
		WRITE_ONCE(fv01_hh_wdt_suspended, false);
		if (!READ_ONCE(fv01_hh_wdt_wdd_registered))
			mod_timer(&fv01_hh_wdt_timer, jiffies + msecs_to_jiffies(100));
		pr_info("fv01-hh-wdt: resumed from suspend pet=0x%lx enable=0x%lx\n",
			(unsigned long)pet.a0, (unsigned long)ctrl.a0);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block fv01_hh_wdt_pm_nb = {
	.notifier_call = fv01_hh_wdt_pm_notify,
};

static int fv01_hh_wdt_syscore_suspend(void *data)
{
	if (READ_ONCE(fv01_hh_wdt_armed)) {
		fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
		fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 0, 0);
	}
	return 0;
}

static void fv01_hh_wdt_syscore_resume(void *data)
{
	if (READ_ONCE(fv01_hh_wdt_armed)) {
		fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
		fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 3, 0);
	}
}

static const struct syscore_ops fv01_hh_wdt_syscore_ops = {
	.suspend = fv01_hh_wdt_syscore_suspend,
	.resume = fv01_hh_wdt_syscore_resume,
};

static struct syscore fv01_hh_wdt_syscore = {
	.ops = &fv01_hh_wdt_syscore_ops,
};

static int __init fv01_hh_wdt_early_init(void)
{
	struct device_node *node;
	struct arm_smccc_res enable, pet;

	node = of_find_compatible_node(NULL, NULL, "qcom,hh-watchdog");
	if (!node)
		return 0;
	of_node_put(node);

	fv01_hh_wdt_present = true;
	fv01_hh_wdt_log_status("before");
	fv01_hh_wdt_set(FV01_HH_WDT_BARK_MS, FV01_HH_WDT_NO_CHANGE);
	fv01_hh_wdt_set(FV01_HH_WDT_NO_CHANGE, FV01_HH_WDT_BITE_MS);
	enable = fv01_hh_wdt_smc(FV01_HH_WDT_CTRL, 3, 0);
	pr_info("fv01-hh-wdt: enable result=0x%lx\n", (unsigned long)enable.a0);
	pet = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	pr_info("fv01-hh-wdt: initial pet result=0x%lx\n", (unsigned long)pet.a0);

	timer_setup(&fv01_hh_wdt_timer, fv01_hh_wdt_pet_fn, 0);
	mod_timer(&fv01_hh_wdt_timer, jiffies + msecs_to_jiffies(100));
	fv01_hh_wdt_armed = true;
	register_pm_notifier(&fv01_hh_wdt_pm_nb);
	register_syscore(&fv01_hh_wdt_syscore);
	fv01_hh_wdt_log_status("armed");
	return 0;
}
arch_initcall(fv01_hh_wdt_early_init);

static int __init fv01_hh_wdt_register_dev(void)
{
	int ret;

	if (!fv01_hh_wdt_present)
		return 0;

	set_bit(WDOG_HW_RUNNING, &fv01_hh_wdd.status);
	ret = watchdog_register_device(&fv01_hh_wdd);
	if (ret) {
		pr_warn("fv01-hh-wdt: watchdog_register_device failed (%d), keeping early timer\n", ret);
		return 0;
	}

	WRITE_ONCE(fv01_hh_wdt_wdd_registered, true);
	timer_delete_sync(&fv01_hh_wdt_timer);
	pr_info("fv01-hh-wdt: registered /dev/watchdog%d (timeout=%us, hw_heartbeat=4000ms)\n",
		fv01_hh_wdd.id, fv01_hh_wdd.timeout);
	return 0;
}
late_initcall(fv01_hh_wdt_register_dev);

module_param_named(armed, fv01_hh_wdt_armed, bool, 0444);
module_param_named(pets, fv01_hh_wdt_count, uint, 0444);
MODULE_DESCRIPTION("Qualcomm Haven hypervisor watchdog keepalive, watchdog_device, and PM driver");
MODULE_LICENSE("GPL");
