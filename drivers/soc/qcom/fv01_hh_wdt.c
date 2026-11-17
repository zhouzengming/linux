/*
 * FV01 diagnostic keepalive for the Qualcomm hypervisor virtual watchdog.
 */
#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/timer.h>

#define FV01_HH_WDT_CTRL       0x86000005UL
#define FV01_HH_WDT_STATUS     0x86000006UL
#define FV01_HH_WDT_PET        0x86000007UL
#define FV01_HH_WDT_SET_TIME   0x86000008UL
#define FV01_HH_WDT_NO_CHANGE  0xffffUL
#define FV01_HH_WDT_BARK_MS    11000U
#define FV01_HH_WDT_BITE_MS    14000U

static struct timer_list fv01_hh_wdt_timer;
static bool fv01_hh_wdt_armed;
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
	pr_info("fv01-hh-wdt: set bark=0x%lx bite=0x%lx result=0x%lx\n",
		bark, bite, (unsigned long)res.a0);
	return res.a0 ? -EIO : 0;
}

static void fv01_hh_wdt_pet_fn(struct timer_list *timer)
{
	struct arm_smccc_res res;

	res = fv01_hh_wdt_smc(FV01_HH_WDT_PET, 0, 0);
	fv01_hh_wdt_count++;
	pr_info("fv01-hh-wdt: pet=%u result=0x%lx\n",
		fv01_hh_wdt_count, (unsigned long)res.a0);
	if (fv01_hh_wdt_count <= 4 || !(fv01_hh_wdt_count % 5))
		fv01_hh_wdt_log_status("status");
	mod_timer(timer, jiffies + msecs_to_jiffies(1000));
}

static int __init fv01_hh_wdt_early_init(void)
{
	struct device_node *node;
	struct arm_smccc_res enable, pet;

	node = of_find_compatible_node(NULL, NULL, "qcom,hh-watchdog");
	if (!node)
		return 0;
	of_node_put(node);

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
	fv01_hh_wdt_log_status("armed");
	return 0;
}
arch_initcall(fv01_hh_wdt_early_init);

module_param_named(armed, fv01_hh_wdt_armed, bool, 0444);
MODULE_DESCRIPTION("FV01 Qualcomm hypervisor watchdog diagnostics");
MODULE_LICENSE("GPL v2");
