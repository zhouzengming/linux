// SPDX-License-Identifier: GPL-2.0-only
/*
 * FV01 / FV04 warm-reset helper for Qualcomm bootloader/fastboot and EDL (9008) modes.
 *
 * - FV01 (QCS6490, OEM TZ.XF.5.11 EL1): standard PSCI_0_2_FN_SYSTEM_RESET forces
 *   PMIC PON_PBS (0x0852) to Hard Reset, wiping SDAM2 reboot-mode and TCSR/IMEM
 *   download cookies. Invoking QCOM_SCM_SVC_PWR / DEASSERT_PS_HOLD (0x42000902)
 *   preserves the warm-reset state when a special reboot target is requested.
 * - FV04 (QCS8550, Gunyah VMID 3): programs IMEM restart_reason@65c (or EDL
 *   diag_dload@c8 + TCSR_BOOT_MISC_DETECT 0x01fd9000) and requests architectural
 *   warm reset (REBOOT_WARM -> PSCI 1.1 SYSTEM_RESET2) alongside PMK8550 SDAM
 *   nvmem-reboot-mode.
 */

#include <asm/barrier.h>
#include <asm/virt.h>
#include <linux/arm-smccc.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/string.h>

#define FV01_TCSR_BOOT_MISC_DETECT	0x01fd3000ULL
#define FV04_TCSR_BOOT_MISC_DETECT	0x01fd9000ULL
#define FV01_IMEM_BASE			0x146aa000ULL
#define FV01_IMEM_SIZE			0x1000UL
#define FV01_IMEM_DIAG_DLOAD_OFFSET	0x0c8UL
#define FV01_IMEM_RESTART_REASON_OFFSET	0x65cUL
#define FV01_RESTART_REASON_BOOTLOADER	0x77665500U
#define FV01_RESTART_REASON_RECOVERY	0x77665502U

#define FV01_SCM_DEASSERT_PS_HOLD	0x42000902UL

enum fv01_reset_target {
	FV01_RESET_NORMAL = 0,
	FV01_RESET_BOOTLOADER,
	FV01_RESET_RECOVERY,
	FV01_RESET_EDL,
};

static void __iomem *fv01_imem_base;
static enum fv01_reset_target fv01_target;
static u64 fv01_tcsr_boot_misc = FV01_TCSR_BOOT_MISC_DETECT;
static bool fv04_psci_warm_mode;

static const char *fv01_reset_name(enum fv01_reset_target target)
{
	switch (target) {
	case FV01_RESET_EDL:
		return "edl";
	case FV01_RESET_BOOTLOADER:
		return "bootloader";
	case FV01_RESET_RECOVERY:
		return "recovery";
	default:
		return "normal";
	}
}

static int fv01_warm_reboot_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	const char *cmd = data;

	if (action != SYS_RESTART || !cmd) {
		fv01_target = FV01_RESET_NORMAL;
		return NOTIFY_DONE;
	}

	if (!strcmp(cmd, "edl"))
		fv01_target = FV01_RESET_EDL;
	else if (!strcmp(cmd, "bootloader") || !strcmp(cmd, "fastboot"))
		fv01_target = FV01_RESET_BOOTLOADER;
	else if (!strcmp(cmd, "recovery"))
		fv01_target = FV01_RESET_RECOVERY;
	else
		fv01_target = FV01_RESET_NORMAL;

	return NOTIFY_DONE;
}

static int fv01_warm_restart_handler(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct arm_smccc_res res;
	int ret;

	if (fv01_target == FV01_RESET_NORMAL)
		return NOTIFY_DONE;

	if (fv01_target == FV01_RESET_EDL) {
		if (fv01_imem_base) {
			writel_relaxed(0x322a4f99U,
				       fv01_imem_base + FV01_IMEM_DIAG_DLOAD_OFFSET + 0x0);
			writel_relaxed(0xc67e4350U,
				       fv01_imem_base + FV01_IMEM_DIAG_DLOAD_OFFSET + 0x4);
			writel_relaxed(0x77777777U,
				       fv01_imem_base + FV01_IMEM_DIAG_DLOAD_OFFSET + 0x8);
			dsb(sy);
		}
		ret = qcom_scm_io_writel(fv01_tcsr_boot_misc, 0x1U);
		if (ret)
			pr_err("fv01-warm-reset: failed to write TCSR_BOOT_MISC_DETECT: %d\n",
			       ret);
	} else if (fv01_imem_base) {
		u32 reason = (fv01_target == FV01_RESET_RECOVERY) ?
			     FV01_RESTART_REASON_RECOVERY :
			     FV01_RESTART_REASON_BOOTLOADER;

		writel_relaxed(reason, fv01_imem_base + FV01_IMEM_RESTART_REASON_OFFSET);
		dsb(sy);
	}

	if (fv04_psci_warm_mode) {
		reboot_mode = REBOOT_WARM;
		pr_emerg("fv01-warm-reset: FV04 warm-reset cookies set for %s\n",
			 fv01_reset_name(fv01_target));
		return NOTIFY_DONE;
	}

	if (!is_hyp_mode_available()) {
		pr_emerg("fv01-warm-reset: invoking SCM DEASSERT_PS_HOLD for %s\n",
			 fv01_reset_name(fv01_target));
		arm_smccc_smc(FV01_SCM_DEASSERT_PS_HOLD, 1, 0, 0, 0, 0, 0, 0, &res);
	}

	return NOTIFY_DONE;
}

static struct notifier_block fv01_warm_reboot_nb = {
	.notifier_call = fv01_warm_reboot_notify,
	.priority = 255,
};

static struct notifier_block fv01_warm_restart_nb = {
	.notifier_call = fv01_warm_restart_handler,
	.priority = 200,
};

static int __init fv01_warm_reset_init(void)
{
	int ret;

	if (of_machine_is_compatible("faiot,fv04") ||
	    of_machine_is_compatible("qcom,kalamap-hdk")) {
		fv01_tcsr_boot_misc = FV04_TCSR_BOOT_MISC_DETECT;
		fv04_psci_warm_mode = true;
	} else if (!of_machine_is_compatible("faiot,fv01") &&
		   !of_machine_is_compatible("qcom,yupikp-iot")) {
		return 0;
	}

	fv01_imem_base = ioremap(FV01_IMEM_BASE, FV01_IMEM_SIZE);
	if (!fv01_imem_base)
		pr_warn("fv01-warm-reset: failed to ioremap IMEM, IMEM cookies skipped\n");

	ret = register_reboot_notifier(&fv01_warm_reboot_nb);
	if (ret)
		goto err_unmap;

	ret = register_restart_handler(&fv01_warm_restart_nb);
	if (ret)
		goto err_unreg_reboot;

	pr_info("fv01-warm-reset: registered bootloader/recovery/edl warm-reset handler\n");
	return 0;

err_unreg_reboot:
	unregister_reboot_notifier(&fv01_warm_reboot_nb);
err_unmap:
	if (fv01_imem_base) {
		iounmap(fv01_imem_base);
		fv01_imem_base = NULL;
	}
	return ret;
}
late_initcall(fv01_warm_reset_init);
