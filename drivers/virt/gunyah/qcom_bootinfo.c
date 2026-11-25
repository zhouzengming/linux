// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Gunyah runtime hypervisor DT overlay recovery for mainline boots.
 *
 * When booting directly via open-source U-Boot / EFI without proprietary ABL,
 * Qualcomm's per-boot /hypervisor overlay (carrying CSPRNG-randomized Resource
 * Manager message-queue capability IDs) is not pre-applied to the base DTB.
 * Retrieve HypBootInfo from QSEE SIP SMC (0x02000609 / 0x42000609), normalize
 * fragment@0 for mainline GICv3 #interrupt-cells, and apply it at runtime.
 */

#include <linux/arm-smccc.h>
#include <linux/gunyah.h>
#include <linux/io.h>
#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "gunyah_qcom.h"

#define QCOM_HYP_INFO_GET_DTB_SMC32	0x02000609UL
#define QCOM_HYP_INFO_GET_DTB_SMC64	0x42000609UL
#define QCOM_HYP_BOOTINFO_MAGIC		0xc06b0071U
#define QCOM_HYP_MAX_DTBOS		4
#define QCOM_HYP_MAX_DTBO_SIZE		SZ_1M

struct qcom_hyp_bootinfo {
	u32 magic;
	u32 version;
	u32 size;
	u32 pil_enable;
	u32 vm_type;
	u32 num_dtbos;
	struct {
		u64 base;
		u64 size;
	} dtbo[QCOM_HYP_MAX_DTBOS];
} __packed;

static int qcom_hyp_ovcs_id = -1;

static int qcom_hyp_read_phys(phys_addr_t phys, void *dst, size_t size)
{
	void __iomem *mapping;
	int ret;

	mapping = ioremap(phys, size);
	if (!mapping)
		return -ENOMEM;

	ret = copy_from_kernel_nofault(dst, (const void __force *)mapping, size);
	iounmap(mapping);
	return ret;
}

static u32 qcom_hyp_root_interrupt_cells(void)
{
	struct device_node *intc;
	u32 cells = 3;

	intc = of_irq_find_parent(of_root);
	if (intc) {
		of_property_read_u32(intc, "#interrupt-cells", &cells);
		of_node_put(intc);
	}
	return cells;
}

static int qcom_hyp_prepare_rm_overlay(void *fdt, u32 intc_cells)
{
	static const char * const discard_paths[] = {
		"/__local_fixups__",
		"/__fixups__",
		"/fragment@7",
		"/fragment@6",
		"/fragment@5",
		"/fragment@4",
		"/fragment@3",
		"/fragment@2",
		"/fragment@1",
	};
	const fdt32_t *old_irqs;
	fdt32_t new_irqs[8];
	unsigned int i;
	int hyp_node, rm_node, wdt_node, len, ret;

	for (i = 0; i < ARRAY_SIZE(discard_paths); i++) {
		int node = fdt_path_offset(fdt, discard_paths[i]);

		if (node < 0) {
			if (node == -FDT_ERR_NOTFOUND)
				continue;
			return node;
		}
		ret = fdt_del_node(fdt, node);
		if (ret)
			return ret;
	}

	hyp_node = fdt_path_offset(fdt, "/fragment@0/__overlay__/hypervisor");
	if (hyp_node < 0)
		return hyp_node;

	rm_node = fdt_node_offset_by_compatible(fdt, hyp_node, "qcom,resource-manager-1-0");
	if (rm_node < 0)
		rm_node = fdt_node_offset_by_compatible(fdt, hyp_node, "qcom,resource-manager");
	if (rm_node < 0)
		return rm_node;

	old_irqs = fdt_getprop(fdt, rm_node, "interrupts", &len);
	if (!old_irqs)
		return -EINVAL;

	/*
	 * Qualcomm Gunyah RM emits 3-cell GICv3 SPI descriptors, whereas
	 * mainline SM8550/QCS8550 GICv3 uses #interrupt-cells = <4>.
	 */
	if (intc_cells == 4 && len == 6 * sizeof(*old_irqs)) {
		for (i = 0; i < 2; i++) {
			memcpy(&new_irqs[i * 4], &old_irqs[i * 3],
			       3 * sizeof(*old_irqs));
			new_irqs[i * 4 + 3] = cpu_to_fdt32(0);
		}
		ret = fdt_setprop(fdt, rm_node, "interrupts", new_irqs,
				  sizeof(new_irqs));
		if (ret)
			return ret;
	}

	hyp_node = fdt_path_offset(fdt, "/fragment@0/__overlay__/hypervisor");
	if (hyp_node >= 0) {
		wdt_node = fdt_node_offset_by_compatible(fdt, hyp_node, "qcom,gh-watchdog");
		if (wdt_node >= 0) {
			old_irqs = fdt_getprop(fdt, wdt_node, "interrupts", &len);
			if (old_irqs && intc_cells == 4 && len == 3 * sizeof(*old_irqs)) {
				memcpy(new_irqs, old_irqs, 3 * sizeof(*old_irqs));
				new_irqs[3] = cpu_to_fdt32(0);
				ret = fdt_setprop(fdt, wdt_node, "interrupts", new_irqs,
						  4 * sizeof(*new_irqs));
				if (ret)
					return ret;
			}
		}
	}

	return fdt_pack(fdt);
}

static int qcom_hyp_apply_dtbo(phys_addr_t phys, size_t allocation, u32 intc_cells)
{
	struct device_node *hypervisor;
	void *fdt;
	u32 total;
	int ovcs_id = 0, ret;

	if (!phys || allocation < sizeof(struct fdt_header) ||
	    allocation > QCOM_HYP_MAX_DTBO_SIZE)
		return -EINVAL;

	fdt = kmalloc(allocation, GFP_KERNEL);
	if (!fdt)
		return -ENOMEM;

	ret = qcom_hyp_read_phys(phys, fdt, allocation);
	if (ret)
		goto out;

	ret = fdt_check_header(fdt);
	if (ret)
		goto out;

	total = fdt_totalsize(fdt);
	if (total > allocation) {
		ret = -EOVERFLOW;
		goto out;
	}

	ret = qcom_hyp_prepare_rm_overlay(fdt, intc_cells);
	if (ret)
		goto out;

	ret = of_overlay_fdt_apply(fdt, fdt_totalsize(fdt), &ovcs_id, NULL);
	if (!ret) {
		qcom_hyp_ovcs_id = ovcs_id;
		hypervisor = of_find_node_by_path("/hypervisor");
		if (hypervisor) {
			ret = of_platform_populate(hypervisor, NULL, NULL, NULL);
			of_node_put(hypervisor);
		}
		pr_info("gunyah_qcom: applied runtime /hypervisor overlay (id=%d, populate=%d)\n",
			ovcs_id, ret);
	}

out:
	kfree(fdt);
	return ret;
}

int qcom_hyp_bootinfo_init(void)
{
	static const unsigned long smc_ids[] = {
		QCOM_HYP_INFO_GET_DTB_SMC32,
		QCOM_HYP_INFO_GET_DTB_SMC64,
	};
	struct qcom_hyp_bootinfo info = {};
	struct arm_smccc_res res = {};
	struct device_node *hypervisor;
	unsigned int i, count;
	phys_addr_t bootinfo_pa = 0;
	u32 intc_cells;
	int ret = -ENODEV;

	if (!arch_is_gh_guest())
		return -ENODEV;

	hypervisor = of_find_node_by_path("/hypervisor");
	if (hypervisor) {
		of_node_put(hypervisor);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(smc_ids); i++) {
		memset(&res, 0, sizeof(res));
		arm_smccc_1_1_smc(smc_ids[i], 0, 0, 0, &res);
		if (!res.a0 && res.a1) {
			bootinfo_pa = res.a1;
			break;
		}
	}
	if (!bootinfo_pa)
		return -ENODEV;

	ret = qcom_hyp_read_phys(bootinfo_pa, &info, sizeof(info));
	if (ret)
		return ret;

	if (info.magic != QCOM_HYP_BOOTINFO_MAGIC ||
	    info.size < offsetofend(struct qcom_hyp_bootinfo, num_dtbos))
		return -EINVAL;

	intc_cells = qcom_hyp_root_interrupt_cells();
	count = min_t(unsigned int, info.num_dtbos, QCOM_HYP_MAX_DTBOS);
	for (i = 0; i < count; i++) {
		ret = qcom_hyp_apply_dtbo(info.dtbo[i].base, info.dtbo[i].size,
					  intc_cells);
		if (!ret)
			return 0;
	}

	pr_warn("gunyah_qcom: runtime /hypervisor overlay recovery failed: %d\n", ret);
	return ret;
}

void qcom_hyp_bootinfo_exit(void)
{
	if (qcom_hyp_ovcs_id >= 0) {
		of_overlay_remove(&qcom_hyp_ovcs_id);
		qcom_hyp_ovcs_id = -1;
	}
}
