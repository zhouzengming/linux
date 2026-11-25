// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/arm-smccc.h>
#include <linux/gunyah.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/uuid.h>

#define QCOM_HYP_INFO_GET_DTB_SMC32	0x02000609UL

enum gh_hvc_abi {
	GH_HVC_ABI_UNKNOWN = 0,
	GH_HVC_ABI_NONE,
	GH_HVC_ABI_SMCCC,
	GH_HVC_ABI_IMM6000,
};

static enum gh_hvc_abi gh_hvc_mode = GH_HVC_ABI_UNKNOWN;

/* {c1d58fcd-a453-5fdb-9265-ce36673d5f14} */
static const uuid_t GUNYAH_UUID =
	UUID_INIT(0xc1d58fcd, 0xa453, 0x5fdb, 0x92, 0x65, 0xce, 0x36, 0x67, 0x3d, 0x5f, 0x14);

static void gh_imm_hyp_identify(struct gh_hypercall_hyp_identify_resp *hyp_identity)
{
	register u64 out_x0 asm("x0") = 0;
	register u64 out_x1 asm("x1") = 0;
	register u64 out_x2 asm("x2") = 0;
	register u64 out_x3 asm("x3") = 0;

	asm volatile("hvc #0x6000\n"
		     : "=r"(out_x0), "=r"(out_x1), "=r"(out_x2), "=r"(out_x3)
		     :
		     : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
		       "x12", "x13", "x14", "x15", "x16", "x17", "memory");

	hyp_identity->api_info = out_x0;
	hyp_identity->flags[0] = out_x1;
	hyp_identity->flags[1] = out_x2;
	hyp_identity->flags[2] = out_x3;
}

static enum gh_hvc_abi gh_get_hvc_abi(void)
{
	struct arm_smccc_res res = {};
	struct gh_hypercall_hyp_identify_resp id = {};
	struct device_node *hyp_np;
	bool has_qcom_gunyah = false;
	uuid_t uuid;
	u32 *up;

	if (likely(gh_hvc_mode != GH_HVC_ABI_UNKNOWN))
		return gh_hvc_mode;

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_CALL_UID_FUNC_ID, &res);
	up = (u32 *)&uuid.b[0];
	up[0] = lower_32_bits(res.a0);
	up[1] = lower_32_bits(res.a1);
	up[2] = lower_32_bits(res.a2);
	up[3] = lower_32_bits(res.a3);
	if (uuid_equal(&uuid, &GUNYAH_UUID)) {
		gh_hvc_mode = GH_HVC_ABI_SMCCC;
		return gh_hvc_mode;
	}

	/*
	 * Production Qualcomm Gunyah firmware (e.g. TZ.XF.5.24 / QCS8550)
	 * exposes the native Gunyah immediate hypercall ABI (HVC #0x60xx)
	 * rather than the SMCCC wrapper ABI. Only probe HVC #0x6000 after
	 * confirming Gunyah presence via DT or QSEE HypBootInfo SMC.
	 */
	hyp_np = of_find_compatible_node(NULL, NULL, "qcom,gunyah-hypervisor");
	if (hyp_np) {
		has_qcom_gunyah = true;
		of_node_put(hyp_np);
	} else {
		struct device_node *scm_np;

		scm_np = of_find_compatible_node(NULL, NULL, "qcom,scm");
		if (scm_np) {
			of_node_put(scm_np);
			memset(&res, 0, sizeof(res));
			arm_smccc_smc(QCOM_HYP_INFO_GET_DTB_SMC32, 0, 0, 0, 0, 0, 0, 0, &res);
			if (!res.a0 && res.a1)
				has_qcom_gunyah = true;
		}
	}

	if (has_qcom_gunyah) {
		gh_imm_hyp_identify(&id);
		if (gh_api_version(&id) == GH_API_V1 &&
		    (id.api_info & GH_API_INFO_IS_64BIT)) {
			gh_hvc_mode = GH_HVC_ABI_IMM6000;
			return gh_hvc_mode;
		}
	}

	gh_hvc_mode = GH_HVC_ABI_NONE;
	return gh_hvc_mode;
}

bool arch_is_gh_guest(void)
{
	return gh_get_hvc_abi() != GH_HVC_ABI_NONE;
}
EXPORT_SYMBOL_GPL(arch_is_gh_guest);

#define GH_HYPERCALL(fn)	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
						   ARM_SMCCC_OWNER_VENDOR_HYP, \
						   fn)

#define GH_HYPERCALL_HYP_IDENTIFY		GH_HYPERCALL(0x8000)
#define GH_HYPERCALL_BELL_SEND			GH_HYPERCALL(0x8012)
#define GH_HYPERCALL_BELL_SET_MASK		GH_HYPERCALL(0x8015)
#define GH_HYPERCALL_MSGQ_SEND			GH_HYPERCALL(0x801B)
#define GH_HYPERCALL_MSGQ_RECV			GH_HYPERCALL(0x801C)
#define GH_HYPERCALL_VCPU_RUN			GH_HYPERCALL(0x8065)

void gh_hypercall_hyp_identify(struct gh_hypercall_hyp_identify_resp *hyp_identity)
{
	struct arm_smccc_res res;

	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		gh_imm_hyp_identify(hyp_identity);
		return;
	}

	arm_smccc_1_1_hvc(GH_HYPERCALL_HYP_IDENTIFY, &res);

	hyp_identity->api_info = res.a0;
	hyp_identity->flags[0] = res.a1;
	hyp_identity->flags[1] = res.a2;
	hyp_identity->flags[2] = res.a3;
}
EXPORT_SYMBOL_GPL(gh_hypercall_hyp_identify);

enum gh_error gh_hypercall_bell_send(u64 capid, u64 new_flags, u64 *old_flags)
{
	struct arm_smccc_res res;

	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		register u64 x0 asm("x0") = capid;
		register u64 x1 asm("x1") = new_flags;
		register u64 x2 asm("x2") = 0;

		asm volatile("hvc #0x6012\n"
			     : "+r"(x0), "+r"(x1), "+r"(x2)
			     :
			     : "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10",
			       "x11", "x12", "x13", "x14", "x15", "x16", "x17",
			       "memory");
		if ((s32)x0 == GH_ERROR_OK && old_flags)
			*old_flags = x1;
		return (enum gh_error)(s32)x0;
	}

	arm_smccc_1_1_hvc(GH_HYPERCALL_BELL_SEND, capid, new_flags, 0, &res);

	if (res.a0 == GH_ERROR_OK && old_flags)
		*old_flags = res.a1;

	return res.a0;
}
EXPORT_SYMBOL_GPL(gh_hypercall_bell_send);

enum gh_error gh_hypercall_bell_set_mask(u64 capid, u64 enable_mask, u64 ack_mask)
{
	struct arm_smccc_res res;

	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		register u64 x0 asm("x0") = capid;
		register u64 x1 asm("x1") = enable_mask;
		register u64 x2 asm("x2") = ack_mask;
		register u64 x3 asm("x3") = 0;

		asm volatile("hvc #0x6015\n"
			     : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			     :
			     : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
			       "x12", "x13", "x14", "x15", "x16", "x17", "memory");
		return (enum gh_error)(s32)x0;
	}

	arm_smccc_1_1_hvc(GH_HYPERCALL_BELL_SET_MASK, capid, enable_mask, ack_mask, 0, &res);

	return res.a0;
}
EXPORT_SYMBOL_GPL(gh_hypercall_bell_set_mask);

enum gh_error gh_hypercall_msgq_send(u64 capid, size_t size, void *buff, u64 tx_flags, bool *ready)
{
	struct arm_smccc_res res;

	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		register u64 x0 asm("x0") = capid;
		register u64 x1 asm("x1") = size;
		register u64 x2 asm("x2") = (uintptr_t)buff;
		register u64 x3 asm("x3") = tx_flags;
		register u64 x4 asm("x4") = 0;

		asm volatile("hvc #0x601b\n"
			     : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4)
			     :
			     : "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12",
			       "x13", "x14", "x15", "x16", "x17", "memory");
		if ((s32)x0 == GH_ERROR_OK)
			*ready = !!x1;
		return (enum gh_error)(s32)x0;
	}

	arm_smccc_1_1_hvc(GH_HYPERCALL_MSGQ_SEND, capid, size, (uintptr_t)buff, tx_flags, 0, &res);

	if (res.a0 == GH_ERROR_OK)
		*ready = !!res.a1;

	return res.a0;
}
EXPORT_SYMBOL_GPL(gh_hypercall_msgq_send);

enum gh_error gh_hypercall_msgq_recv(u64 capid, void *buff, size_t size, size_t *recv_size,
					bool *ready)
{
	struct arm_smccc_res res;

	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		register u64 x0 asm("x0") = capid;
		register u64 x1 asm("x1") = (uintptr_t)buff;
		register u64 x2 asm("x2") = size;
		register u64 x3 asm("x3") = 0;

		asm volatile("hvc #0x601c\n"
			     : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			     :
			     : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
			       "x12", "x13", "x14", "x15", "x16", "x17", "memory");
		if ((s32)x0 == GH_ERROR_OK) {
			*recv_size = x1;
			*ready = !!x2;
		}
		return (enum gh_error)(s32)x0;
	}

	arm_smccc_1_1_hvc(GH_HYPERCALL_MSGQ_RECV, capid, (uintptr_t)buff, size, 0, &res);

	if (res.a0 == GH_ERROR_OK) {
		*recv_size = res.a1;
		*ready = !!res.a2;
	}

	return res.a0;
}
EXPORT_SYMBOL_GPL(gh_hypercall_msgq_recv);

enum gh_error gh_hypercall_vcpu_run(u64 capid, u64 *resume_data,
					struct gh_hypercall_vcpu_run_resp *resp)
{
	if (gh_get_hvc_abi() == GH_HVC_ABI_IMM6000) {
		register u64 x0 asm("x0") = capid;
		register u64 x1 asm("x1") = resume_data[0];
		register u64 x2 asm("x2") = resume_data[1];
		register u64 x3 asm("x3") = resume_data[2];
		register u64 x4 asm("x4") = 0;

		asm volatile("hvc #0x6065\n"
			     : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4)
			     :
			     : "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12",
			       "x13", "x14", "x15", "x16", "x17", "memory");
		if ((s32)x0 == GH_ERROR_OK) {
			resp->sized_state = x1;
			resp->state_data[0] = x2;
			resp->state_data[1] = x3;
			resp->state_data[2] = x4;
		}
		return (enum gh_error)(s32)x0;
	}

	{
		struct arm_smccc_1_2_regs args = {
			.a0 = GH_HYPERCALL_VCPU_RUN,
			.a1 = capid,
			.a2 = resume_data[0],
			.a3 = resume_data[1],
			.a4 = resume_data[2],
			.a5 = 0,
		};
		struct arm_smccc_1_2_regs res;

		arm_smccc_1_2_hvc(&args, &res);

		if (res.a0 == GH_ERROR_OK) {
			resp->sized_state = res.a1;
			resp->state_data[0] = res.a2;
			resp->state_data[1] = res.a3;
			resp->state_data[2] = res.a4;
		}

		return res.a0;
	}
}
EXPORT_SYMBOL_GPL(gh_hypercall_vcpu_run);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Gunyah Hypervisor Hypercalls");
