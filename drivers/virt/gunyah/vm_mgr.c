// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "gh_vm_mgr: " fmt

#include <linux/anon_inodes.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/elf.h>
#include <linux/file.h>
#include <linux/gunyah_rsc_mgr.h>
#include <linux/gunyah_vm_mgr.h>
#include <linux/highmem.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/xarray.h>

#include <uapi/linux/gunyah.h>

#include "vm_mgr.h"

static void gh_vm_free(struct work_struct *work);

static DEFINE_XARRAY(gh_vm_functions);

static void gh_vm_put_function(struct gh_vm_function *fn)
{
	module_put(fn->mod);
}

static struct gh_vm_function *gh_vm_get_function(u32 type)
{
	struct gh_vm_function *fn;

	fn = xa_load(&gh_vm_functions, type);
	if (!fn) {
		request_module("ghfunc:%d", type);

		fn = xa_load(&gh_vm_functions, type);
	}

	if (!fn || !try_module_get(fn->mod))
		fn = ERR_PTR(-ENOENT);

	return fn;
}

static void gh_vm_remove_function_instance(struct gh_vm_function_instance *inst)
	__must_hold(&inst->ghvm->fn_lock)
{
	inst->fn->unbind(inst);
	list_del(&inst->vm_list);
	gh_vm_put_function(inst->fn);
	kfree(inst->argp);
	kfree(inst);
}

static void gh_vm_remove_functions(struct gh_vm *ghvm)
{
	struct gh_vm_function_instance *inst, *iiter;

	mutex_lock(&ghvm->fn_lock);
	list_for_each_entry_safe(inst, iiter, &ghvm->functions, vm_list) {
		gh_vm_remove_function_instance(inst);
	}
	mutex_unlock(&ghvm->fn_lock);
}

static long gh_vm_add_function_instance(struct gh_vm *ghvm, struct gh_fn_desc *f)
{
	struct gh_vm_function_instance *inst;
	void __user *argp;
	long r = 0;

	if (f->arg_size > GH_FN_MAX_ARG_SIZE) {
		dev_err_ratelimited(ghvm->parent, "%s: arg_size > %d\n",
					__func__, GH_FN_MAX_ARG_SIZE);
		return -EINVAL;
	}

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->arg_size = f->arg_size;
	if (inst->arg_size) {
		inst->argp = kzalloc(inst->arg_size, GFP_KERNEL);
		if (!inst->argp) {
			r = -ENOMEM;
			goto free;
		}

		argp = u64_to_user_ptr(f->arg);
		if (copy_from_user(inst->argp, argp, f->arg_size)) {
			r = -EFAULT;
			goto free_arg;
		}
	}

	inst->fn = gh_vm_get_function(f->type);
	if (IS_ERR(inst->fn)) {
		r = PTR_ERR(inst->fn);
		goto free_arg;
	}

	inst->ghvm = ghvm;
	inst->rm = ghvm->rm;

	mutex_lock(&ghvm->fn_lock);
	r = inst->fn->bind(inst);
	if (r < 0) {
		mutex_unlock(&ghvm->fn_lock);
		gh_vm_put_function(inst->fn);
		goto free_arg;
	}

	list_add(&inst->vm_list, &ghvm->functions);
	mutex_unlock(&ghvm->fn_lock);

	return r;
free_arg:
	kfree(inst->argp);
free:
	kfree(inst);
	return r;
}

static long gh_vm_rm_function_instance(struct gh_vm *ghvm, struct gh_fn_desc *f)
{
	struct gh_vm_function_instance *inst, *iter;
	void __user *user_argp;
	void *argp;
	long r = 0;

	r = mutex_lock_interruptible(&ghvm->fn_lock);
	if (r)
		return r;

	if (f->arg_size) {
		argp = kzalloc(f->arg_size, GFP_KERNEL);
		if (!argp) {
			r = -ENOMEM;
			goto out;
		}

		user_argp = u64_to_user_ptr(f->arg);
		if (copy_from_user(argp, user_argp, f->arg_size)) {
			r = -EFAULT;
			kfree(argp);
			goto out;
		}

		r = -ENOENT;
		list_for_each_entry_safe(inst, iter, &ghvm->functions, vm_list) {
			if (inst->fn->type == f->type &&
				inst->fn->compare(inst, argp, f->arg_size)) {
				gh_vm_remove_function_instance(inst);
				r = 0;
			}
		}

		kfree(argp);
	}

out:
	mutex_unlock(&ghvm->fn_lock);
	return r;
}

int gh_vm_function_register(struct gh_vm_function *fn)
{
	if (!fn->bind || !fn->unbind)
		return -EINVAL;

	return xa_err(xa_store(&gh_vm_functions, fn->type, fn, GFP_KERNEL));
}
EXPORT_SYMBOL_GPL(gh_vm_function_register);

void gh_vm_function_unregister(struct gh_vm_function *fn)
{
	/* Expecting unregister to only come when unloading a module */
	WARN_ON(fn->mod && module_refcount(fn->mod));
	xa_erase(&gh_vm_functions, fn->type);
}
EXPORT_SYMBOL_GPL(gh_vm_function_unregister);

int gh_vm_add_resource_ticket(struct gh_vm *ghvm, struct gh_vm_resource_ticket *ticket)
{
	struct gh_vm_resource_ticket *iter;
	struct gh_resource *ghrsc, *rsc_iter;
	int ret = 0;

	mutex_lock(&ghvm->resources_lock);
	list_for_each_entry(iter, &ghvm->resource_tickets, vm_list) {
		if (iter->resource_type == ticket->resource_type && iter->label == ticket->label) {
			ret = -EEXIST;
			goto out;
		}
	}

	if (!try_module_get(ticket->owner)) {
		ret = -ENODEV;
		goto out;
	}

	list_add(&ticket->vm_list, &ghvm->resource_tickets);
	INIT_LIST_HEAD(&ticket->resources);

	list_for_each_entry_safe(ghrsc, rsc_iter, &ghvm->resources, list) {
		if (ghrsc->type == ticket->resource_type && ghrsc->rm_label == ticket->label) {
			if (ticket->populate(ticket, ghrsc))
				list_move(&ghrsc->list, &ticket->resources);
		}
	}
out:
	mutex_unlock(&ghvm->resources_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gh_vm_add_resource_ticket);

void gh_vm_remove_resource_ticket(struct gh_vm *ghvm, struct gh_vm_resource_ticket *ticket)
{
	struct gh_resource *ghrsc, *iter;

	mutex_lock(&ghvm->resources_lock);
	list_for_each_entry_safe(ghrsc, iter, &ticket->resources, list) {
		ticket->unpopulate(ticket, ghrsc);
		list_move(&ghrsc->list, &ghvm->resources);
	}

	module_put(ticket->owner);
	list_del(&ticket->vm_list);
	mutex_unlock(&ghvm->resources_lock);
}
EXPORT_SYMBOL_GPL(gh_vm_remove_resource_ticket);

static void gh_vm_add_resource(struct gh_vm *ghvm, struct gh_resource *ghrsc)
{
	struct gh_vm_resource_ticket *ticket;

	mutex_lock(&ghvm->resources_lock);
	list_for_each_entry(ticket, &ghvm->resource_tickets, vm_list) {
		if (ghrsc->type == ticket->resource_type && ghrsc->rm_label == ticket->label) {
			if (ticket->populate(ticket, ghrsc))
				list_add(&ghrsc->list, &ticket->resources);
			else
				list_add(&ghrsc->list, &ghvm->resources);
			/* unconditonal -- we prevent multiple identical
			 * resource tickets so there will not be some other
			 * ticket elsewhere in the list if populate() failed.
			 */
			goto found;
		}
	}
	list_add(&ghrsc->list, &ghvm->resources);
found:
	mutex_unlock(&ghvm->resources_lock);
}

static void gh_vm_clean_resources(struct gh_vm *ghvm)
{
	struct gh_vm_resource_ticket *ticket, *titer;
	struct gh_resource *ghrsc, *riter;

	mutex_lock(&ghvm->resources_lock);
	if (!list_empty(&ghvm->resource_tickets)) {
		dev_warn(ghvm->parent, "Dangling resource tickets:\n");
		list_for_each_entry_safe(ticket, titer, &ghvm->resource_tickets, vm_list) {
			dev_warn(ghvm->parent, "  %pS\n", ticket->populate);
			gh_vm_remove_resource_ticket(ghvm, ticket);
		}
	}

	list_for_each_entry_safe(ghrsc, riter, &ghvm->resources, list) {
		gh_rm_free_resource(ghrsc);
	}
	mutex_unlock(&ghvm->resources_lock);
}

static int _gh_vm_io_handler_compare(const struct rb_node *node, const struct rb_node *parent)
{
	struct gh_vm_io_handler *n = container_of(node, struct gh_vm_io_handler, node);
	struct gh_vm_io_handler *p = container_of(parent, struct gh_vm_io_handler, node);

	if (n->addr < p->addr)
		return -1;
	if (n->addr > p->addr)
		return 1;
	if ((n->len && !p->len) || (!n->len && p->len))
		return 0;
	if (n->len < p->len)
		return -1;
	if (n->len > p->len)
		return 1;
	/* one of the io handlers doesn't have datamatch and the other does.
	 * For purposes of comparison, that makes them identical since the
	 * one that doesn't have datamatch will cover the same handler that
	 * does.
	 */
	if (n->datamatch != p->datamatch)
		return 0;
	if (n->data < p->data)
		return -1;
	if (n->data > p->data)
		return 1;
	return 0;
}

static int gh_vm_io_handler_compare(struct rb_node *node, const struct rb_node *parent)
{
	return _gh_vm_io_handler_compare(node, parent);
}

static int gh_vm_io_handler_find(const void *key, const struct rb_node *node)
{
	const struct gh_vm_io_handler *k = key;

	return _gh_vm_io_handler_compare(&k->node, node);
}

static struct gh_vm_io_handler *gh_vm_mgr_find_io_hdlr(struct gh_vm *ghvm, u64 addr,
								u64 len, u64 data)
{
	struct gh_vm_io_handler key = {
		.addr = addr,
		.len = len,
		.datamatch = true,
		.data = data,
	};
	struct rb_node *node;

	node = rb_find(&key, &ghvm->mmio_handler_root, gh_vm_io_handler_find);
	if (!node)
		return NULL;

	return container_of(node, struct gh_vm_io_handler, node);
}

int gh_vm_mmio_write(struct gh_vm *ghvm, u64 addr, u32 len, u64 data)
{
	struct gh_vm_io_handler *io_hdlr = NULL;
	int ret;

	down_read(&ghvm->mmio_handler_lock);
	io_hdlr = gh_vm_mgr_find_io_hdlr(ghvm, addr, len, data);
	if (!io_hdlr || !io_hdlr->ops || !io_hdlr->ops->write) {
		ret = -ENODEV;
		goto out;
	}

	ret = io_hdlr->ops->write(io_hdlr, addr, len, data);

out:
	up_read(&ghvm->mmio_handler_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gh_vm_mmio_write);

int gh_vm_add_io_handler(struct gh_vm *ghvm, struct gh_vm_io_handler *io_hdlr)
{
	struct rb_node *found;

	if (io_hdlr->datamatch && (!io_hdlr->len || io_hdlr->len > sizeof(io_hdlr->data)))
		return -EINVAL;

	down_write(&ghvm->mmio_handler_lock);
	found = rb_find_add(&io_hdlr->node, &ghvm->mmio_handler_root, gh_vm_io_handler_compare);
	up_write(&ghvm->mmio_handler_lock);

	return found ? -EEXIST : 0;
}
EXPORT_SYMBOL_GPL(gh_vm_add_io_handler);

void gh_vm_remove_io_handler(struct gh_vm *ghvm, struct gh_vm_io_handler *io_hdlr)
{
	down_write(&ghvm->mmio_handler_lock);
	rb_erase(&io_hdlr->node, &ghvm->mmio_handler_root);
	up_write(&ghvm->mmio_handler_lock);
}
EXPORT_SYMBOL_GPL(gh_vm_remove_io_handler);

static int gh_vm_rm_notification_status(struct gh_vm *ghvm, void *data)
{
	struct gh_rm_vm_status_payload *payload = data;

	if (le16_to_cpu(payload->vmid) != ghvm->vmid)
		return NOTIFY_OK;

	/* All other state transitions are synchronous to a corresponding RM call */
	if (payload->vm_status == GH_RM_VM_STATUS_RESET) {
		down_write(&ghvm->status_lock);
		ghvm->vm_status = payload->vm_status;
		up_write(&ghvm->status_lock);
		wake_up(&ghvm->vm_status_wait);
	} else if (payload->vm_status == GH_RM_VM_STATUS_EXITED ||
		   (payload->os_status == 4 &&
		    le16_to_cpu(payload->app_status) == 0xdead)) {
		/* Allow Guest EL1 time to issue final PSCI SYSTEM_OFF after status RPC */
		msleep(50);
		down_write(&ghvm->status_lock);
		ghvm->vm_status = GH_RM_VM_STATUS_EXITED;
		up_write(&ghvm->status_lock);
		wake_up(&ghvm->vm_status_wait);
		wake_up_interruptible(&ghvm->cons_wait);
	}

	return NOTIFY_DONE;
}

static int gh_vm_rm_notification_exited(struct gh_vm *ghvm, void *data)
{
	struct gh_rm_vm_exited_payload *payload = data;

	if (le16_to_cpu(payload->vmid) != ghvm->vmid)
		return NOTIFY_OK;

	down_write(&ghvm->status_lock);
	ghvm->vm_status = GH_RM_VM_STATUS_EXITED;
	ghvm->exit_info.type = le16_to_cpu(payload->exit_type);
	ghvm->exit_info.reason_size = le32_to_cpu(payload->exit_reason_size);
	memcpy(&ghvm->exit_info.reason, payload->exit_reason,
		min(GH_VM_MAX_EXIT_REASON_SIZE, ghvm->exit_info.reason_size));
	up_write(&ghvm->status_lock);
	wake_up(&ghvm->vm_status_wait);
	wake_up_interruptible(&ghvm->cons_wait);

	return NOTIFY_DONE;
}

static int gh_vm_rm_notification_console(struct gh_vm *ghvm, void *data)
{
	struct gh_rm_vm_console_chars_payload *payload = data;
	unsigned long flags;
	u16 num_bytes;

	if (le16_to_cpu(payload->vmid) != ghvm->vmid)
		return NOTIFY_OK;

	num_bytes = le16_to_cpu(payload->num_bytes);
	if (num_bytes) {
		spin_lock_irqsave(&ghvm->cons_lock, flags);
		kfifo_in(&ghvm->cons_fifo, payload->data, num_bytes);
		spin_unlock_irqrestore(&ghvm->cons_lock, flags);
		wake_up_interruptible(&ghvm->cons_wait);
	}

	return NOTIFY_OK;
}

static int gh_vm_rm_notification(struct notifier_block *nb, unsigned long action, void *data)
{
	struct gh_vm *ghvm = container_of(nb, struct gh_vm, nb);

	switch (action) {
	case GH_RM_NOTIFICATION_VM_STATUS:
		return gh_vm_rm_notification_status(ghvm, data);
	case GH_RM_NOTIFICATION_VM_EXITED:
		return gh_vm_rm_notification_exited(ghvm, data);
	case GH_RM_NOTIFICATION_VM_CONSOLE_CHARS:
		return gh_vm_rm_notification_console(ghvm, data);
	default:
		return NOTIFY_OK;
	}
}

static void gh_vm_stop(struct gh_vm *ghvm)
{
	int ret = 0;

	down_write(&ghvm->status_lock);
	if (ghvm->vm_status == GH_RM_VM_STATUS_RUNNING) {
		ret = gh_rm_vm_stop(ghvm->rm, ghvm->vmid);
		if (ret)
			dev_warn(ghvm->parent, "Failed to stop VM: %d\n", ret);
	}
	up_write(&ghvm->status_lock);

	if (!ret)
		wait_event_timeout(ghvm->vm_status_wait,
				   ghvm->vm_status == GH_RM_VM_STATUS_EXITED,
				   msecs_to_jiffies(5000));
}

static __must_check struct gh_vm *gh_vm_alloc(struct gh_rm *rm)
{
	struct gh_vm *ghvm;

	ghvm = kzalloc(sizeof(*ghvm), GFP_KERNEL);
	if (!ghvm)
		return ERR_PTR(-ENOMEM);

	ghvm->parent = gh_rm_get(rm);
	ghvm->vmid = GH_VMID_INVAL;
	ghvm->rm = rm;

	mmgrab(current->mm);
	ghvm->mm = current->mm;
	mutex_init(&ghvm->mm_lock);
	INIT_LIST_HEAD(&ghvm->memory_mappings);
	init_rwsem(&ghvm->status_lock);
	init_waitqueue_head(&ghvm->vm_status_wait);
	INIT_WORK(&ghvm->free_work, gh_vm_free);
	kref_init(&ghvm->kref);
	mutex_init(&ghvm->resources_lock);
	INIT_LIST_HEAD(&ghvm->resources);
	INIT_LIST_HEAD(&ghvm->resource_tickets);
	init_rwsem(&ghvm->mmio_handler_lock);
	ghvm->mmio_handler_root = RB_ROOT;
	mutex_init(&ghvm->fn_lock);
	INIT_LIST_HEAD(&ghvm->functions);
	spin_lock_init(&ghvm->cons_lock);
	INIT_KFIFO(ghvm->cons_fifo);
	init_waitqueue_head(&ghvm->cons_wait);
	ghvm->vm_status = GH_RM_VM_STATUS_NO_STATE;

	return ghvm;
}

#define GH_QCOM_MDT_TYPE_MASK		(7 << 24)
#define GH_QCOM_MDT_TYPE_HASH		(2 << 24)

static void gh_vm_autodetect_qcom_pil(struct gh_vm *ghvm)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_hdr *ehdr;
	struct gh_vm_mem *mapping;
	unsigned long p, cand_pages[2];
	int n_cand, c, i;

	if (ghvm->auth != GH_RM_VM_AUTH_NONE)
		return;

	mutex_lock(&ghvm->mm_lock);
	list_for_each_entry(mapping, &ghvm->memory_mappings, list) {
		if (mapping->share_type != VM_MEM_LEND || !mapping->npages)
			continue;

		cand_pages[0] = mapping->npages - 1;
		cand_pages[1] = 0;
		n_cand = mapping->npages > 1 ? 2 : 1;

		for (c = 0; c < n_cand; c++) {
			u32 ehdr_sz = 0, hash_sz = 0;
			const u8 *buf;

			p = cand_pages[c];
			buf = page_address(mapping->pages[p]);
			ehdr = (const struct elf32_hdr *)buf;
			if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0 ||
			    ehdr->e_ident[EI_CLASS] != ELFCLASS32 ||
			    ehdr->e_phentsize != sizeof(struct elf32_phdr) ||
			    ehdr->e_phnum < 3 ||
			    ehdr->e_phoff + ehdr->e_phnum * sizeof(struct elf32_phdr) > PAGE_SIZE)
				continue;

			phdrs = (const struct elf32_phdr *)(buf + ehdr->e_phoff);
			ehdr_sz = phdrs[0].p_filesz;
			for (i = 1; i < ehdr->e_phnum; i++) {
				if ((phdrs[i].p_flags & GH_QCOM_MDT_TYPE_MASK) ==
				    GH_QCOM_MDT_TYPE_HASH) {
					hash_sz = phdrs[i].p_filesz;
					break;
				}
			}
			if (!ehdr_sz || !hash_sz || ehdr_sz + 0x38 > PAGE_SIZE)
				continue;

			if (le32_to_cpup((const __le32 *)(buf + ehdr_sz + 4)) == 7) {
				u32 sw_id = le32_to_cpup((const __le32 *)(buf + ehdr_sz + 0x30));

				ghvm->auth = GH_RM_VM_AUTH_QCOM_PIL_ELF;
				ghvm->fw_config.guest_phys_addr =
					mapping->guest_phys_addr + ((u64)p << PAGE_SHIFT);
				ghvm->fw_config.size = PAGE_ALIGN(ehdr_sz + hash_sz);
				ghvm->has_fw_config = true;
				if (!ghvm->pas_id) {
					if (sw_id == 0x4c) {
						ghvm->requested_vmid = 0x31;
						ghvm->pas_id = 0x22;
					} else if (sw_id == 0x2f) {
						ghvm->requested_vmid = 0x2d;
						ghvm->pas_id = 0x1c;
					}
				}
				dev_info(ghvm->parent,
					 "Auto-detected QCOM PIL_ELF VM (sw_id=0x%x vmid=0x%x pas_id=0x%x meta=%lluB @ +0x%llx)\n",
					 sw_id, ghvm->requested_vmid, ghvm->pas_id,
					 ghvm->fw_config.size,
					 (u64)p << PAGE_SHIFT);
				goto out_unlock;
			}
		}
	}
out_unlock:
	mutex_unlock(&ghvm->mm_lock);
}

static int gh_vm_start(struct gh_vm *ghvm)
{
	struct gh_vm_mem *mapping;
	struct gh_rm_hyp_resources *resources;
	struct gh_resource *ghrsc;
	u64 dtb_offset, dtb_size, image_offset = 0, image_size = 0;
	u32 mem_handle;
	int ret, i, n;

	down_write(&ghvm->status_lock);
	if (ghvm->vm_status != GH_RM_VM_STATUS_NO_STATE) {
		up_write(&ghvm->status_lock);
		return 0;
	}

	gh_vm_autodetect_qcom_pil(ghvm);

	ghvm->nb.notifier_call = gh_vm_rm_notification;
	ret = gh_rm_notifier_register(ghvm->rm, &ghvm->nb);
	if (ret)
		goto err;

	ret = gh_rm_alloc_vmid(ghvm->rm, ghvm->requested_vmid);
	if (ret == -ENODEV && ghvm->auth == GH_RM_VM_AUTH_QCOM_PIL_ELF &&
	    ghvm->pas_id == 0x22 && ghvm->requested_vmid == 0x31) {
		static const u16 oem_vmid_pool[] = { 0x32, 0x34, 0x35 };
		unsigned int k;

		for (k = 0; k < ARRAY_SIZE(oem_vmid_pool); k++) {
			ret = gh_rm_alloc_vmid(ghvm->rm, oem_vmid_pool[k]);
			if (ret >= 0) {
				dev_info(ghvm->parent,
					 "Selected next available OEM_VM slot vmid=0x%x\n",
					 ret);
				break;
			}
		}
	}
	if (ret < 0) {
		gh_rm_notifier_unregister(ghvm->rm, &ghvm->nb);
		goto err;
	}
	ghvm->vmid = ret;
	ghvm->vm_status = GH_RM_VM_STATUS_LOAD;

	mutex_lock(&ghvm->mm_lock);
	list_for_each_entry(mapping, &ghvm->memory_mappings, list) {
		mapping->parcel.acl_entries[0].vmid = cpu_to_le16(ghvm->vmid);
		switch (mapping->share_type) {
		case VM_MEM_LEND:
			gh_vm_mem_sync_lend(ghvm, mapping);
			ret = gh_rm_mem_lend(ghvm->rm, &mapping->parcel);
			break;
		case VM_MEM_SHARE:
			ret = gh_rm_mem_share(ghvm->rm, &mapping->parcel);
			break;
		}
		if (ret) {
			dev_warn(ghvm->parent, "Failed to %s parcel %d: %d\n",
				mapping->share_type == VM_MEM_LEND ? "lend" : "share",
				mapping->parcel.label,
				ret);
			mutex_unlock(&ghvm->mm_lock);
			goto err;
		}
	}
	mutex_unlock(&ghvm->mm_lock);

	if (ghvm->dtb_config.size) {
		mapping = gh_vm_mem_find_by_addr(ghvm,
						 ghvm->dtb_config.guest_phys_addr,
						 ghvm->dtb_config.size);
	} else {
		mutex_lock(&ghvm->mm_lock);
		mapping = list_first_entry_or_null(&ghvm->memory_mappings,
						   struct gh_vm_mem, list);
		mutex_unlock(&ghvm->mm_lock);
	}
	if (!mapping) {
		dev_warn(ghvm->parent, "Failed to find the memory_handle for DTB\n");
		ret = -EINVAL;
		goto err;
	}

	mem_handle = mapping->parcel.mem_handle;
	if (ghvm->auth == GH_RM_VM_AUTH_QCOM_PIL_ELF) {
		dtb_offset = 0;
		dtb_size = 0;
	} else {
		dtb_offset = ghvm->dtb_config.guest_phys_addr -
			     mapping->guest_phys_addr;
		dtb_size = ghvm->dtb_config.size;
	}

	if (ghvm->has_fw_config) {
		if (ghvm->fw_config.guest_phys_addr >= mapping->guest_phys_addr)
			image_offset = ghvm->fw_config.guest_phys_addr -
				       mapping->guest_phys_addr;
		else
			image_offset = ghvm->fw_config.guest_phys_addr;
		image_size = PAGE_ALIGN(ghvm->fw_config.size);
	}

	ret = gh_rm_vm_configure(ghvm->rm, ghvm->vmid, ghvm->auth, mem_handle,
				image_offset, image_size,
				dtb_offset, dtb_size);
	if (ret) {
		dev_warn(ghvm->parent, "Failed to configure VM: %d\n", ret);
		goto err;
	}

	if (ghvm->auth == GH_RM_VM_AUTH_QCOM_PIL_ELF) {
		ghvm->vm_status = GH_RM_VM_STATUS_AUTH;
		ret = gh_rm_vm_auth_image(ghvm->rm, ghvm->vmid, ghvm->pas_id);
		if (ret) {
			ghvm->vm_status = GH_RM_VM_STATUS_INIT_FAILED;
			dev_warn(ghvm->parent, "Failed to authenticate VM image (vmid=0x%x pas=0x%x): %d\n",
				 ghvm->vmid, ghvm->pas_id, ret);
			goto err;
		}
	}

	ret = gh_rm_vm_init(ghvm->rm, ghvm->vmid);
	if (ret) {
		ghvm->vm_status = GH_RM_VM_STATUS_INIT_FAILED;
		dev_warn(ghvm->parent, "Failed to initialize VM: %d\n", ret);
		goto err;
	}
	ghvm->vm_status = GH_RM_VM_STATUS_READY;

	{
		int cons_ret = gh_rm_console_open(ghvm->rm, ghvm->vmid);

		if (!cons_ret)
			ghvm->cons_open = true;
		else
			dev_info(ghvm->parent, "RM console_open(vmid=0x%x) returned %d\n",
				 ghvm->vmid, cons_ret);
	}

	ret = gh_rm_get_hyp_resources(ghvm->rm, ghvm->vmid, &resources);
	if (ret) {
		dev_warn(ghvm->parent, "Failed to get hypervisor resources for VM: %d\n", ret);
		goto err;
	}

	for (i = 0, n = le32_to_cpu(resources->n_entries); i < n; i++) {
		ghrsc = gh_rm_alloc_resource(ghvm->rm, &resources->entries[i]);
		if (!ghrsc) {
			kfree(resources);
			ret = -ENOMEM;
			goto err;
		}

		gh_vm_add_resource(ghvm, ghrsc);
	}
	kfree(resources);

	ret = gh_rm_vm_start(ghvm->rm, ghvm->vmid);
	if (ret) {
		dev_warn(ghvm->parent, "Failed to start VM: %d\n", ret);
		goto err;
	}

	ghvm->vm_status = GH_RM_VM_STATUS_RUNNING;
	up_write(&ghvm->status_lock);
	return ret;
err:
	/* gh_vm_free will handle releasing resources and reclaiming memory */
	up_write(&ghvm->status_lock);
	return ret;
}

static int gh_vm_ensure_started(struct gh_vm *ghvm)
{
	int ret;

	ret = down_read_interruptible(&ghvm->status_lock);
	if (ret)
		return ret;

	/* Unlikely because VM is typically started */
	if (unlikely(ghvm->vm_status == GH_RM_VM_STATUS_NO_STATE)) {
		up_read(&ghvm->status_lock);
		ret = gh_vm_start(ghvm);
		if (ret)
			return ret;
		/** gh_vm_start() is guaranteed to bring status out of
		 * GH_RM_VM_STATUS_LOAD, thus infinitely recursive call is not
		 * possible
		 */
		return gh_vm_ensure_started(ghvm);
	}

	/* Unlikely because VM is typically running */
	if (unlikely(ghvm->vm_status != GH_RM_VM_STATUS_RUNNING))
		ret = -ENODEV;

	up_read(&ghvm->status_lock);
	return ret;
}

static long gh_vm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gh_vm *ghvm = filp->private_data;
	void __user *argp = (void __user *)arg;
	long r;
	bool lend = false;

	switch (cmd) {
	case GH_VM_ANDROID_LEND_USER_MEM:
		lend = true;
		fallthrough;
	case GH_VM_SET_USER_MEM_REGION: {
		struct gh_userspace_memory_region region;

		/* only allow owner task to add memory */
		if (ghvm->mm != current->mm)
			return -EPERM;

		if (copy_from_user(&region, argp, sizeof(region)))
			return -EFAULT;

		/* All other flag bits are reserved for future use */
		if (region.flags & ~(GH_MEM_ALLOW_READ | GH_MEM_ALLOW_WRITE | GH_MEM_ALLOW_EXEC))
			return -EINVAL;

		r = gh_vm_mem_alloc(ghvm, &region, lend);
		break;
	}
	case GH_VM_SET_DTB_CONFIG: {
		struct gh_vm_dtb_config dtb_config;

		if (copy_from_user(&dtb_config, argp, sizeof(dtb_config)))
			return -EFAULT;

		if (overflows_type(dtb_config.guest_phys_addr + dtb_config.size, u64))
			return -EOVERFLOW;

		ghvm->dtb_config = dtb_config;

		r = 0;
		break;
	}
	case GH_VM_ANDROID_SET_FW_CONFIG: {
		struct gh_vm_firmware_config fw_config;

		if (copy_from_user(&fw_config, argp, sizeof(fw_config)))
			return -EFAULT;

		if (overflows_type(fw_config.guest_phys_addr + fw_config.size, u64))
			return -EOVERFLOW;

		ghvm->fw_config = fw_config;
		ghvm->has_fw_config = true;
		if (ghvm->auth == GH_RM_VM_AUTH_NONE)
			ghvm->auth = GH_RM_VM_AUTH_QCOM_ANDROID_PVM;

		r = 0;
		break;
	}
	case GH_VM_QCOM_SET_AUTH_CONFIG: {
		struct gh_vm_qcom_auth_config auth_cfg;

		if (copy_from_user(&auth_cfg, argp, sizeof(auth_cfg)))
			return -EFAULT;

		if (auth_cfg.auth_mech > GH_RM_VM_AUTH_QCOM_ANDROID_PVM)
			return -EINVAL;

		ghvm->requested_vmid = auth_cfg.vmid;
		ghvm->auth = (enum gh_rm_vm_auth_mechanism)auth_cfg.auth_mech;
		ghvm->pas_id = auth_cfg.pas_id;
		ghvm->fw_config.guest_phys_addr = auth_cfg.image_offset;
		ghvm->fw_config.size = auth_cfg.image_size;
		ghvm->has_fw_config = (auth_cfg.image_size != 0);

		r = 0;
		break;
	}
	case GH_VM_START: {
		r = gh_vm_ensure_started(ghvm);
		break;
	}
	case GH_VM_ADD_FUNCTION: {
		struct gh_fn_desc f;

		if (copy_from_user(&f, argp, sizeof(f)))
			return -EFAULT;

		r = gh_vm_add_function_instance(ghvm, &f);
		break;
	}
	case GH_VM_REMOVE_FUNCTION: {
		struct gh_fn_desc f;

		if (copy_from_user(&f, argp, sizeof(f)))
			return -EFAULT;

		r = gh_vm_rm_function_instance(ghvm, &f);
		break;
	}
	default:
		r = -ENOTTY;
		break;
	}

	return r;
}

static void gh_vm_free(struct work_struct *work)
{
	struct gh_vm *ghvm = container_of(work, struct gh_vm, free_work);
	int ret;

	if (ghvm->vm_status == GH_RM_VM_STATUS_RUNNING)
		gh_vm_stop(ghvm);

	gh_vm_remove_functions(ghvm);
	gh_vm_clean_resources(ghvm);

	if (ghvm->cons_open) {
		gh_rm_console_close(ghvm->rm, ghvm->vmid);
		ghvm->cons_open = false;
	}

	if (ghvm->vm_status != GH_RM_VM_STATUS_NO_STATE &&
	    ghvm->vm_status != GH_RM_VM_STATUS_LOAD &&
	    ghvm->vm_status != GH_RM_VM_STATUS_RESET) {
		ret = gh_rm_vm_reset(ghvm->rm, ghvm->vmid);
		if (!ret)
			wait_event_timeout(ghvm->vm_status_wait,
					   ghvm->vm_status == GH_RM_VM_STATUS_RESET,
					   msecs_to_jiffies(5000));
		else if (ret != -EOPNOTSUPP)
			dev_err(ghvm->parent, "Failed to reset the vm: %d\n", ret);
	}

	gh_vm_mem_reclaim(ghvm);

	if (ghvm->vm_status > GH_RM_VM_STATUS_NO_STATE) {
		gh_rm_notifier_unregister(ghvm->rm, &ghvm->nb);

		ret = gh_rm_dealloc_vmid(ghvm->rm, ghvm->vmid);
		if (ret && ret != -EOPNOTSUPP)
			dev_warn(ghvm->parent, "Failed to deallocate vmid: %d\n", ret);
	}

	gh_rm_put(ghvm->rm);
	mmdrop(ghvm->mm);
	kfree(ghvm);
}

int __must_check gh_vm_get(struct gh_vm *ghvm)
{
	return kref_get_unless_zero(&ghvm->kref);
}
EXPORT_SYMBOL_GPL(gh_vm_get);

static void _gh_vm_put(struct kref *kref)
{
	struct gh_vm *ghvm = container_of(kref, struct gh_vm, kref);

	/* VM will be reset and make RM calls which can interruptible sleep.
	 * Defer to a work so this thread can receive signal.
	 */
	schedule_work(&ghvm->free_work);
}

void gh_vm_put(struct gh_vm *ghvm)
{
	kref_put(&ghvm->kref, _gh_vm_put);
}
EXPORT_SYMBOL_GPL(gh_vm_put);

static ssize_t gh_vm_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct gh_vm *ghvm = filp->private_data;
	unsigned long flags;
	unsigned int n;
	u8 tmp[256];
	int ret;

	if (!count)
		return 0;

	for (;;) {
		spin_lock_irqsave(&ghvm->cons_lock, flags);
		n = kfifo_out(&ghvm->cons_fifo, tmp, min_t(size_t, count, sizeof(tmp)));
		spin_unlock_irqrestore(&ghvm->cons_lock, flags);
		if (n)
			break;

		if (ghvm->vm_status == GH_RM_VM_STATUS_EXITED ||
		    ghvm->vm_status == GH_RM_VM_STATUS_PAUSED ||
		    ghvm->vm_status == GH_RM_VM_STATUS_INIT_FAILED ||
		    ghvm->vm_status == GH_RM_VM_STATUS_RESET)
			return 0;

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(ghvm->cons_wait,
					       !kfifo_is_empty(&ghvm->cons_fifo) ||
					       ghvm->vm_status == GH_RM_VM_STATUS_EXITED ||
					       ghvm->vm_status == GH_RM_VM_STATUS_PAUSED ||
					       ghvm->vm_status == GH_RM_VM_STATUS_INIT_FAILED ||
					       ghvm->vm_status == GH_RM_VM_STATUS_RESET);
		if (ret)
			return ret;
	}

	if (copy_to_user(buf, tmp, n))
		return -EFAULT;
	return (ssize_t)n;
}

static __poll_t gh_vm_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct gh_vm *ghvm = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &ghvm->cons_wait, wait);

	if (!kfifo_is_empty(&ghvm->cons_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (ghvm->vm_status == GH_RM_VM_STATUS_EXITED ||
	    ghvm->vm_status == GH_RM_VM_STATUS_INIT_FAILED ||
	    ghvm->vm_status == GH_RM_VM_STATUS_RESET)
		mask |= EPOLLHUP;

	return mask;
}

static int gh_vm_release(struct inode *inode, struct file *filp)
{
	struct gh_vm *ghvm = filp->private_data;

	gh_vm_put(ghvm);
	return 0;
}

static const struct file_operations gh_vm_fops = {
	.owner = THIS_MODULE,
	.read = gh_vm_read,
	.poll = gh_vm_poll,
	.unlocked_ioctl = gh_vm_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.release = gh_vm_release,
	.llseek = noop_llseek,
};

static long gh_dev_ioctl_create_vm(struct gh_rm *rm, unsigned long arg)
{
	struct gh_vm *ghvm;
	struct file *file;
	int fd, err;

	/* arg reserved for future use. */
	if (arg)
		return -EINVAL;

	ghvm = gh_vm_alloc(rm);
	if (IS_ERR(ghvm))
		return PTR_ERR(ghvm);

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		err = fd;
		goto err_destroy_vm;
	}

	file = anon_inode_getfile("gunyah-vm", &gh_vm_fops, ghvm, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto err_put_fd;
	}

	fd_install(fd, file);

	return fd;

err_put_fd:
	put_unused_fd(fd);
err_destroy_vm:
	gh_vm_put(ghvm);
	return err;
}

long gh_dev_vm_mgr_ioctl(struct gh_rm *rm, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case GH_CREATE_VM:
		return gh_dev_ioctl_create_vm(rm, arg);
	default:
		return -ENOTTY;
	}
}
