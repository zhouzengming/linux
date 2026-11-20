// SPDX-License-Identifier: GPL-2.0
/*
 * Device Tree Overlay ConfigFS interface (/sys/kernel/config/device-tree/overlays)
 *
 * Allows dynamic runtime loading and unloading of compiled .dtbo Device Tree
 * overlays via configfs (compatible with Raspberry Pi dtoverlay convention):
 *   mkdir /sys/kernel/config/device-tree/overlays/<name>
 *   cat foo.dtbo > /sys/kernel/config/device-tree/overlays/<name>/dtbo
 *   # or: echo fv01-overlays/foo.dtbo > .../path
 *   cat /sys/kernel/config/device-tree/overlays/<name>/status
 *   rmdir /sys/kernel/config/device-tree/overlays/<name>
 */

#include <linux/configfs.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>

struct cfs_overlay_item {
	struct config_item	item;
	struct mutex		lock;
	char			path[PATH_MAX];
	const struct firmware	*fw;
	void			*dtbo;
	size_t			dtbo_size;
	int			ov_id;
};

static struct platform_device *of_cfs_fw_pdev;
static struct config_group *overlays_group;

static inline struct cfs_overlay_item *to_cfs_overlay_item(struct config_item *item)
{
	return item ? container_of(item, struct cfs_overlay_item, item) : NULL;
}

static int create_overlay_locked(struct cfs_overlay_item *overlay,
				 const void *blob, size_t size)
{
	int err;

	if (overlay->ov_id >= 0)
		return -EEXIST;
	if (!blob || size == 0 || size > SZ_1M)
		return -EINVAL;

	err = of_overlay_fdt_apply(blob, size, &overlay->ov_id, NULL);
	if (err) {
		overlay->ov_id = -1;
		pr_err("of_configfs: failed to apply overlay '%s': %d\n",
		       config_item_name(&overlay->item), err);
		return err;
	}

	pr_info("of_configfs: applied overlay '%s' (ov_id=%d, %zu bytes)\n",
		config_item_name(&overlay->item), overlay->ov_id, size);
	return 0;
}

static void destroy_overlay_locked(struct cfs_overlay_item *overlay)
{
	if (overlay->ov_id >= 0) {
		int ov_id = overlay->ov_id;

		of_overlay_remove(&overlay->ov_id);
		overlay->ov_id = -1;
		pr_info("of_configfs: removed overlay '%s' (ov_id=%d)\n",
			config_item_name(&overlay->item), ov_id);
	}
	if (overlay->fw) {
		release_firmware(overlay->fw);
		overlay->fw = NULL;
	}
	if (overlay->dtbo) {
		kfree(overlay->dtbo);
		overlay->dtbo = NULL;
		overlay->dtbo_size = 0;
	}
	overlay->path[0] = '\0';
}

static ssize_t cfs_overlay_item_path_show(struct config_item *item, char *page)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);
	ssize_t ret;

	mutex_lock(&overlay->lock);
	ret = sysfs_emit(page, "%s\n", overlay->path);
	mutex_unlock(&overlay->lock);
	return ret;
}

static ssize_t cfs_overlay_item_path_store(struct config_item *item,
					   const char *page, size_t count)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);
	const struct firmware *fw = NULL;
	char *path_buf;
	size_t len;
	int err;

	if (count == 0 || count >= PATH_MAX)
		return -EINVAL;

	path_buf = kmemdup_nul(page, count, GFP_KERNEL);
	if (!path_buf)
		return -ENOMEM;

	len = strlen(path_buf);
	while (len > 0 && (path_buf[len - 1] == '\n' || path_buf[len - 1] == '\r'))
		path_buf[--len] = '\0';

	if (len == 0) {
		kfree(path_buf);
		return -EINVAL;
	}

	mutex_lock(&overlay->lock);
	if (overlay->ov_id >= 0 || overlay->dtbo || overlay->fw) {
		mutex_unlock(&overlay->lock);
		kfree(path_buf);
		return -EEXIST;
	}

	err = request_firmware(&fw, path_buf, &of_cfs_fw_pdev->dev);
	if (err) {
		mutex_unlock(&overlay->lock);
		kfree(path_buf);
		return err;
	}

	err = create_overlay_locked(overlay, fw->data, fw->size);
	if (err) {
		release_firmware(fw);
		mutex_unlock(&overlay->lock);
		kfree(path_buf);
		return err;
	}

	overlay->fw = fw;
	strscpy(overlay->path, path_buf, sizeof(overlay->path));
	mutex_unlock(&overlay->lock);
	kfree(path_buf);
	return count;
}

static ssize_t cfs_overlay_item_status_show(struct config_item *item, char *page)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);
	bool applied;

	mutex_lock(&overlay->lock);
	applied = (overlay->ov_id >= 0);
	mutex_unlock(&overlay->lock);

	return sysfs_emit(page, "%s\n", applied ? "applied" : "unapplied");
}

CONFIGFS_ATTR(cfs_overlay_item_, path);
CONFIGFS_ATTR_RO(cfs_overlay_item_, status);

static struct configfs_attribute *cfs_overlay_attrs[] = {
	&cfs_overlay_item_attr_path,
	&cfs_overlay_item_attr_status,
	NULL,
};

static ssize_t cfs_overlay_item_dtbo_read(struct config_item *item,
					  void *buf, size_t max_count)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);
	const void *src = NULL;
	size_t sz = 0;

	mutex_lock(&overlay->lock);
	if (overlay->dtbo) {
		src = overlay->dtbo;
		sz = overlay->dtbo_size;
	} else if (overlay->fw) {
		src = overlay->fw->data;
		sz = overlay->fw->size;
	}

	if (!src || sz == 0) {
		mutex_unlock(&overlay->lock);
		return 0;
	}

	if (!buf) {
		mutex_unlock(&overlay->lock);
		return sz;
	}

	if (max_count < sz) {
		mutex_unlock(&overlay->lock);
		return -EINVAL;
	}

	memcpy(buf, src, sz);
	mutex_unlock(&overlay->lock);
	return sz;
}

static ssize_t cfs_overlay_item_dtbo_write(struct config_item *item,
					   const void *buf, size_t count)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);
	void *copy;
	int err;

	if (count == 0 || count > SZ_1M)
		return -EINVAL;

	copy = kmemdup(buf, count, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	mutex_lock(&overlay->lock);
	if (overlay->ov_id >= 0 || overlay->dtbo || overlay->fw) {
		mutex_unlock(&overlay->lock);
		kfree(copy);
		return -EEXIST;
	}

	err = create_overlay_locked(overlay, copy, count);
	if (err) {
		mutex_unlock(&overlay->lock);
		kfree(copy);
		return err;
	}

	overlay->dtbo = copy;
	overlay->dtbo_size = count;
	mutex_unlock(&overlay->lock);
	return count;
}

CONFIGFS_BIN_ATTR(cfs_overlay_item_, dtbo, NULL, SZ_1M);

static struct configfs_bin_attribute *cfs_overlay_bin_attrs[] = {
	&cfs_overlay_item_attr_dtbo,
	NULL,
};

static void cfs_overlay_release(struct config_item *item)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);

	mutex_lock(&overlay->lock);
	destroy_overlay_locked(overlay);
	mutex_unlock(&overlay->lock);
	kfree(overlay);
}

static struct configfs_item_operations cfs_overlay_item_ops = {
	.release = cfs_overlay_release,
};

static const struct config_item_type cfs_overlay_type = {
	.ct_item_ops	= &cfs_overlay_item_ops,
	.ct_attrs	= cfs_overlay_attrs,
	.ct_bin_attrs	= cfs_overlay_bin_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_item *cfs_overlay_group_make_item(struct config_group *group,
						       const char *name)
{
	struct cfs_overlay_item *overlay;

	overlay = kzalloc(sizeof(*overlay), GFP_KERNEL);
	if (!overlay)
		return ERR_PTR(-ENOMEM);

	mutex_init(&overlay->lock);
	overlay->ov_id = -1;
	config_item_init_type_name(&overlay->item, name, &cfs_overlay_type);
	return &overlay->item;
}

static void cfs_overlay_group_drop_item(struct config_group *group,
					struct config_item *item)
{
	struct cfs_overlay_item *overlay = to_cfs_overlay_item(item);

	mutex_lock(&overlay->lock);
	destroy_overlay_locked(overlay);
	mutex_unlock(&overlay->lock);
	config_item_put(item);
}

static struct configfs_group_operations overlays_ops = {
	.make_item = cfs_overlay_group_make_item,
	.drop_item = cfs_overlay_group_drop_item,
};

static const struct config_item_type overlays_type = {
	.ct_group_ops	= &overlays_ops,
	.ct_owner	= THIS_MODULE,
};

static const struct config_item_type of_cfs_type = {
	.ct_owner	= THIS_MODULE,
};

static struct configfs_subsystem of_cfs_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf	= "device-tree",
			.ci_type	= &of_cfs_type,
		},
	},
};

static int __init of_cfs_init(void)
{
	int ret;

	of_cfs_fw_pdev = platform_device_register_simple("of-configfs-fw", -1, NULL, 0);
	if (IS_ERR(of_cfs_fw_pdev))
		return PTR_ERR(of_cfs_fw_pdev);

	config_group_init(&of_cfs_subsys.su_group);
	mutex_init(&of_cfs_subsys.su_mutex);

	ret = configfs_register_subsystem(&of_cfs_subsys);
	if (ret) {
		platform_device_unregister(of_cfs_fw_pdev);
		return ret;
	}

	overlays_group = configfs_register_default_group(&of_cfs_subsys.su_group,
							 "overlays",
							 &overlays_type);
	if (IS_ERR(overlays_group)) {
		ret = PTR_ERR(overlays_group);
		configfs_unregister_subsystem(&of_cfs_subsys);
		platform_device_unregister(of_cfs_fw_pdev);
		return ret;
	}

	pr_info("of_configfs: registered /sys/kernel/config/device-tree/overlays\n");
	return 0;
}

static void __exit of_cfs_exit(void)
{
	configfs_unregister_default_group(overlays_group);
	configfs_unregister_subsystem(&of_cfs_subsys);
	platform_device_unregister(of_cfs_fw_pdev);
}

module_init(of_cfs_init);
module_exit(of_cfs_exit);

MODULE_DESCRIPTION("Device Tree Overlay ConfigFS interface");
MODULE_LICENSE("GPL");
