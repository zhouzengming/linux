// SPDX-License-Identifier: GPL-2.0-only
/*
 * FV01 onboard M.2 PCIe/NVMe slot power, reset & TC9563 PERST# sequencer.
 *
 * Follows the exact Factory 5.4.259 x62_power_probe sequence on PM7250B
 * GPIO1 (power_enable), GPIO2 (wwan_disable / W_DISABLE1#), and GPIO3 (reset),
 * and ensures glitch-free TC9563 downstream endpoint PERST# release + PCIe
 * Bus 0001:03 rescan once PCIe1 Downstream Port 1 (0001:02:01.0) enumerates.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define TC9563_VERSION_REG		0x800000
#define TC9563_GPIO_CONFIG_REG		0x801208
#define TC9563_RESET_GPIO_REG		0x801210
#define TC9563_EP_RESET_BITS		0x0000000cU

struct fv01_m2_pwr {
	struct device *dev;
	struct mutex lock;
	struct gpio_desc *pwr_en_gpio;
	struct gpio_desc *wwan_dis_gpio;
	struct gpio_desc *reset_gpio;
	struct i2c_adapter *i2c_adap;
	u16 i2c_addr;
	int pwr_en_val;
	int wwan_dis_val;
	int reset_val;
	struct notifier_block pci_nb;
	struct delayed_work check_work;
};

static int fv01_tc9563_i2c_read(struct i2c_adapter *adap, u16 addr,
				u32 reg, u32 *val)
{
	u8 reg_buf[3];
	u8 data_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	put_unaligned_be24(reg, reg_buf);

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg_buf);
	msgs[0].buf = reg_buf;

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(data_buf);
	msgs[1].buf = data_buf;

	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = get_unaligned_le32(data_buf);
	return 0;
}

static int fv01_tc9563_i2c_write(struct i2c_adapter *adap, u16 addr,
				 u32 reg, u32 val)
{
	u8 buf[7];
	struct i2c_msg msg;
	int ret;

	put_unaligned_be24(reg, &buf[0]);
	put_unaligned_le32(val, &buf[3]);

	msg.addr = addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	ret = i2c_transfer(adap, &msg, 1);
	return (ret == 1) ? 0 : (ret < 0 ? ret : -EIO);
}

static int fv01_m2_power_on_locked(struct fv01_m2_pwr *m2, bool cold_seq)
{
	int ret;

	if (cold_seq) {
		ret = gpiod_direction_output_raw(m2->reset_gpio, 1);
		if (ret)
			return ret;
		m2->reset_val = 1;
		msleep(10);
	}

	ret = gpiod_direction_output_raw(m2->pwr_en_gpio, 1);
	if (ret)
		return ret;
	m2->pwr_en_val = 1;

	if (cold_seq)
		msleep(20);

	ret = gpiod_direction_output_raw(m2->reset_gpio, 0);
	if (ret)
		return ret;
	m2->reset_val = 0;

	/*
	 * On FV01 (A6490PF1), PM7250B GPIO2 (wwan-disable) must be driven LOW (0)
	 * for M.2 NVMe SSDs to enable and train PCIe Gen3 x2 (driving HIGH holds
	 * the SSD disabled with DLActive=0).
	 */
	ret = gpiod_direction_output_raw(m2->wwan_dis_gpio, 0);
	if (ret)
		return ret;
	m2->wwan_dis_val = 0;

	return 0;
}

static void fv01_m2_check_work_fn(struct work_struct *work)
{
	struct fv01_m2_pwr *m2 = container_of(to_delayed_work(work),
					      struct fv01_m2_pwr, check_work);
	struct pci_dev *dsp1, *ep;
	u16 lnksta = 0;
	int i;

	ep = pci_get_domain_bus_and_slot(1, 3, 0);
	if (ep) {
		dsp1 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(1, 0));
		if (dsp1) {
			pcie_capability_read_word(dsp1, PCI_EXP_LNKSTA, &lnksta);
			pci_dev_put(dsp1);
		}
		dev_info(m2->dev,
			 "M.2 endpoint 0001:03:00.0 online (%04x:%04x, class=0x%06x, lnksta=0x%04x speed=%u width=x%u)\n",
			 ep->vendor, ep->device, ep->class, lnksta,
			 lnksta & PCI_EXP_LNKSTA_CLS,
			 (lnksta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT);
		pci_dev_put(ep);
		return;
	}

	dsp1 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(1, 0));
	if (!dsp1)
		return;

	if (m2->i2c_adap) {
		u32 ver = 0, gpio_cfg = 0, rst_gpio = 0;
		int r1, r2, r3;

		r1 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
					  TC9563_VERSION_REG, &ver);
		r2 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
					  TC9563_GPIO_CONFIG_REG, &gpio_cfg);
		r3 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
					  TC9563_RESET_GPIO_REG, &rst_gpio);
		if (!r1 && !r2 && !r3) {
			dev_info(m2->dev,
				 "TC9563 0x%02x state: ver=0x%x gpio_cfg=0x%08x reset_gpio=0x%08x\n",
				 m2->i2c_addr, ver, gpio_cfg, rst_gpio);

			if ((gpio_cfg & TC9563_EP_RESET_BITS) != 0 ||
			    (rst_gpio & TC9563_EP_RESET_BITS) != TC9563_EP_RESET_BITS) {
				u32 new_rst = rst_gpio | TC9563_EP_RESET_BITS;
				u32 new_cfg = gpio_cfg & ~TC9563_EP_RESET_BITS;

				/*
				 * Preload output latch HIGH before enabling output
				 * drivers so sibling DSP2 (0001:04:00.0) never sees
				 * a negative reset glitch.
				 */
				fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
						      TC9563_RESET_GPIO_REG, new_rst);
				fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
						      TC9563_GPIO_CONFIG_REG, new_cfg);
				fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
						     TC9563_GPIO_CONFIG_REG, &gpio_cfg);
				fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
						     TC9563_RESET_GPIO_REG, &rst_gpio);
				dev_info(m2->dev,
					 "TC9563 0x%02x deasserted EP PERST#: gpio_cfg=0x%08x reset_gpio=0x%08x\n",
					 m2->i2c_addr, gpio_cfg, rst_gpio);
				msleep(120);
			}
		} else {
			dev_warn(m2->dev,
				 "TC9563 0x%02x I2C read skipped/failed (%d/%d/%d)\n",
				 m2->i2c_addr, r1, r2, r3);
		}
	}

	/* Poll up to 400 ms for DSP1 Data Link Layer Link Active if card present */
	for (i = 0; i < 4; i++) {
		pcie_capability_read_word(dsp1, PCI_EXP_LNKSTA, &lnksta);
		if (lnksta & PCI_EXP_LNKSTA_DLLLA)
			break;
		msleep(100);
	}

	pci_lock_rescan_remove();
	if (dsp1->subordinate)
		pci_rescan_bus(dsp1->subordinate);
	pci_unlock_rescan_remove();

	pcie_capability_read_word(dsp1, PCI_EXP_LNKSTA, &lnksta);
	ep = pci_get_domain_bus_and_slot(1, 3, 0);
	if (ep) {
		dev_info(m2->dev,
			 "M.2 endpoint 0001:03:00.0 enumerated (%04x:%04x, class=0x%06x, lnksta=0x%04x speed=%u width=x%u)\n",
			 ep->vendor, ep->device, ep->class, lnksta,
			 lnksta & PCI_EXP_LNKSTA_CLS,
			 (lnksta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT);
		pci_dev_put(ep);
	} else {
		dev_info(m2->dev,
			 "M.2 slot Bus 0001:03 scanned (empty / DLActive=%d, lnksta=0x%04x)\n",
			 !!(lnksta & PCI_EXP_LNKSTA_DLLLA), lnksta);
	}

	pci_dev_put(dsp1);
}

static int fv01_m2_pci_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct fv01_m2_pwr *m2 = container_of(nb, struct fv01_m2_pwr, pci_nb);
	struct device *dev = data;
	struct pci_dev *pdev;

	if (!dev_is_pci(dev))
		return NOTIFY_DONE;

	pdev = to_pci_dev(dev);

	if (action == BUS_NOTIFY_ADD_DEVICE &&
	    pci_domain_nr(pdev->bus) == 1 &&
	    pdev->bus->number == 2 &&
	    pdev->devfn == PCI_DEVFN(1, 0)) {
		schedule_delayed_work(&m2->check_work, msecs_to_jiffies(200));
	}

	return NOTIFY_DONE;
}

static ssize_t power_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m2->pwr_en_val);
}

static ssize_t power_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&m2->lock);
	ret = gpiod_direction_output_raw(m2->pwr_en_gpio, val ? 1 : 0);
	if (!ret)
		m2->pwr_en_val = val ? 1 : 0;
	mutex_unlock(&m2->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(power_enable);

static ssize_t wwan_disable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m2->wwan_dis_val);
}

static ssize_t wwan_disable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&m2->lock);
	ret = gpiod_direction_output_raw(m2->wwan_dis_gpio, val ? 1 : 0);
	if (!ret)
		m2->wwan_dis_val = val ? 1 : 0;
	mutex_unlock(&m2->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(wwan_disable);

static ssize_t reset_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m2->reset_val);
}

static ssize_t reset_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&m2->lock);
	ret = gpiod_direction_output_raw(m2->reset_gpio, val ? 1 : 0);
	if (!ret)
		m2->reset_val = val ? 1 : 0;
	mutex_unlock(&m2->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(reset);

static ssize_t rescan_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (val)
		mod_delayed_work(system_wq, &m2->check_work, 0);

	return count;
}
static DEVICE_ATTR_WO(rescan);

static struct attribute *fv01_m2_pwr_attrs[] = {
	&dev_attr_power_enable.attr,
	&dev_attr_wwan_disable.attr,
	&dev_attr_reset.attr,
	&dev_attr_rescan.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fv01_m2_pwr);

static void fv01_m2_pwr_cleanup(void *data)
{
	struct fv01_m2_pwr *m2 = data;

	bus_unregister_notifier(&pci_bus_type, &m2->pci_nb);
	cancel_delayed_work_sync(&m2->check_work);
	if (m2->i2c_adap)
		i2c_put_adapter(m2->i2c_adap);
}

static int fv01_m2_pwr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *i2c_np;
	struct pci_dev *dsp1;
	struct fv01_m2_pwr *m2;
	u32 addr = 0x77;
	int ret;

	if (!of_machine_is_compatible("faiot,fv01") &&
	    !of_machine_is_compatible("qcom,yupikp-iot"))
		return -ENODEV;

	m2 = devm_kzalloc(dev, sizeof(*m2), GFP_KERNEL);
	if (!m2)
		return -ENOMEM;

	m2->dev = dev;
	mutex_init(&m2->lock);
	INIT_DELAYED_WORK(&m2->check_work, fv01_m2_check_work_fn);
	platform_set_drvdata(pdev, m2);

	m2->pwr_en_gpio = devm_gpiod_get(dev, "power-enable", GPIOD_ASIS);
	if (IS_ERR(m2->pwr_en_gpio))
		return dev_err_probe(dev, PTR_ERR(m2->pwr_en_gpio),
				     "failed to get power-enable GPIO\n");

	m2->wwan_dis_gpio = devm_gpiod_get(dev, "wwan-disable", GPIOD_ASIS);
	if (IS_ERR(m2->wwan_dis_gpio))
		return dev_err_probe(dev, PTR_ERR(m2->wwan_dis_gpio),
				     "failed to get wwan-disable GPIO\n");

	m2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(m2->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(m2->reset_gpio),
				     "failed to get reset GPIO\n");

	i2c_np = of_parse_phandle(dev->of_node, "tc9563-i2c", 0);
	if (i2c_np) {
		of_property_read_u32_index(dev->of_node, "tc9563-i2c", 1, &addr);
		m2->i2c_addr = (u16)addr;
		m2->i2c_adap = of_get_i2c_adapter_by_node(i2c_np);
		of_node_put(i2c_np);
		if (!m2->i2c_adap)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "waiting for TC9563 I2C adapter\n");
	}

	mutex_lock(&m2->lock);
	ret = fv01_m2_power_on_locked(m2, true);
	mutex_unlock(&m2->lock);
	if (ret) {
		if (m2->i2c_adap)
			i2c_put_adapter(m2->i2c_adap);
		return dev_err_probe(dev, ret,
				     "failed to execute M.2 power sequence\n");
	}

	m2->pci_nb.notifier_call = fv01_m2_pci_notify;
	ret = bus_register_notifier(&pci_bus_type, &m2->pci_nb);
	if (ret) {
		if (m2->i2c_adap)
			i2c_put_adapter(m2->i2c_adap);
		return dev_err_probe(dev, ret,
				     "failed to register PCI bus notifier\n");
	}

	ret = devm_add_action_or_reset(dev, fv01_m2_pwr_cleanup, m2);
	if (ret)
		return ret;

	dev_info(dev,
		 "M.2 slot power enabled & reset released (PM7250B GPIO1=1, GPIO2=0, GPIO3=0)\n");

	dsp1 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(1, 0));
	if (dsp1) {
		pci_dev_put(dsp1);
		schedule_delayed_work(&m2->check_work, msecs_to_jiffies(150));
	}

	return 0;
}

static int fv01_m2_pwr_resume(struct device *dev)
{
	struct fv01_m2_pwr *m2 = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&m2->lock);
	if (m2->pwr_en_val)
		ret = fv01_m2_power_on_locked(m2, false);
	else
		ret = 0;
	mutex_unlock(&m2->lock);

	if (ret)
		dev_warn(dev, "failed to restore M.2 GPIOs on resume: %d\n", ret);
	else if (m2->pwr_en_val)
		schedule_delayed_work(&m2->check_work, msecs_to_jiffies(250));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fv01_m2_pwr_pm_ops, NULL, fv01_m2_pwr_resume);

static const struct of_device_id fv01_m2_pwr_of_match[] = {
	{ .compatible = "faiot,fv01-m2-power-ctrl" },
	{}
};

static struct platform_driver fv01_m2_pwr_driver = {
	.probe = fv01_m2_pwr_probe,
	.driver = {
		.name = "fv01-m2-pwr",
		.of_match_table = fv01_m2_pwr_of_match,
		.dev_groups = fv01_m2_pwr_groups,
		.pm = pm_sleep_ptr(&fv01_m2_pwr_pm_ops),
	},
};
builtin_platform_driver(fv01_m2_pwr_driver);
