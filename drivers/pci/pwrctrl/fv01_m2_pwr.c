// SPDX-License-Identifier: GPL-2.0-only
/*
 * FV01 / FV04 onboard M.2 PCIe/NVMe slot power, reset & TC9563 PERST# sequencer.
 *
 * - FV01 (QCS6490 / A6490PF1): executes the Factory 5.4.259 x62_power_probe
 *   sequence on PM7250B GPIO1 (power_enable), GPIO2 (wwan_disable / W_DISABLE1#),
 *   and GPIO3 (reset), then deasserts TC9563 EP PERST# mask 0x0c and rescans
 *   PCIe1 Downstream Port 1 (0001:02:01.0 -> Bus 03).
 * - FV04 (QCS8550 / A8550PF1): carrier rails are sequenced via chained GPIO
 *   fixed-regulators on &pcie1; once TC9563 (0x77 on &i2c_hub_2) exits PERST#,
 *   applies optional TC9563 SerDes/switch register updates, deasserts EP PERST#
 *   mask 0x06 (DSP2 Renesas USB3 + DSP1 M.2 Key-M NVMe), and rescans PCIe1
 *   Downstream Ports 1..3 (Buses 03..05).
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
#include <linux/property.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define TC9563_VERSION_REG		0x800000
#define TC9563_GPIO_CONFIG_REG		0x801208
#define TC9563_RESET_GPIO_REG		0x801210
#define TC9563_DEFAULT_EP_RESET_BITS	0x0000000cU

struct fv01_m2_variant {
	bool gpios_optional;
	u32 default_ep_reset_mask;
	u32 m2_ep_reset_bit;
	bool pulse_ep_reset;
	bool rescan_all_dsps;
};

struct fv01_m2_pwr {
	struct device *dev;
	const struct fv01_m2_variant *variant;
	struct mutex lock;
	struct gpio_desc *pwr_en_gpio;
	struct gpio_desc *wwan_dis_gpio;
	struct gpio_desc *reset_gpio;
	struct i2c_adapter *i2c_adap;
	u16 i2c_addr;
	u32 ep_reset_mask;
	u32 *reg_updates;
	int num_reg_updates;
	u32 *switch_reg_updates;
	int num_switch_reg_updates;
	bool tc9563_programmed;
	bool usp_initialized;
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

static void fv01_tc9563_apply_pairs(struct fv01_m2_pwr *m2,
				    const u32 *pairs, int count,
				    const char *label)
{
	int i, ret;

	for (i = 0; i + 1 < count; i += 2) {
		ret = fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
					    pairs[i], pairs[i + 1]);
		if (ret) {
			dev_warn(m2->dev,
				 "TC9563 %s write [0x%06x]=0x%08x failed: %d\n",
				 label, pairs[i], pairs[i + 1], ret);
			return;
		}
	}
}

static int fv01_m2_power_on_locked(struct fv01_m2_pwr *m2, bool cold_seq)
{
	int ret;

	if (!m2->pwr_en_gpio && !m2->wwan_dis_gpio && !m2->reset_gpio) {
		m2->pwr_en_val = 1;
		return 0;
	}

	if (cold_seq && m2->reset_gpio) {
		ret = gpiod_direction_output_raw(m2->reset_gpio, 1);
		if (ret)
			return ret;
		m2->reset_val = 1;
		msleep(10);
	}

	if (m2->pwr_en_gpio) {
		ret = gpiod_direction_output_raw(m2->pwr_en_gpio, 1);
		if (ret)
			return ret;
		m2->pwr_en_val = 1;
	}

	if (cold_seq && (m2->pwr_en_gpio || m2->reset_gpio))
		msleep(20);

	if (m2->reset_gpio) {
		ret = gpiod_direction_output_raw(m2->reset_gpio, 0);
		if (ret)
			return ret;
		m2->reset_val = 0;
	}

	/*
	 * On FV01 (A6490PF1), PM7250B GPIO2 (wwan-disable) must be driven LOW (0)
	 * for M.2 NVMe SSDs to enable and train PCIe Gen3 x2 (driving HIGH holds
	 * the SSD disabled with DLActive=0).
	 */
	if (m2->wwan_dis_gpio) {
		ret = gpiod_direction_output_raw(m2->wwan_dis_gpio, 0);
		if (ret)
			return ret;
		m2->wwan_dis_val = 0;
	}

	return 0;
}

static void fv01_m2_rescan_dsp(struct pci_dev *dsp)
{
	if (!dsp)
		return;
	if (dsp->subordinate)
		pci_rescan_bus(dsp->subordinate);
}

static void fv01_m2_check_work_fn(struct work_struct *work)
{
	struct fv01_m2_pwr *m2 = container_of(to_delayed_work(work),
					      struct fv01_m2_pwr, check_work);
	struct pci_dev *dsp1, *dsp2 = NULL, *dsp3 = NULL, *ep;
	u32 mask = m2->ep_reset_mask;
	u16 lnksta = 0;
	int i;

	ep = pci_get_domain_bus_and_slot(1, 3, 0);
	if (ep && (!m2->variant->rescan_all_dsps || m2->tc9563_programmed)) {
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
	if (ep)
		pci_dev_put(ep);

	dsp1 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(1, 0));
	if (!dsp1)
		return;

	if (m2->variant->rescan_all_dsps) {
		dsp2 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(2, 0));
		dsp3 = pci_get_domain_bus_and_slot(1, 2, PCI_DEVFN(3, 0));
	}

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
				 "TC9563 0x%02x state: ver=0x%x gpio_cfg=0x%08x reset_gpio=0x%08x mask=0x%x\n",
				 m2->i2c_addr, ver, gpio_cfg, rst_gpio, mask);

			if (m2->variant->pulse_ep_reset) {
				u32 pulse_bits = m2->variant->m2_ep_reset_bit ?
						 (m2->variant->m2_ep_reset_bit & mask) : mask;
				u32 new_cfg = gpio_cfg & ~mask;
				u32 assert_rst, deassert_rst;

				/*
				 * Always keep sibling DSP2 (0001:04:00.0 Renesas USB3, BIT(1))
				 * deasserted HIGH so its link training / early xHCI handoff is
				 * never interrupted, and issue a clean active-low PERST# pulse
				 * solely on DSP1 M.2 Key-M (BIT(2)).
				 */
				deassert_rst = rst_gpio | mask;
				assert_rst = deassert_rst & ~pulse_bits;

				fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
						      TC9563_RESET_GPIO_REG, assert_rst);
				fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
						      TC9563_GPIO_CONFIG_REG, new_cfg);
				msleep(25);
				fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
						      TC9563_RESET_GPIO_REG, deassert_rst);
				fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
						     TC9563_GPIO_CONFIG_REG, &gpio_cfg);
				fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
						     TC9563_RESET_GPIO_REG, &rst_gpio);
				dev_info(m2->dev,
					 "TC9563 0x%02x pulsed EP PERST# (pulse=0x%x): gpio_cfg=0x%08x reset_gpio=0x%08x\n",
					 m2->i2c_addr, pulse_bits, gpio_cfg, rst_gpio);

				for (i = 0; i < 6; i++) {
					msleep(100);
					pcie_capability_read_word(dsp1, PCI_EXP_LNKSTA, &lnksta);
					if (lnksta & PCI_EXP_LNKSTA_DLLLA)
						break;
				}

				if (!(lnksta & PCI_EXP_LNKSTA_DLLLA) && !m2->tc9563_programmed) {
					fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
							      TC9563_RESET_GPIO_REG, assert_rst);
					if (m2->num_reg_updates > 0)
						fv01_tc9563_apply_pairs(m2, m2->reg_updates,
									m2->num_reg_updates,
									"reg-update");
					if (m2->num_switch_reg_updates > 0)
						fv01_tc9563_apply_pairs(m2, m2->switch_reg_updates,
									m2->num_switch_reg_updates,
									"switch-reg-update");
					m2->tc9563_programmed = true;
					msleep(25);
					fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
							      TC9563_RESET_GPIO_REG, deassert_rst);
					msleep(200);
				}
			} else {
				if (!m2->tc9563_programmed) {
					if (m2->num_reg_updates > 0)
						fv01_tc9563_apply_pairs(m2, m2->reg_updates,
									m2->num_reg_updates,
									"reg-update");
					if (m2->num_switch_reg_updates > 0)
						fv01_tc9563_apply_pairs(m2, m2->switch_reg_updates,
									m2->num_switch_reg_updates,
									"switch-reg-update");
					m2->tc9563_programmed = true;
				}

				if ((gpio_cfg & mask) != 0 ||
				    (rst_gpio & mask) != mask) {
					u32 new_rst = rst_gpio | mask;
					u32 new_cfg = gpio_cfg & ~mask;

					/*
					 * Preload output latch HIGH before enabling output
					 * drivers so sibling downstream ports never see
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
			}
		} else {
			dev_warn(m2->dev,
				 "TC9563 0x%02x I2C read skipped/failed (%d/%d/%d)\n",
				 m2->i2c_addr, r1, r2, r3);
		}
	}

	/* Poll up to 400 ms for DSP1/DSP2 Data Link Layer Link Active */
	for (i = 0; i < 4; i++) {
		u16 lnk2 = PCI_EXP_LNKSTA_DLLLA;

		pcie_capability_read_word(dsp1, PCI_EXP_LNKSTA, &lnksta);
		if (dsp2)
			pcie_capability_read_word(dsp2, PCI_EXP_LNKSTA, &lnk2);
		if ((lnksta & PCI_EXP_LNKSTA_DLLLA) && (lnk2 & PCI_EXP_LNKSTA_DLLLA))
			break;
		msleep(100);
	}

	pci_lock_rescan_remove();
	fv01_m2_rescan_dsp(dsp1);
	fv01_m2_rescan_dsp(dsp2);
	fv01_m2_rescan_dsp(dsp3);
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

	if (dsp3)
		pci_dev_put(dsp3);
	if (dsp2)
		pci_dev_put(dsp2);
	pci_dev_put(dsp1);
}

static void fv04_tc9563_early_usp_init(struct fv01_m2_pwr *m2)
{
	u32 mask = m2->ep_reset_mask;
	u32 pulse_bits = m2->variant->m2_ep_reset_bit ?
			 (m2->variant->m2_ep_reset_bit & mask) : mask;
	u32 ver = 0, gpio_cfg = 0, rst_gpio = 0;
	int r1, r2, r3;

	if (m2->usp_initialized || !m2->i2c_adap)
		return;

	r1 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
				  TC9563_VERSION_REG, &ver);
	r2 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
				  TC9563_GPIO_CONFIG_REG, &gpio_cfg);
	r3 = fv01_tc9563_i2c_read(m2->i2c_adap, m2->i2c_addr,
				  TC9563_RESET_GPIO_REG, &rst_gpio);
	if (r1 || r2 || r3) {
		dev_warn(m2->dev,
			 "TC9563 0x%02x early USP I2C read failed (%d/%d/%d)\n",
			 m2->i2c_addr, r1, r2, r3);
		return;
	}

	if ((gpio_cfg & mask) != 0 || (rst_gpio & mask) != mask) {
		/*
		 * Cold boot or reboot from factory 5.15 (where pcie_shutdown
		 * leaves reset_gpio=0x0): program SerDes/switch updates and
		 * release BOTH BIT(1) (Renesas uPD720201 USB3) and BIT(2)
		 * (M.2 Key-M NVMe) BEFORE pci_scan_child_bus_extend() scans
		 * Bus 02, waiting 450 ms so uPD720201 exits POR without ever
		 * triggering pcie_failed_link_retrain() 2.5 GT/s downgrade and
		 * Root Port / Switch MMIO windows are sized for all endpoints.
		 */
		u32 new_cfg = gpio_cfg & ~mask;
		u32 assert_rst = rst_gpio & ~mask;
		u32 deassert_rst = rst_gpio | mask;

		fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
				      TC9563_RESET_GPIO_REG, assert_rst);
		fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
				      TC9563_GPIO_CONFIG_REG, new_cfg);
		if (!m2->tc9563_programmed) {
			if (m2->num_reg_updates > 0)
				fv01_tc9563_apply_pairs(m2, m2->reg_updates,
							m2->num_reg_updates,
							"reg-update");
			if (m2->num_switch_reg_updates > 0)
				fv01_tc9563_apply_pairs(m2, m2->switch_reg_updates,
							m2->num_switch_reg_updates,
							"switch-reg-update");
			m2->tc9563_programmed = true;
		}
		msleep(25);
		fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
				      TC9563_RESET_GPIO_REG, deassert_rst);
		msleep(450);
		dev_info(m2->dev,
			 "TC9563 0x%02x early USP cold-init (ver=0x%x): released mask=0x%x -> 0x%08x\n",
			 m2->i2c_addr, ver, mask, deassert_rst);
	} else {
		/*
		 * Warm reboot with BIT(1) Renesas USB3 already deasserted HIGH:
		 * keep BIT(1) HIGH and pulse BIT(2) M.2 Key-M PERST# before Bus 02 scan.
		 */
		u32 deassert_rst = rst_gpio | mask;
		u32 assert_rst = deassert_rst & ~pulse_bits;

		fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
				      TC9563_RESET_GPIO_REG, assert_rst);
		msleep(25);
		fv01_tc9563_i2c_write(m2->i2c_adap, m2->i2c_addr,
				      TC9563_RESET_GPIO_REG, deassert_rst);
		msleep(150);
		dev_info(m2->dev,
			 "TC9563 0x%02x early USP warm-pulse (ver=0x%x): pulsed 0x%x -> 0x%08x\n",
			 m2->i2c_addr, ver, pulse_bits, deassert_rst);
	}

	m2->usp_initialized = true;
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
	    pdev->bus->number == 1 &&
	    pdev->devfn == 0 &&
	    m2->variant->pulse_ep_reset) {
		fv04_tc9563_early_usp_init(m2);
		return NOTIFY_DONE;
	}

	if (action == BUS_NOTIFY_ADD_DEVICE &&
	    pci_domain_nr(pdev->bus) == 1 &&
	    pdev->bus->number == 2 &&
	    pdev->devfn == (m2->variant->pulse_ep_reset ? PCI_DEVFN(3, 0) : PCI_DEVFN(1, 0))) {
		schedule_delayed_work(&m2->check_work, msecs_to_jiffies(250));
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

	if (!m2->pwr_en_gpio)
		return -EOPNOTSUPP;

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

	if (!m2->wwan_dis_gpio)
		return -EOPNOTSUPP;

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

	if (!m2->reset_gpio)
		return -EOPNOTSUPP;

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

static int fv01_m2_read_u32_pairs(struct device *dev, const char *propname,
				  u32 **out_buf, int *out_count)
{
	int n = of_property_count_u32_elems(dev->of_node, propname);
	u32 *buf;
	int ret;

	if (n <= 0)
		return 0;
	if (n % 2 != 0)
		return -EINVAL;

	buf = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, propname, buf, n);
	if (ret)
		return ret;

	*out_buf = buf;
	*out_count = n;
	return 0;
}

static const struct fv01_m2_variant fv01_variant = {
	.gpios_optional = false,
	.default_ep_reset_mask = TC9563_DEFAULT_EP_RESET_BITS,
	.rescan_all_dsps = false,
};

static const struct fv01_m2_variant fv04_variant = {
	.gpios_optional = true,
	.default_ep_reset_mask = 0x00000006U,
	.m2_ep_reset_bit = 0x00000004U,
	.pulse_ep_reset = true,
	/*
	 * FV04's onboard TC9563 EEPROM already brings up Buses 03..05 at PCIe
	 * Gen3 x2 during root-port enumeration; skip soft-resetting TC9563 when
	 * 0001:03:00.0 is already online, and run I2C recovery only if absent.
	 */
	.rescan_all_dsps = false,
};

static int fv01_m2_pwr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct fv01_m2_variant *variant;
	struct device_node *i2c_np;
	struct pci_dev *dsp1;
	struct fv01_m2_pwr *m2;
	u32 addr = 0x77;
	int ret;

	if (!of_machine_is_compatible("faiot,fv01") &&
	    !of_machine_is_compatible("faiot,fv04") &&
	    !of_machine_is_compatible("qcom,yupikp-iot") &&
	    !of_machine_is_compatible("qcom,kalamap-hdk"))
		return -ENODEV;

	variant = device_get_match_data(dev);
	if (!variant)
		variant = &fv01_variant;

	m2 = devm_kzalloc(dev, sizeof(*m2), GFP_KERNEL);
	if (!m2)
		return -ENOMEM;

	m2->dev = dev;
	m2->variant = variant;
	m2->ep_reset_mask = variant->default_ep_reset_mask;
	of_property_read_u32(dev->of_node, "ep-reset-gpio-mask", &m2->ep_reset_mask);

	ret = fv01_m2_read_u32_pairs(dev, "tc9563-reg-update",
				     &m2->reg_updates, &m2->num_reg_updates);
	if (ret)
		return dev_err_probe(dev, ret, "invalid tc9563-reg-update\n");

	ret = fv01_m2_read_u32_pairs(dev, "tc9563-switch-reg-update",
				     &m2->switch_reg_updates,
				     &m2->num_switch_reg_updates);
	if (ret)
		return dev_err_probe(dev, ret, "invalid tc9563-switch-reg-update\n");

	mutex_init(&m2->lock);
	INIT_DELAYED_WORK(&m2->check_work, fv01_m2_check_work_fn);
	platform_set_drvdata(pdev, m2);

	if (variant->gpios_optional) {
		m2->pwr_en_gpio = devm_gpiod_get_optional(dev, "power-enable", GPIOD_ASIS);
		m2->wwan_dis_gpio = devm_gpiod_get_optional(dev, "wwan-disable", GPIOD_ASIS);
		m2->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	} else {
		m2->pwr_en_gpio = devm_gpiod_get(dev, "power-enable", GPIOD_ASIS);
		m2->wwan_dis_gpio = devm_gpiod_get(dev, "wwan-disable", GPIOD_ASIS);
		m2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	}

	if (IS_ERR(m2->pwr_en_gpio))
		return dev_err_probe(dev, PTR_ERR(m2->pwr_en_gpio),
				     "failed to get power-enable GPIO\n");
	if (IS_ERR(m2->wwan_dis_gpio))
		return dev_err_probe(dev, PTR_ERR(m2->wwan_dis_gpio),
				     "failed to get wwan-disable GPIO\n");
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
		 "M.2/TC9563 power controller ready (ep_reset_mask=0x%x, reg_updates=%d/%d)\n",
		 m2->ep_reset_mask, m2->num_reg_updates / 2,
		 m2->num_switch_reg_updates / 2);

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
	m2->tc9563_programmed = false;
	m2->usp_initialized = false;
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
	{ .compatible = "faiot,fv01-m2-power-ctrl", .data = &fv01_variant },
	{ .compatible = "faiot,fv04-m2-power-ctrl", .data = &fv04_variant },
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
