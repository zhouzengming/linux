// SPDX-License-Identifier: GPL-2.0-only
/*
 * FV01 (A6490PF1 / QCS6490) Lontium LT6911C / LT6911UXC HDMI-to-CSI2 Bridge Driver
 *
 * Implements the hardware power-on, reset, 8051 I2C gate protocol
 * (0xff=0x80 / 0xee=0x01..0x00), Chip ID verification (0xa000..0xa001 == 0x1605),
 * HDMI timing / MIPI clock readout (0xd2 / 0xa0 / 0xb8 banks), and V4L2 subdev
 * interface wired to QCS6490 CAMSS CSIPHY0 (4 D-PHY lanes, UYVY8_1X16).
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/workqueue.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define LT6911C_CHIP_ID			0x1605
#define LT6911C_DEFAULT_LANES		4
#define LT6911C_WORK_DELAY_MS		300

/* Bank selection & 8051 MCU external I2C gate */
#define LT6911C_REG_BANK		0xff
#define LT6911C_BANK_SYS		0x80
#define LT6911C_REG_I2C_EN		0xee

#define LT6911C_BANK_ID			0xa0
#define LT6911C_REG_ID_H		0x00
#define LT6911C_REG_ID_L		0x01
#define LT6911C_REG_CLK_SEL		0x34
#define LT6911C_CLK_SEL_HDMI_HALF_PCLK	0x21
#define LT6911C_CLK_SEL_MIPI_BYTECLK	0x1a

#define LT6911C_BANK_CLK_VAL		0xb8
#define LT6911C_REG_CLK_B2		0xb1
#define LT6911C_REG_CLK_B1		0xb2
#define LT6911C_REG_CLK_B0		0xb3

#define LT6911C_BANK_TIMING		0xd2
#define LT6911C_REG_HALF_HTOTAL_H	0x89
#define LT6911C_REG_HALF_HTOTAL_L	0x8a
#define LT6911C_REG_HALF_HACT_H		0x8b
#define LT6911C_REG_HALF_HACT_L		0x8c
#define LT6911C_REG_VACT_H		0x96
#define LT6911C_REG_VACT_L		0x97
#define LT6911C_REG_VTOTAL_H		0x9e
#define LT6911C_REG_VTOTAL_L		0x9f

struct fv01_lt6911c_mode {
	u32 width;
	u32 height;
	u32 htotal;
	u32 vtotal;
	u32 code;
	u32 fps;
	u32 lanes;
	u64 pixel_clk;
	s64 link_freq;
};

struct fv01_lt6911c {
	struct i2c_client *client;
	struct mutex lock; /* Serializes banked I2C accesses and mode state */

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_dv_timings timings;
	struct fv01_lt6911c_mode cur_mode;

	struct regulator *vdd3v3;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *det_gpio;
	struct gpio_desc *irq_gpio;

	int det_irq;
	int event_irq;
	struct delayed_work status_work;
	struct mutex power_lock; /* Serializes LT6911C power/reset transitions */

	u16 chip_id;
	bool cable_connected;
	bool powered;
	bool streaming;
};

static const s64 fv01_lt6911c_link_freqs[] = {
	297000000ULL,
	148500000ULL,
	594000000ULL,
};

static const struct v4l2_dv_timings_cap fv01_lt6911c_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	V4L2_INIT_BT_TIMINGS(160, 3840, 120, 2160, 25000000, 600000000,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT,
			     V4L2_DV_BT_CAP_PROGRESSIVE |
			     V4L2_DV_BT_CAP_CUSTOM |
			     V4L2_DV_BT_CAP_REDUCED_BLANKING)
};

static const struct v4l2_event fv01_lt6911c_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static inline struct fv01_lt6911c *to_fv01_lt6911c(struct v4l2_subdev *sd)
{
	return container_of(sd, struct fv01_lt6911c, sd);
}

static int lt6911c_write_reg(struct fv01_lt6911c *lt, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(lt->client, reg, val);

	return ret < 0 ? ret : 0;
}

static int lt6911c_read_reg(struct fv01_lt6911c *lt, u8 reg)
{
	return i2c_smbus_read_byte_data(lt->client, reg);
}

static int lt6911c_read_be16(struct fv01_lt6911c *lt, u8 reg_hi, u8 reg_lo, u16 *val)
{
	int hi, lo;

	hi = lt6911c_read_reg(lt, reg_hi);
	if (hi < 0)
		return hi;

	lo = lt6911c_read_reg(lt, reg_lo);
	if (lo < 0)
		return lo;

	*val = ((u16)hi << 8) | (u16)lo;
	return 0;
}

static int lt6911c_read_be24(struct fv01_lt6911c *lt, u8 b2_reg, u8 b1_reg, u8 b0_reg, u32 *val)
{
	int b2, b1, b0;

	b2 = lt6911c_read_reg(lt, b2_reg);
	if (b2 < 0)
		return b2;

	b1 = lt6911c_read_reg(lt, b1_reg);
	if (b1 < 0)
		return b1;

	b0 = lt6911c_read_reg(lt, b0_reg);
	if (b0 < 0)
		return b0;

	*val = ((u32)b2 << 16) | ((u32)b1 << 8) | (u32)b0;
	return 0;
}

/**
 * lt6911c_set_i2c_gate - Enable or release external I2C access over the 8051 MCU
 * @lt: driver context (caller must hold lt->lock)
 * @enable: true to claim bank register access, false to hand CSI control back to MCU
 */
static int lt6911c_set_i2c_gate(struct fv01_lt6911c *lt, bool enable)
{
	int ret;

	ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_SYS);
	if (ret)
		return ret;

	return lt6911c_write_reg(lt, LT6911C_REG_I2C_EN, enable ? 0x01 : 0x00);
}

static void lt6911c_set_default_mode(struct fv01_lt6911c *lt)
{
	struct v4l2_bt_timings *bt = &lt->timings.bt;

	lt->cur_mode.width = 1920;
	lt->cur_mode.height = 1080;
	lt->cur_mode.htotal = 2200;
	lt->cur_mode.vtotal = 1125;
	lt->cur_mode.fps = 60;
	lt->cur_mode.lanes = LT6911C_DEFAULT_LANES;
	lt->cur_mode.code = MEDIA_BUS_FMT_UYVY8_1X16;
	lt->cur_mode.pixel_clk = 148500000ULL;
	lt->cur_mode.link_freq = 297000000ULL;

	memset(&lt->timings, 0, sizeof(lt->timings));
	lt->timings.type = V4L2_DV_BT_656_1120;
	bt->width = lt->cur_mode.width;
	bt->height = lt->cur_mode.height;
	bt->hsync = lt->cur_mode.htotal - lt->cur_mode.width;
	bt->vsync = lt->cur_mode.vtotal - lt->cur_mode.height;
	bt->pixelclock = lt->cur_mode.pixel_clk;
}

static void lt6911c_hw_power_off(struct fv01_lt6911c *lt)
{
	/*
	 * Hold LT6911C in hardware reset and disable its local rail when idle
	 * so its internal 8051 MCU never drives or glitches the shared QUP
	 * I2C1 bus (984000.i2c, shared with LT9611 HDMI-OUT and S-35390A RTC).
	 */
	if (lt->reset_gpio)
		gpiod_set_value_cansleep(lt->reset_gpio, 0);
	if (lt->enable_gpio)
		gpiod_set_value_cansleep(lt->enable_gpio, 0);
	lt->powered = false;
}

static void lt6911c_hw_power_on(struct fv01_lt6911c *lt)
{
	if (lt->powered)
		return;

	if (lt->enable_gpio) {
		gpiod_set_value_cansleep(lt->enable_gpio, 1);
		msleep(50);
	}

	/* Exact hardware reset sequence required for internal 8051 MCU flash boot */
	if (lt->reset_gpio) {
		gpiod_set_value_cansleep(lt->reset_gpio, 1);
		msleep(50);
		gpiod_set_value_cansleep(lt->reset_gpio, 0);
		msleep(50);
		gpiod_set_value_cansleep(lt->reset_gpio, 1);
		msleep(800);
	}

	lt->powered = true;
}

static int lt6911c_check_chip_id(struct fv01_lt6911c *lt)
{
	u16 id = 0;
	int ret;

	mutex_lock(&lt->lock);
	ret = lt6911c_set_i2c_gate(lt, true);
	if (!ret)
		ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_ID);
	if (!ret)
		ret = lt6911c_read_be16(lt, LT6911C_REG_ID_H, LT6911C_REG_ID_L, &id);
	lt6911c_set_i2c_gate(lt, false);
	mutex_unlock(&lt->lock);

	if (ret)
		return ret;

	if (id != LT6911C_CHIP_ID) {
		dev_err(&lt->client->dev, "unexpected chip id 0x%04x (expected 0x%04x)\n",
			id, LT6911C_CHIP_ID);
		return -ENODEV;
	}

	lt->chip_id = id;
	dev_info(&lt->client->dev, "LT6911UXC HDMI-CSI bridge online (chip ID 0x%04x)\n",
		 lt->chip_id);
	return 0;
}

static int lt6911c_read_hdmi_timings(struct fv01_lt6911c *lt,
				     struct fv01_lt6911c_mode *out)
{
	u16 half_htotal, vtotal, half_hact, vact;
	u32 raw_pclk, raw_bclk;
	u64 pclk, bclk, total_px, fps;
	int ret;

	mutex_lock(&lt->lock);
	ret = lt6911c_set_i2c_gate(lt, true);
	if (ret)
		goto out_unlock;

	msleep(100);

	ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_TIMING);
	if (!ret)
		ret = lt6911c_read_be16(lt, LT6911C_REG_HALF_HTOTAL_H,
					LT6911C_REG_HALF_HTOTAL_L, &half_htotal);
	if (!ret)
		ret = lt6911c_read_be16(lt, LT6911C_REG_VTOTAL_H,
					LT6911C_REG_VTOTAL_L, &vtotal);
	if (!ret)
		ret = lt6911c_read_be16(lt, LT6911C_REG_HALF_HACT_H,
					LT6911C_REG_HALF_HACT_L, &half_hact);
	if (!ret)
		ret = lt6911c_read_be16(lt, LT6911C_REG_VACT_H,
					LT6911C_REG_VACT_L, &vact);
	if (ret)
		goto out_gate_off;

	ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_ID);
	if (!ret)
		ret = lt6911c_write_reg(lt, LT6911C_REG_CLK_SEL,
					LT6911C_CLK_SEL_HDMI_HALF_PCLK);
	if (!ret) {
		msleep(20);
		ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_CLK_VAL);
	}
	if (!ret)
		ret = lt6911c_read_be24(lt, LT6911C_REG_CLK_B2,
					LT6911C_REG_CLK_B1, LT6911C_REG_CLK_B0,
					&raw_pclk);
	if (ret)
		goto out_gate_off;

	ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_ID);
	if (!ret)
		ret = lt6911c_write_reg(lt, LT6911C_REG_CLK_SEL,
					LT6911C_CLK_SEL_MIPI_BYTECLK);
	if (!ret) {
		msleep(20);
		ret = lt6911c_write_reg(lt, LT6911C_REG_BANK, LT6911C_BANK_CLK_VAL);
	}
	if (!ret)
		ret = lt6911c_read_be24(lt, LT6911C_REG_CLK_B2,
					LT6911C_REG_CLK_B1, LT6911C_REG_CLK_B0,
					&raw_bclk);

out_gate_off:
	lt6911c_set_i2c_gate(lt, false);
out_unlock:
	mutex_unlock(&lt->lock);

	if (ret)
		return ret;

	if (!half_hact || !vact || !half_htotal || !vtotal || !raw_bclk)
		return -ENODATA;

	if ((u32)half_hact * 2 < 160 || (u32)half_hact * 2 > 3840 ||
	    vact < 120 || vact > 2160)
		return -ERANGE;

	pclk = (u64)raw_pclk * 2000ULL;
	bclk = (u64)raw_bclk * 4000ULL;
	total_px = (u64)half_htotal * 2ULL * (u64)vtotal;
	fps = total_px ? div64_u64(pclk + div64_u64(total_px, 2), total_px) : 60;
	if (!fps || fps > 120)
		fps = 60;

	*out = lt->cur_mode;
	out->width = (u32)half_hact * 2;
	out->height = vact;
	out->htotal = (u32)half_htotal * 2;
	out->vtotal = vtotal;
	out->fps = (u32)fps;
	out->pixel_clk = pclk ? pclk : (total_px * fps);
	out->link_freq = (s64)bclk * 4LL;
	return 0;
}

static void lt6911c_update_pad_format(const struct fv01_lt6911c_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
}

static void lt6911c_sync_controls_and_state(struct fv01_lt6911c *lt)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	s64 pixel_rate = lt->cur_mode.link_freq;

	if (lt->pixel_rate) {
		v4l2_ctrl_lock(lt->pixel_rate);
		__v4l2_ctrl_modify_range(lt->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);
		v4l2_ctrl_unlock(lt->pixel_rate);
	}

	state = v4l2_subdev_lock_and_get_active_state(&lt->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);
	if (fmt)
		lt6911c_update_pad_format(&lt->cur_mode, fmt);
	v4l2_subdev_unlock_state(state);
}

static void lt6911c_refresh_status(struct fv01_lt6911c *lt, bool notify,
				   bool probe_hw)
{
	struct fv01_lt6911c_mode new_mode;
	int ret;

	if (probe_hw) {
		mutex_lock(&lt->power_lock);
		lt6911c_hw_power_on(lt);
		if (!lt->chip_id) {
			ret = lt6911c_check_chip_id(lt);
			if (ret)
				dev_dbg(&lt->client->dev,
					"LT6911C chip ID probe deferred: %d\n",
					ret);
		}
		ret = lt->chip_id ? lt6911c_read_hdmi_timings(lt, &new_mode) : -ENOLINK;
		if (!lt->streaming)
			lt6911c_hw_power_off(lt);
		mutex_unlock(&lt->power_lock);

		if (!ret) {
			struct v4l2_bt_timings *bt = &lt->timings.bt;

			lt->cable_connected = true;
			lt->cur_mode = new_mode;
			memset(&lt->timings, 0, sizeof(lt->timings));
			lt->timings.type = V4L2_DV_BT_656_1120;
			bt->width = new_mode.width;
			bt->height = new_mode.height;
			bt->hsync = new_mode.htotal - new_mode.width;
			bt->vsync = new_mode.vtotal - new_mode.height;
			bt->pixelclock = new_mode.pixel_clk;

			dev_info(&lt->client->dev,
				 "HDMI-IN locked: %ux%u@%ufps htotal=%u vtotal=%u pclk=%llu link=%lld Hz\n",
				 new_mode.width, new_mode.height, new_mode.fps,
				 new_mode.htotal, new_mode.vtotal,
				 new_mode.pixel_clk, new_mode.link_freq);
		} else {
			lt->cable_connected = false;
			lt6911c_set_default_mode(lt);
			dev_dbg(&lt->client->dev,
				"HDMI-IN source unavailable/standby: %d\n",
				ret);
		}
	} else {
		lt->cable_connected = false;
		lt6911c_set_default_mode(lt);
	}

	if (lt->ctrl_handler.error == 0 && lt->sd.entity.num_pads)
		lt6911c_sync_controls_and_state(lt);

	if (notify)
		v4l2_subdev_notify_event(&lt->sd, &fv01_lt6911c_ev_source_change);
}

static void lt6911c_status_work_fn(struct work_struct *work)
{
	struct fv01_lt6911c *lt = container_of(to_delayed_work(work),
					       struct fv01_lt6911c, status_work);

	lt6911c_refresh_status(lt, true, true);
}

static irqreturn_t lt6911c_irq_handler(int irq, void *dev_id)
{
	struct fv01_lt6911c *lt = dev_id;

	mod_delayed_work(system_dfl_wq, &lt->status_work,
			 msecs_to_jiffies(LT6911C_WORK_DELAY_MS));
	return IRQ_HANDLED;
}

static void lt6911c_regulator_disable(void *data)
{
	regulator_disable(data);
}

static int lt6911c_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int lt6911c_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_UYVY8_1X16)
		return -EINVAL;

	fse->min_width = lt->cur_mode.width;
	fse->max_width = lt->cur_mode.width;
	fse->min_height = lt->cur_mode.height;
	fse->max_height = lt->cur_mode.height;
	return 0;
}

static int lt6911c_set_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);
	struct v4l2_mbus_framefmt *active;

	lt6911c_update_pad_format(&lt->cur_mode, &fmt->format);
	active = v4l2_subdev_state_get_format(sd_state, fmt->pad);
	if (active)
		*active = fmt->format;

	return 0;
}

static int lt6911c_get_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval *fi)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	fi->interval.numerator = 1;
	fi->interval.denominator = lt->cur_mode.fps ? lt->cur_mode.fps : 60;
	return 0;
}

static int lt6911c_s_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_dv_timings *timings)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	if (!v4l2_valid_dv_timings(timings, &fv01_lt6911c_timings_cap, NULL, NULL))
		return -ERANGE;

	lt->timings = *timings;
	return 0;
}

static int lt6911c_g_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_dv_timings *timings)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	*timings = lt->timings;
	return 0;
}

static int lt6911c_query_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				    struct v4l2_dv_timings *timings)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);
	struct fv01_lt6911c_mode mode;
	int ret;

	mutex_lock(&lt->power_lock);
	lt6911c_hw_power_on(lt);
	if (!lt->chip_id)
		lt6911c_check_chip_id(lt);
	ret = lt->chip_id ? lt6911c_read_hdmi_timings(lt, &mode) : -ENOLINK;
	if (!lt->streaming)
		lt6911c_hw_power_off(lt);
	mutex_unlock(&lt->power_lock);

	lt->cable_connected = !ret;
	if (ret)
		return ret;

	memset(timings, 0, sizeof(*timings));
	timings->type = V4L2_DV_BT_656_1120;
	timings->bt.width = mode.width;
	timings->bt.height = mode.height;
	timings->bt.hsync = mode.htotal - mode.width;
	timings->bt.vsync = mode.vtotal - mode.height;
	timings->bt.pixelclock = mode.pixel_clk;
	return 0;
}

static int lt6911c_enum_dv_timings(struct v4l2_subdev *sd,
				   struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &fv01_lt6911c_timings_cap,
					NULL, NULL);
}

static int lt6911c_dv_timings_cap(struct v4l2_subdev *sd,
				  struct v4l2_dv_timings_cap *cap)
{
	*cap = fv01_lt6911c_timings_cap;
	return 0;
}

static int lt6911c_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *cfg)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = lt->cur_mode.lanes;
	cfg->link_freq = lt->cur_mode.link_freq;
	return 0;
}

static int lt6911c_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);
	int ret;

	mutex_lock(&lt->power_lock);
	lt6911c_hw_power_on(lt);

	ret = lt6911c_check_chip_id(lt);
	if (ret || !lt->chip_id) {
		lt6911c_hw_power_off(lt);
		mutex_unlock(&lt->power_lock);
		return -ENOLINK;
	}

	/* Release I2C slave gate so the 8051 MCU actively drives CSI-2 output */
	mutex_lock(&lt->lock);
	ret = lt6911c_set_i2c_gate(lt, false);
	mutex_unlock(&lt->lock);
	if (ret) {
		lt6911c_hw_power_off(lt);
		mutex_unlock(&lt->power_lock);
		return ret;
	}

	lt->streaming = true;
	mutex_unlock(&lt->power_lock);
	return 0;
}

static int lt6911c_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	mutex_lock(&lt->power_lock);
	lt->streaming = false;
	lt6911c_hw_power_off(lt);
	mutex_unlock(&lt->power_lock);

	return 0;
}

static int lt6911c_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	return lt6911c_set_format(sd, sd_state, &fmt);
}

static const struct v4l2_subdev_core_ops lt6911c_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops lt6911c_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops lt6911c_pad_ops = {
	.enum_mbus_code = lt6911c_enum_mbus_code,
	.enum_frame_size = lt6911c_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = lt6911c_set_format,
	.get_frame_interval = lt6911c_get_frame_interval,
	.s_dv_timings = lt6911c_s_dv_timings,
	.g_dv_timings = lt6911c_g_dv_timings,
	.query_dv_timings = lt6911c_query_dv_timings,
	.enum_dv_timings = lt6911c_enum_dv_timings,
	.dv_timings_cap = lt6911c_dv_timings_cap,
	.get_mbus_config = lt6911c_get_mbus_config,
	.enable_streams = lt6911c_enable_streams,
	.disable_streams = lt6911c_disable_streams,
};

static const struct v4l2_subdev_ops lt6911c_subdev_ops = {
	.core = &lt6911c_core_ops,
	.video = &lt6911c_video_ops,
	.pad = &lt6911c_pad_ops,
};

static const struct v4l2_subdev_internal_ops lt6911c_internal_ops = {
	.init_state = lt6911c_init_state,
};

static const struct media_entity_operations lt6911c_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static ssize_t chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	if (!lt->chip_id) {
		mutex_lock(&lt->power_lock);
		lt6911c_hw_power_on(lt);
		lt6911c_check_chip_id(lt);
		if (!lt->streaming)
			lt6911c_hw_power_off(lt);
		mutex_unlock(&lt->power_lock);
	}

	if (!lt->chip_id)
		return sysfs_emit(buf, "0x0000 (standby)\n");

	return sysfs_emit(buf, "0x%04x\n", lt->chip_id);
}
static DEVICE_ATTR_RO(chip_id);

static ssize_t cable_connected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	return sysfs_emit(buf, "%d\n", lt->cable_connected ? 1 : 0);
}
static DEVICE_ATTR_RO(cable_connected);

static ssize_t detected_timings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	lt6911c_refresh_status(lt, false, true);
	return sysfs_emit(buf,
			  "standby=%d connected=%d width=%u height=%u fps=%u htotal=%u vtotal=%u pixel_clk=%llu link_freq=%lld\n",
			  lt->powered ? 0 : 1,
			  lt->cable_connected ? 1 : 0,
			  lt->cur_mode.width, lt->cur_mode.height, lt->cur_mode.fps,
			  lt->cur_mode.htotal, lt->cur_mode.vtotal,
			  lt->cur_mode.pixel_clk, lt->cur_mode.link_freq);
}
static DEVICE_ATTR_RO(detected_timings);

static struct attribute *fv01_lt6911c_attrs[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_cable_connected.attr,
	&dev_attr_detected_timings.attr,
	NULL
};
ATTRIBUTE_GROUPS(fv01_lt6911c);

static int lt6911c_parse_fwnode(struct fv01_lt6911c *lt)
{
	struct device *dev = &lt->client->dev;
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(dev, -EPROBE_DEFER, "OF graph endpoint missing\n");

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (vep.bus.mipi_csi2.num_data_lanes != LT6911C_DEFAULT_LANES) {
		v4l2_fwnode_endpoint_free(&vep);
		return dev_err_probe(dev, -EINVAL,
				     "expected 4 MIPI lanes, got %u\n",
				     vep.bus.mipi_csi2.num_data_lanes);
	}

	lt->cur_mode.lanes = vep.bus.mipi_csi2.num_data_lanes;
	v4l2_fwnode_endpoint_free(&vep);
	return 0;
}

static int lt6911c_init_controls(struct fv01_lt6911c *lt)
{
	struct v4l2_ctrl_handler *hdl = &lt->ctrl_handler;
	s64 pixel_rate = lt->cur_mode.link_freq;

	v4l2_ctrl_handler_init(hdl, 2);

	lt->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
					       ARRAY_SIZE(fv01_lt6911c_link_freqs) - 1,
					       0, fv01_lt6911c_link_freqs);
	if (lt->link_freq)
		lt->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	lt->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					   25000000ULL, 600000000ULL, 1, pixel_rate);
	if (lt->pixel_rate)
		lt->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		int err = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return err;
	}

	lt->sd.ctrl_handler = hdl;
	return 0;
}

static int fv01_lt6911c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fv01_lt6911c *lt;
	int ret;

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	lt->client = client;
	lt->det_irq = -EINVAL;
	lt->event_irq = -EINVAL;
	mutex_init(&lt->lock);
	mutex_init(&lt->power_lock);
	INIT_DELAYED_WORK(&lt->status_work, lt6911c_status_work_fn);
	lt6911c_set_default_mode(lt);

	ret = lt6911c_parse_fwnode(lt);
	if (ret)
		return ret;

	lt->vdd3v3 = devm_regulator_get_optional(dev, "vdd3v3");
	if (IS_ERR(lt->vdd3v3)) {
		if (PTR_ERR(lt->vdd3v3) == -ENODEV)
			lt->vdd3v3 = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(lt->vdd3v3),
					     "failed to get vdd3v3 regulator\n");
	}

	lt->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->reset_gpio),
				     "failed to get reset gpio\n");

	lt->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(lt->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->enable_gpio),
				     "failed to get enable gpio\n");

	lt->det_gpio = devm_gpiod_get_optional(dev, "det", GPIOD_IN);
	if (IS_ERR(lt->det_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->det_gpio),
				     "failed to get det gpio\n");

	lt->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(lt->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->irq_gpio),
				     "failed to get irq gpio\n");

	if (lt->vdd3v3) {
		ret = regulator_enable(lt->vdd3v3);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vdd3v3\n");
		ret = devm_add_action_or_reset(dev, lt6911c_regulator_disable, lt->vdd3v3);
		if (ret)
			return ret;
	}

	dev_info(dev,
		 "LT6911C held in low-power Hi-Z standby (awaiting HDMI-IN hotplug or stream)\n");

	v4l2_i2c_subdev_init(&lt->sd, client, &lt6911c_subdev_ops);
	lt->sd.dev = dev;
	lt->sd.internal_ops = &lt6911c_internal_ops;
	lt->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	lt->sd.entity.ops = &lt6911c_entity_ops;
	/*
	 * Qualcomm CAMSS camss_find_sensor_pad() requires MEDIA_ENT_F_CAM_SENSOR
	 * when walking upstream from CSIPHY to obtain link_freq and pixel_rate.
	 */
	lt->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = lt6911c_init_controls(lt);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init V4L2 controls\n");

	lt->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&lt->sd.entity, 1, &lt->pad);
	if (ret) {
		dev_err(dev, "failed to init media pads: %d\n", ret);
		goto err_ctrl_free;
	}

	ret = v4l2_subdev_init_finalize(&lt->sd);
	if (ret) {
		dev_err(dev, "failed to finalize v4l2 subdev: %d\n", ret);
		goto err_media_cleanup;
	}

	lt6911c_refresh_status(lt, false, false);

	if (lt->det_gpio) {
		int irq = gpiod_to_irq(lt->det_gpio);

		if (irq > 0) {
			ret = request_threaded_irq(irq, NULL, lt6911c_irq_handler,
						   IRQF_TRIGGER_RISING |
						   IRQF_TRIGGER_FALLING |
						   IRQF_ONESHOT,
						   "lt6911c-det", lt);
			if (ret)
				dev_warn(dev, "failed to request det IRQ %d: %d\n", irq, ret);
			else
				lt->det_irq = irq;
		}
	}

	if (lt->irq_gpio) {
		int irq = gpiod_to_irq(lt->irq_gpio);

		if (irq > 0) {
			ret = request_threaded_irq(irq, NULL, lt6911c_irq_handler,
						   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						   "lt6911c-irq", lt);
			if (ret)
				dev_warn(dev, "failed to request event IRQ %d: %d\n", irq, ret);
			else
				lt->event_irq = irq;
		}
	}

	ret = v4l2_async_register_subdev_sensor(&lt->sd);
	if (ret) {
		dev_err(dev, "failed to register async subdev: %d\n", ret);
		goto err_free_irqs;
	}

	return 0;

err_free_irqs:
	if (lt->event_irq >= 0)
		free_irq(lt->event_irq, lt);
	if (lt->det_irq >= 0)
		free_irq(lt->det_irq, lt);
	cancel_delayed_work_sync(&lt->status_work);
	v4l2_subdev_cleanup(&lt->sd);
err_media_cleanup:
	media_entity_cleanup(&lt->sd.entity);
err_ctrl_free:
	v4l2_ctrl_handler_free(&lt->ctrl_handler);
	return ret;
}

static void fv01_lt6911c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	if (lt->event_irq >= 0)
		free_irq(lt->event_irq, lt);
	if (lt->det_irq >= 0)
		free_irq(lt->det_irq, lt);
	cancel_delayed_work_sync(&lt->status_work);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&lt->ctrl_handler);
	mutex_lock(&lt->power_lock);
	lt->streaming = false;
	lt6911c_hw_power_off(lt);
	mutex_unlock(&lt->power_lock);
}

static int fv01_lt6911c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	if (lt->event_irq >= 0)
		disable_irq(lt->event_irq);
	if (lt->det_irq >= 0)
		disable_irq(lt->det_irq);
	cancel_delayed_work_sync(&lt->status_work);
	mutex_lock(&lt->power_lock);
	lt6911c_hw_power_off(lt);
	mutex_unlock(&lt->power_lock);
	return 0;
}

static int fv01_lt6911c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fv01_lt6911c *lt = to_fv01_lt6911c(sd);

	mutex_lock(&lt->power_lock);
	if (lt->streaming) {
		lt6911c_hw_power_on(lt);
		mutex_lock(&lt->lock);
		lt6911c_set_i2c_gate(lt, false);
		mutex_unlock(&lt->lock);
	}
	mutex_unlock(&lt->power_lock);

	if (lt->det_irq >= 0)
		enable_irq(lt->det_irq);
	if (lt->event_irq >= 0)
		enable_irq(lt->event_irq);
	if (lt->streaming)
		schedule_delayed_work(&lt->status_work,
				      msecs_to_jiffies(LT6911C_WORK_DELAY_MS));
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fv01_lt6911c_pm_ops,
				fv01_lt6911c_suspend,
				fv01_lt6911c_resume);

static const struct of_device_id fv01_lt6911c_of_match[] = {
	{ .compatible = "faiot,fv01-lt6911c" },
	{ .compatible = "lontium,lt6911uxc" },
	{ }
};
MODULE_DEVICE_TABLE(of, fv01_lt6911c_of_match);

static struct i2c_driver fv01_lt6911c_i2c_driver = {
	.driver = {
		.name = "fv01-lt6911c",
		.of_match_table = fv01_lt6911c_of_match,
		.dev_groups = fv01_lt6911c_groups,
		.pm = pm_sleep_ptr(&fv01_lt6911c_pm_ops),
	},
	.probe = fv01_lt6911c_probe,
	.remove = fv01_lt6911c_remove,
};
module_i2c_driver(fv01_lt6911c_i2c_driver);

MODULE_DESCRIPTION("FAIOT FV01 Lontium LT6911C/LT6911UXC HDMI-to-CSI2 Bridge Driver");
MODULE_LICENSE("GPL");
