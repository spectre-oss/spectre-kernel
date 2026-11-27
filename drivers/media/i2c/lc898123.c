// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 sunfish bring-up

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define LC898123_NAME			"lc898123"
#define LC898123_REG_STATUS		0xF100
#define LC898123_REG_SERVO		0xF010
#define LC898123_REG_OIS_MODE		0xF012
#define LC898123_REG_ROMINFO		0x8010
#define LC898123_IO_ADR_ACCESS		0xC000
#define LC898123_IO_DAT_ACCESS		0xD000
#define LC898123_SYSDSP_REMAP		0xD000AC
#define LC898123_SYSDSP_SOFTRES		0xD0006C
#define LC898123_SYSDSP_DSPDIV		0xD00014
#define LC898123_REMAP_BOOT		0x00001440U
#define LC898123_REG_DSP_INIT		0xF006

#define LC898123_STATUS_INIT_MASK	0x01000000U
#define LC898123_REMAP_RUNTIME		0x00001000U
#define LC898123_REMAP_RUNTIME_ALT	0x00001001U
#define LC898123_REG_GYRO_CMD		0xF014
#define LC898123_REG_GYRO_X		0x0278
#define LC898123_REG_GYRO_Y		0x027C
#define LC898123_GYRO_RAM_FIRST		0x0208U
#define LC898123_GYRO_RAM_LAST		0x0304U
#define LC898123_REG_NVR_OFF_A		0xF00A
#define LC898123_REG_NVR_CNT_B		0xF00B
#define LC898123_REG_NVR_CMD_C		0xF00C
#define LC898123_REG_NVR_CHKS_D	0xF00D
#define LC898123_NVR_LOAD_CMD		0x00000100U
#define LC898123_NVR_LOAD_BLOCKS	2U
#define LC898123_NVR_LOAD_POLL_MS	5000U

#define LC898123_FLASH_BASE		0xE07000U
#define LC898123_FLASH_RDATL		(LC898123_FLASH_BASE + 0x00)
#define LC898123_FLASH_ADR		(LC898123_FLASH_BASE + 0x10)
#define LC898123_FLASH_ACSCNT		(LC898123_FLASH_BASE + 0x14)
#define LC898123_FLASH_CMD		(LC898123_FLASH_BASE + 0x18)

#define LC898123_NVR2_CAL_BASE		0x00010040U
#define LC898123_NVR2_CAL_WORDS		64
#define LC898123_NVR2_BATCH		32
#define LC898123_CAL_GYRO_X		19
#define LC898123_CAL_GYRO_Y		20
#define LC898123_CAL_GYRO_FX		49
#define LC898123_CAL_GYRO_FY		50

struct lc898123_cal_map {
	u8 idx;
	u16 reg;
};

static const struct lc898123_cal_map lc898123_cal_regs[] = {
	{ LC898123_CAL_GYRO_X, LC898123_REG_GYRO_X },
	{ LC898123_CAL_GYRO_Y, LC898123_REG_GYRO_Y },
	{ 21, 0x82B8 },
	{ 22, 0x8318 },
};

static bool lc898123_skip_status;
module_param_named(skip_status, lc898123_skip_status, bool, 0644);
MODULE_PARM_DESC(skip_status, "Skip 0xF100 init poll after OIS enable (debug)");

static bool lc898123_gyro_stub;
module_param_named(gyro_stub, lc898123_gyro_stub, bool, 0644);
MODULE_PARM_DESC(gyro_stub, "RamWrite 0x0278/0x027C during enable (offset regs — leave off)");

static bool lc898123_gyro_block;
module_param_named(gyro_block, lc898123_gyro_block, bool, 0644);
MODULE_PARM_DESC(gyro_block, "CamX ladder 0x0208..0x0304 (zeros + gyro @0x0278/0x027C)");

static bool lc898123_comm_timer;
module_param_named(comm_timer, lc898123_comm_timer, bool, 0644);
MODULE_PARM_DESC(comm_timer, "F100 poll: full gyro_block every comm_period_ms + pair between");

static unsigned int lc898123_comm_period_ms = 5;
module_param_named(comm_period_ms, lc898123_comm_period_ms, uint, 0644);
MODULE_PARM_DESC(comm_period_ms, "Full gyro_block interval during F100 poll (ms, min 5, 0=every iter)");

static unsigned int lc898123_poll_ms(unsigned int ms)
{
	return lc898123_skip_status ? 0 : ms;
}

static unsigned int lc898123_comm_period(void)
{
	unsigned int period = lc898123_comm_period_ms ? lc898123_comm_period_ms : 1;

	if (period < 5)
		period = 5;
	return period;
}

static bool lc898123_cal_ladder = true;
module_param_named(cal_ladder, lc898123_cal_ladder, bool, 0644);
MODULE_PARM_DESC(cal_ladder, "NVR2 cal[i] -> ladder 0x0208+i*4 (64 words); gyro live on [19]/[20]");

static bool lc898123_gyro_cal;
module_param_named(gyro_cal, lc898123_gyro_cal, bool, 0644);
MODULE_PARM_DESC(gyro_cal, "Run F014=0 before servo (PhoneUpdate F40_GyroReCalib; needs feed)");

static bool lc898123_gyro_cal_post;
module_param_named(gyro_cal_post, lc898123_gyro_cal_post, bool, 0644);
MODULE_PARM_DESC(gyro_cal_post, "F014=0 after F012=1 during final F100 poll (CamX timer window)");

static unsigned int lc898123_remap_val = LC898123_REMAP_RUNTIME;
module_param_named(remap_val, lc898123_remap_val, uint, 0644);
MODULE_PARM_DESC(remap_val, "SYSDSP_REMAP value (default 0x1000, try 0x1001)");

static unsigned int lc898123_probe_poll_ms;
module_param_named(probe_poll_ms, lc898123_probe_poll_ms, uint, 0644);
MODULE_PARM_DESC(probe_poll_ms, "Poll 0xF100 after probe remap (ms, 0=off)");

static unsigned int lc898123_remap_wait_ms = 25;
module_param_named(remap_wait_ms, lc898123_remap_wait_ms, uint, 0644);
MODULE_PARM_DESC(remap_wait_ms, "ms after SYSDSP_REMAP (PhoneUpdate WitTim 25; flash path uses 1000)");

static bool lc898123_servo_on = true;
module_param_named(servo_on, lc898123_servo_on, bool, 0644);
MODULE_PARM_DESC(servo_on, "F010=0 before remap, then F010=3 before OIS on (PhoneUpdate F40_WrGyroOffsetData)");

static bool lc898123_hard_reset;
module_param_named(hard_reset, lc898123_hard_reset, bool, 0644);
MODULE_PARM_DESC(hard_reset, "F010=0 and F012=0 before enable (recover drifted ROMINFO session)");

static bool lc898123_nvr_load_dsp;
module_param_named(nvr_load_dsp, lc898123_nvr_load_dsp, bool, 0644);
MODULE_PARM_DESC(nvr_load_dsp, "PhoneUpdate flash step6 only: F00A=0x10000 MagicCode load (not runtime NVR2)");

static bool lc898123_load_nvr_cal = true;
module_param_named(load_nvr_cal, lc898123_load_nvr_cal, bool, 0644);
MODULE_PARM_DESC(load_nvr_cal, "Read NVR2 cal (PhoneUpdate F40_ReadCalData) after remap; push gyro off to 0x0278/0x027C");

struct lc898123_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	bool ois_on;
	struct regulator *vcc;
	s16 gyro_x;
	s16 gyro_y;
	u32 cal[LC898123_NVR2_CAL_WORDS];
	u32 cal_chksum;
	bool cal_loaded;
	struct dentry *dbg_dir;
};

static inline struct lc898123_device *sd_to_lc898123(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lc898123_device, sd);
}

static inline struct lc898123_device *ctrl_to_lc898123(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct lc898123_device, ctrls);
}

static unsigned int lc898123_i2c_mode;
module_param_named(i2c_mode, lc898123_i2c_mode, uint, 0644);
MODULE_PARM_DESC(i2c_mode, "0=6byte BE (default); 1=split addr then data; 2=6byte LE data");

static int lc898123_ram_write(struct i2c_client *client, u16 addr, u32 val)
{
	u8 buf[6];
	u8 adata[4];
	struct i2c_msg msgs[2];
	int ret;

	if (lc898123_i2c_mode == 2) {
		buf[0] = addr >> 8;
		buf[1] = addr & 0xff;
		buf[2] = val & 0xff;
		buf[3] = (val >> 8) & 0xff;
		buf[4] = (val >> 16) & 0xff;
		buf[5] = val >> 24;
		return i2c_master_send(client, buf, sizeof(buf)) == sizeof(buf) ? 0 : -EIO;
	}

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;
	buf[2] = val >> 24;
	buf[3] = (val >> 16) & 0xff;
	buf[4] = (val >> 8) & 0xff;
	buf[5] = val & 0xff;

	if (lc898123_i2c_mode == 1) {
		adata[0] = buf[2];
		adata[1] = buf[3];
		adata[2] = buf[4];
		adata[3] = buf[5];
		msgs[0].addr = client->addr;
		msgs[0].flags = 0;
		msgs[0].len = 2;
		msgs[0].buf = buf;
		msgs[1].addr = client->addr;
		msgs[1].flags = 0;
		msgs[1].len = 4;
		msgs[1].buf = adata;
		ret = i2c_transfer(client->adapter, msgs, 2);
		return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
	}

	if (i2c_master_send(client, buf, sizeof(buf)) == sizeof(buf))
		return 0;

	return -EIO;
}

static int lc898123_ram_read(struct i2c_client *client, u16 addr, u32 *val)
{
	u8 reg[2];
	u8 data[4];
	struct i2c_msg msg[2];
	int ret;

	reg[0] = addr >> 8;
	reg[1] = addr & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(reg);
	msg[0].buf = reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(data);
	msg[1].buf = data;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		return ret < 0 ? ret : -EIO;

	*val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	return 0;
}

static int lc898123_io_write(struct i2c_client *client, u32 io_addr, u32 val)
{
	int ret;

	ret = lc898123_ram_write(client, LC898123_IO_ADR_ACCESS, io_addr);
	if (ret)
		return ret;
	return lc898123_ram_write(client, LC898123_IO_DAT_ACCESS, val);
}

static int lc898123_io_read(struct i2c_client *client, u32 io_addr, u32 *val)
{
	int ret;

	ret = lc898123_ram_write(client, LC898123_IO_ADR_ACCESS, io_addr);
	if (ret)
		return ret;
	return lc898123_ram_read(client, LC898123_IO_DAT_ACCESS, val);
}

static int lc898123_read_nvr2_cal(struct i2c_client *client, struct lc898123_device *dev)
{
	unsigned int idx = 0;

	while (idx < LC898123_NVR2_CAL_WORDS) {
		unsigned int n;

		if (lc898123_io_write(client, LC898123_FLASH_ACSCNT, LC898123_NVR2_BATCH - 1))
			return -EIO;
		if (lc898123_io_write(client, LC898123_FLASH_ADR,
				      LC898123_NVR2_CAL_BASE + idx))
			return -EIO;
		if (lc898123_io_write(client, LC898123_FLASH_CMD, 1))
			return -EIO;
		if (lc898123_ram_write(client, LC898123_IO_ADR_ACCESS, LC898123_FLASH_RDATL))
			return -EIO;

		for (n = 0; n < LC898123_NVR2_BATCH && idx < LC898123_NVR2_CAL_WORDS;
		     n++, idx++) {
			if (lc898123_ram_read(client, LC898123_IO_DAT_ACCESS,
					      &dev->cal[idx]))
				return -EIO;
			dev->cal_chksum += dev->cal[idx];
		}
	}

	dev->cal_loaded = true;
	return 0;
}

static u32 lc898123_cal_gyro_word(const struct lc898123_device *dev, unsigned int idx,
				  unsigned int fidx)
{
	if (dev->cal[idx] != 0xffffffffU)
		return dev->cal[idx];
	if (dev->cal[fidx] != 0xffffffffU)
		return dev->cal[fidx];
	return 0;
}

static u32 lc898123_gyro_word(const struct lc898123_device *dev, bool x_axis)
{
	u32 cal;
	s16 live;

	if (x_axis) {
		cal = lc898123_cal_gyro_word(dev, LC898123_CAL_GYRO_X, LC898123_CAL_GYRO_FX);
		live = dev->gyro_x;
	} else {
		cal = lc898123_cal_gyro_word(dev, LC898123_CAL_GYRO_Y, LC898123_CAL_GYRO_FY);
		live = dev->gyro_y;
	}

	if (live)
		return (cal & 0xffff) | ((u32)(live & 0xffff) << 16);
	if (cal)
		return cal;
	return (u32)(live & 0xffff) << 16;
}

static int lc898123_gyro_feed_pair(struct i2c_client *client, struct lc898123_device *dev)
{
	u32 gx = lc898123_gyro_word(dev, true);
	u32 gy = lc898123_gyro_word(dev, false);
	int ret;

	ret = lc898123_ram_write(client, LC898123_REG_GYRO_X, gx);
	if (ret)
		return ret;
	return lc898123_ram_write(client, LC898123_REG_GYRO_Y, gy);
}

static u32 lc898123_ladder_word(struct lc898123_device *dev, unsigned int idx)
{
	if (idx == LC898123_CAL_GYRO_X)
		return lc898123_gyro_word(dev, true);
	if (idx == LC898123_CAL_GYRO_Y)
		return lc898123_gyro_word(dev, false);
	if (lc898123_cal_ladder && dev->cal_loaded && dev->cal[idx] != 0xffffffffU)
		return dev->cal[idx];
	return 0;
}

static int lc898123_gyro_feed_block(struct i2c_client *client, struct lc898123_device *dev)
{
	u16 addr;
	int ret;

	for (addr = LC898123_GYRO_RAM_FIRST; addr <= LC898123_GYRO_RAM_LAST; addr += 4) {
		unsigned int idx = (addr - LC898123_GYRO_RAM_FIRST) / 4;
		u32 val = lc898123_ladder_word(dev, idx);

		ret = lc898123_ram_write(client, addr, val);
		if (ret)
			return ret;
	}
	return 0;
}

static int lc898123_gyro_feed(struct i2c_client *client, struct lc898123_device *dev)
{
	if (lc898123_gyro_block)
		return lc898123_gyro_feed_block(client, dev);
	return lc898123_gyro_feed_pair(client, dev);
}

static int lc898123_gyro_store(struct lc898123_device *dev, struct i2c_client *client,
			       const char *kbuf, size_t count)
{
	long x, y;
	int n;

	n = sscanf(kbuf, "%ld,%ld", &x, &y);
	if (n == 1) {
		dev->gyro_x = (s16)x;
		dev->gyro_y = 0;
	} else if (n == 2) {
		dev->gyro_x = (s16)x;
		dev->gyro_y = (s16)y;
	} else {
		return -EINVAL;
	}

	if (client)
		lc898123_gyro_feed(client, dev);
	return count;
}

static u32 lc898123_cal_padded_chksum(const struct lc898123_device *dev)
{
	u32 sum = dev->cal_chksum;
	unsigned int i;

	for (i = LC898123_NVR2_CAL_WORDS * sizeof(u32); i < 320; i++)
		sum += 0xffU;
	return sum;
}

static int lc898123_nvr_load_to_dsp(struct i2c_client *client, struct lc898123_device *dev)
{
	static const u32 bases[] = { 0x00010000U };
	u32 expect, got, gx, gy;
	unsigned int b, i;
	int ret = -ETIMEDOUT;

	expect = lc898123_cal_padded_chksum(dev);

	for (b = 0; b < ARRAY_SIZE(bases); b++) {
		ret = lc898123_ram_write(client, LC898123_REG_NVR_OFF_A, bases[b]);
		if (ret)
			continue;
		ret = lc898123_ram_write(client, LC898123_REG_NVR_CNT_B, LC898123_NVR_LOAD_BLOCKS);
		if (ret)
			continue;
		ret = lc898123_ram_write(client, LC898123_REG_NVR_CMD_C, LC898123_NVR_LOAD_CMD);
		if (ret)
			continue;

		got = ~0U;
		for (i = 0; i < LC898123_NVR_LOAD_POLL_MS; i++) {
			if (lc898123_ram_read(client, LC898123_REG_NVR_CMD_C, &got))
				return -EIO;
			if (got == 0)
				break;
			usleep_range(1000, 2000);
		}
		if (got)
			continue;

		if (lc898123_ram_read(client, LC898123_REG_NVR_CHKS_D, &got))
			return -EIO;

		dev_info(&client->dev,
			 "NVR2 DSP load F00A=0x%x F00D=0x%08x expect=0x%08x cal_chksum=0x%08x\n",
			 bases[b], got, expect, dev->cal_chksum);
		if (expect && got != expect)
			dev_warn(&client->dev, "NVR2 DSP load checksum mismatch\n");
		ret = 0;
		break;
	}

	if (ret)
		dev_warn(&client->dev, "NVR2 DSP load timeout F00C=0x%08x\n", got);

	gx = lc898123_cal_gyro_word(dev, LC898123_CAL_GYRO_X, LC898123_CAL_GYRO_FX);
	gy = lc898123_cal_gyro_word(dev, LC898123_CAL_GYRO_Y, LC898123_CAL_GYRO_FY);
	dev->gyro_x = (s16)(gx >> 16);
	dev->gyro_y = (s16)(gy >> 16);
	return ret;
}

static int lc898123_apply_nvr_cal_regs(struct i2c_client *client, struct lc898123_device *dev)
{
	unsigned int i;
	int ret, n = 0;

	if (!dev->cal_loaded)
		return -ENODATA;

	for (i = 0; i < ARRAY_SIZE(lc898123_cal_regs); i++) {
		u8 idx = lc898123_cal_regs[i].idx;
		u32 val = lc898123_cal_gyro_word(dev, idx, idx);

		if (val == 0 && dev->cal[idx] == 0xffffffffU)
			continue;
		ret = lc898123_ram_write(client, lc898123_cal_regs[i].reg, val);
		if (ret)
			return ret;
		n++;
	}

	if (n && lc898123_gyro_stub)
		lc898123_gyro_feed(client, dev);

	dev_info(&client->dev, "NVR2 applied %u mapped cal registers\n", n);
	return 0;
}

static int lc898123_do_gyro_cal(struct i2c_client *client, struct lc898123_device *dev)
{
	unsigned int i;

	if (!dev->cal_loaded && lc898123_read_nvr2_cal(client, dev))
		return -EIO;

	if (lc898123_ram_write(client, LC898123_REG_GYRO_CMD, 0))
		return -EIO;

	for (i = 0; i < 200; i++) {
		u32 status;

		if (lc898123_gyro_stub)
			lc898123_gyro_feed(client, dev);
		if (lc898123_ram_read(client, LC898123_REG_STATUS, &status))
			return -EIO;
		if (!(status & LC898123_STATUS_INIT_MASK))
			return 0;
		usleep_range(1000, 2000);
	}
	return -ETIMEDOUT;
}

static int lc898123_wait_ready(struct i2c_client *client, struct lc898123_device *dev,
			       unsigned int ms, bool feed_gyro)
{
	u32 status;
	unsigned int i;

	for (i = 0; i < ms; i++) {
		if (feed_gyro) {
			unsigned int period = lc898123_comm_period();

			if (lc898123_comm_timer && lc898123_gyro_block && !(i % period))
				lc898123_gyro_feed_block(client, dev);
			else
				lc898123_gyro_feed_pair(client, dev);
		}
		if (lc898123_ram_read(client, LC898123_REG_STATUS, &status))
			return -EIO;
		if (!(status & LC898123_STATUS_INIT_MASK))
			return 0;
		usleep_range(1000, 2000);
	}
	return -ETIMEDOUT;
}

static bool lc898123_dsp_boot;
module_param_named(dsp_boot, lc898123_dsp_boot, bool, 0644);
MODULE_PARM_DESC(dsp_boot, "PhoneUpdate flash step0: remap 0x1440, SOFTRES, F006=0, DSPDIV=1, then runtime remap");

static int lc898123_dsp_boot_seq(struct i2c_client *client)
{
	u32 val;
	int ret;

	ret = lc898123_io_write(client, LC898123_SYSDSP_REMAP, LC898123_REMAP_BOOT);
	if (ret)
		return ret;
	msleep(25);
	ret = lc898123_io_read(client, LC898123_SYSDSP_SOFTRES, &val);
	if (ret)
		return ret;
	val &= ~0x1000U;
	ret = lc898123_io_write(client, LC898123_SYSDSP_SOFTRES, val);
	if (ret)
		return ret;
	ret = lc898123_ram_write(client, LC898123_REG_DSP_INIT, 0);
	if (ret)
		return ret;
	ret = lc898123_io_write(client, LC898123_SYSDSP_DSPDIV, 1);
	if (ret)
		return ret;
	return 0;
}

static int lc898123_remap(struct i2c_client *client)
{
	int ret;

	if (lc898123_dsp_boot) {
		ret = lc898123_dsp_boot_seq(client);
		if (ret)
			return ret;
	}

	ret = lc898123_io_write(client, LC898123_SYSDSP_REMAP, lc898123_remap_val);
	if (ret)
		return ret;
	if (lc898123_remap_wait_ms)
		msleep(lc898123_remap_wait_ms);
	return 0;
}

static int lc898123_set_ois(struct lc898123_device *dev, bool on)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	u32 status, srv = 0, ois = 0;
	int ret;

	if (!on) {
		ret = lc898123_ram_write(client, LC898123_REG_OIS_MODE, 0);
		if (ret)
			return ret;
		dev->ois_on = false;
		return 0;
	}

	lc898123_ram_read(client, LC898123_REG_SERVO, &srv);
	lc898123_ram_read(client, LC898123_REG_OIS_MODE, &ois);

	if (lc898123_hard_reset) {
		lc898123_ram_write(client, LC898123_REG_OIS_MODE, 0);
		lc898123_ram_write(client, LC898123_REG_SERVO, 0);
	}

	ret = lc898123_ram_write(client, LC898123_REG_SERVO, 0);
	if (ret)
		return ret;

	ret = lc898123_remap(client);
	if (ret)
		return ret;

	if (lc898123_load_nvr_cal) {
		dev->cal_chksum = 0;
		ret = lc898123_read_nvr2_cal(client, dev);
		if (ret) {
			dev_warn(&client->dev, "NVR2 read: %d\n", ret);
		} else {
			if (lc898123_nvr_load_dsp) {
				ret = lc898123_nvr_load_to_dsp(client, dev);
				if (ret)
					dev_warn(&client->dev, "NVR2 DSP load: %d\n", ret);
			}
			if (!lc898123_cal_ladder)
				lc898123_apply_nvr_cal_regs(client, dev);
		}
	}

	if (lc898123_gyro_cal) {
		ret = lc898123_do_gyro_cal(client, dev);
		if (ret && !lc898123_skip_status)
			dev_dbg(&client->dev, "gyro cal F014 poll: %d\n", ret);
	}

	if (lc898123_gyro_stub)
		lc898123_gyro_feed(client, dev);

	ret = lc898123_wait_ready(client, dev, lc898123_poll_ms(2000), lc898123_gyro_stub);
	if (ret && !lc898123_skip_status)
		dev_dbg(&client->dev, "post-remap F100 poll: %d\n", ret);

	if (lc898123_servo_on) {
		u32 val = srv == 3 ? 3 : srv ? srv : 3;

		ret = lc898123_ram_write(client, LC898123_REG_SERVO, val);
		if (ret)
			return ret;
		ret = lc898123_wait_ready(client, dev, lc898123_poll_ms(2000), lc898123_gyro_stub);
		if (ret && !lc898123_skip_status)
			dev_dbg(&client->dev, "post-servo F100 poll: %d\n", ret);
	}

	ret = lc898123_ram_write(client, LC898123_REG_OIS_MODE, 1);
	if (ret)
		return ret;

	if (lc898123_gyro_cal_post && lc898123_ram_write(client, LC898123_REG_GYRO_CMD, 0))
		return -EIO;

	if (lc898123_skip_status && lc898123_gyro_stub)
		lc898123_gyro_feed(client, dev);

	ret = lc898123_wait_ready(client, dev, lc898123_poll_ms(5000), lc898123_gyro_stub);
	if (ret && !lc898123_skip_status) {
		if (!lc898123_ram_read(client, LC898123_REG_STATUS, &status))
			dev_warn(&client->dev,
				 "OIS enable timeout 0xF100=0x%08x (ois was 0x%x srv 0x%x)\n",
				 status, ois, srv);
		return -EAGAIN;
	}

	if (ret)
		dev_warn(&client->dev, "OIS enable: 0xF100 init bit still set (skip_status=1)\n");

	dev->ois_on = true;
	return 0;
}

static int lc898123_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898123_device *dev = ctrl_to_lc898123(ctrl);

	if (ctrl->id == V4L2_CID_IMAGE_STABILIZATION)
		return lc898123_set_ois(dev, ctrl->val);

	return -EINVAL;
}

static const struct v4l2_ctrl_ops lc898123_ctrl_ops = {
	.s_ctrl = lc898123_set_ctrl,
};

static int lc898123_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_get_sync(sd->dev);
}

static int lc898123_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops lc898123_int_ops = {
	.open = lc898123_open,
	.close = lc898123_close,
};

static const struct v4l2_subdev_core_ops lc898123_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lc898123_ops = {
	.core = &lc898123_core_ops,
};

static ssize_t lc898123_gyro_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct lc898123_device *dev = file->private_data;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	char kbuf[32];
	int ret;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	ret = lc898123_gyro_store(dev, client, kbuf, count);
	return ret < 0 ? ret : count;
}

static ssize_t gyro_inject_store(struct device *d, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(d);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898123_device *dev = sd_to_lc898123(sd);
	char kbuf[32];
	int ret;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	memcpy(kbuf, buf, count);
	kbuf[count] = '\0';
	ret = lc898123_gyro_store(dev, client, kbuf, count);
	return ret < 0 ? ret : count;
}

static DEVICE_ATTR_WO(gyro_inject);

static const struct file_operations lc898123_gyro_fops = {
	.open = simple_open,
	.write = lc898123_gyro_write,
	.llseek = noop_llseek,
};

static int lc898123_status_show(struct seq_file *s, void *unused)
{
	struct lc898123_device *dev = s->private;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	u32 rom = 0, status = 0, srv = 0, ois = 0, f014 = 0, gx = 0, gy = 0;

	if (!lc898123_ram_read(client, LC898123_REG_ROMINFO, &rom))
		seq_printf(s, "rominfo=0x%08x\n", rom);
	if (!lc898123_ram_read(client, LC898123_REG_STATUS, &status))
		seq_printf(s, "f100=0x%08x init=%u\n", status,
			   !!(status & LC898123_STATUS_INIT_MASK));
	if (!lc898123_ram_read(client, LC898123_REG_SERVO, &srv))
		seq_printf(s, "f010=0x%08x\n", srv);
	if (!lc898123_ram_read(client, LC898123_REG_OIS_MODE, &ois))
		seq_printf(s, "f012=0x%08x\n", ois);
	if (!lc898123_ram_read(client, LC898123_REG_GYRO_CMD, &f014))
		seq_printf(s, "f014=0x%08x\n", f014);
	if (!lc898123_ram_read(client, LC898123_REG_GYRO_X, &gx))
		seq_printf(s, "r0278=0x%08x\n", gx);
	if (!lc898123_ram_read(client, LC898123_REG_GYRO_Y, &gy))
		seq_printf(s, "r027c=0x%08x\n", gy);
	seq_printf(s, "ois_on=%d gyro=%d,%d\n", dev->ois_on, dev->gyro_x, dev->gyro_y);
	return 0;
}

static int lc898123_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, lc898123_status_show, inode->i_private);
}

static const struct file_operations lc898123_status_fops = {
	.open = lc898123_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t lc898123_ram_dbg_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct lc898123_device *dev = file->private_data;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	char kbuf[48];
	unsigned int addr, val;
	int ret;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	if (sscanf(kbuf, "%x %x", &addr, &val) != 2)
		return -EINVAL;
	if (addr > 0xffff)
		return -EINVAL;
	ret = lc898123_ram_write(client, addr, val);
	return ret < 0 ? ret : count;
}

static const struct file_operations lc898123_ram_fops = {
	.open = simple_open,
	.write = lc898123_ram_dbg_write,
	.llseek = noop_llseek,
};

static void lc898123_dbgfs_create(struct lc898123_device *dev, struct i2c_client *client)
{
	char name[32];

	if (!debugfs_initialized())
		return;
	scnprintf(name, sizeof(name), "lc898123-%s", dev_name(&client->dev));
	dev->dbg_dir = debugfs_create_dir(name, NULL);
	if (!dev->dbg_dir)
		return;
	debugfs_create_file("gyro", 0200, dev->dbg_dir, dev, &lc898123_gyro_fops);
	debugfs_create_file("status", 0444, dev->dbg_dir, dev, &lc898123_status_fops);
	debugfs_create_file("ram", 0200, dev->dbg_dir, dev, &lc898123_ram_fops);
}

static void lc898123_dbgfs_remove(struct lc898123_device *dev)
{
	if (dev->dbg_dir) {
		debugfs_remove_recursive(dev->dbg_dir);
		dev->dbg_dir = NULL;
	}
}

static void lc898123_subdev_cleanup(struct lc898123_device *dev)
{
	v4l2_async_unregister_subdev(&dev->sd);
	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrls);
}

static int lc898123_probe(struct i2c_client *client)
{
	struct lc898123_device *dev;
	u32 rominfo, status = 0;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	v4l2_ctrl_handler_init(&dev->ctrls, 1);
	v4l2_ctrl_new_std(&dev->ctrls, &lc898123_ctrl_ops,
			    V4L2_CID_IMAGE_STABILIZATION, 0, 1, 1, 0);
	if (dev->ctrls.error) {
		ret = dev->ctrls.error;
		v4l2_ctrl_handler_free(&dev->ctrls);
		return ret;
	}

	dev->sd.ctrl_handler = &dev->ctrls;
	v4l2_i2c_subdev_init(&dev->sd, client, &lc898123_ops);
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->sd.internal_ops = &lc898123_int_ops;
	dev->sd.entity.function = MEDIA_ENT_F_LENS;

	dev->vcc = devm_regulator_get_optional(&client->dev, "vcc");
	if (IS_ERR(dev->vcc)) {
		if (PTR_ERR(dev->vcc) != -ENODEV)
			return PTR_ERR(dev->vcc);
		dev->vcc = NULL;
	}

	if (dev->vcc) {
		ret = regulator_enable(dev->vcc);
		if (ret)
			return ret;
	}

	usleep_range(10000, 12000);

	if (!lc898123_ram_read(client, 0x8010, &rominfo))
		dev_info(&client->dev, "ROMINFO probe 0x8010 = 0x%08x\n", rominfo);
	else
		dev_dbg(&client->dev, "ROMINFO read failed (needs firmware init)\n");

	if (!lc898123_ram_read(client, LC898123_REG_STATUS, &status))
		dev_info(&client->dev, "STATUS probe 0xF100 = 0x%08x\n", status);

	if (lc898123_probe_poll_ms && !lc898123_remap(client)) {
		unsigned int i, ms = lc898123_probe_poll_ms;
		u32 after = status;

		for (i = 0; i < ms; i++) {
			if (lc898123_ram_read(client, LC898123_REG_STATUS, &after))
				break;
			if (!(after & LC898123_STATUS_INIT_MASK))
				break;
			usleep_range(1000, 2000);
		}
		if (after != status)
			dev_info(&client->dev, "STATUS after remap 0xF100 = 0x%08x\n", after);
	}

	ret = media_entity_pads_init(&dev->sd.entity, 0, NULL);
	if (ret)
		goto err_reg;

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret)
		goto err_media;

	lc898123_dbgfs_create(dev, client);

	ret = device_create_file(&client->dev, &dev_attr_gyro_inject);
	if (ret)
		dev_warn(&client->dev, "gyro_inject sysfs: %d\n", ret);

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_media:
	media_entity_cleanup(&dev->sd.entity);
err_reg:
	if (dev->vcc)
		regulator_disable(dev->vcc);
	v4l2_ctrl_handler_free(&dev->ctrls);
	return ret;
}

static void lc898123_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898123_device *dev = sd_to_lc898123(sd);

	pm_runtime_disable(&client->dev);
	device_remove_file(&client->dev, &dev_attr_gyro_inject);
	lc898123_dbgfs_remove(dev);
	lc898123_subdev_cleanup(dev);
	if (dev->vcc)
		regulator_disable(dev->vcc);
}

static const struct of_device_id lc898123_of_table[] = {
	{ .compatible = "onsemi,lc898123f40" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lc898123_of_table);

static struct i2c_driver lc898123_i2c_driver = {
	.driver = {
		.name = LC898123_NAME,
		.of_match_table = lc898123_of_table,
	},
	.probe = lc898123_probe,
	.remove = lc898123_remove,
};

module_i2c_driver(lc898123_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898123F40 OIS controller");
MODULE_LICENSE("GPL");
