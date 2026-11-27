// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 sunfish bring-up

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define LC898219_NAME		"lc898219"
#define LC898219_MAX_FOCUS_POS	1023
#define LC898219_FOCUS_STEPS	1
#define LC898219_CTRL_STEPS	16
#define LC898219_CTRL_DELAY_US	1000
#define LC898219_REG_DAC	0x84

struct lc898219_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	u16 current_val;
	struct regulator *vcc;
};

static inline struct lc898219_device *sd_to_lc898219(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct lc898219_device, sd);
}

static inline struct lc898219_device *ctrl_to_lc898219(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct lc898219_device, ctrls_vcm);
}

static int lc898219_write_pos(struct i2c_client *client, u16 pos)
{
	u8 buf4[4];
	u8 buf3[3];

	pos &= 0x03ff;

	buf4[0] = 0x00;
	buf4[1] = LC898219_REG_DAC;
	buf4[2] = (u8)(pos >> 8);
	buf4[3] = (u8)(pos & 0xff);
	if (i2c_master_send(client, buf4, sizeof(buf4)) == sizeof(buf4))
		return 0;

	buf3[0] = LC898219_REG_DAC;
	buf3[1] = (u8)(pos >> 8);
	buf3[2] = (u8)(pos & 0xff);
	if (i2c_master_send(client, buf3, sizeof(buf3)) == sizeof(buf3))
		return 0;

	return -EIO;
}

static int lc898219_set_pos(struct lc898219_device *dev, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret;

	dev->current_val = val;
	ret = lc898219_write_pos(client, val);
	if (ret)
		dev_err(&client->dev, "focus write failed: %d\n", ret);
	return ret;
}

static int lc898219_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898219_device *dev = ctrl_to_lc898219(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return lc898219_set_pos(dev, ctrl->val);
	return -EINVAL;
}

static const struct v4l2_ctrl_ops lc898219_ctrl_ops = {
	.s_ctrl = lc898219_set_ctrl,
};

static int lc898219_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_get_sync(sd->dev);
}

static int lc898219_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops lc898219_int_ops = {
	.open = lc898219_open,
	.close = lc898219_close,
};

static const struct v4l2_subdev_core_ops lc898219_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lc898219_ops = {
	.core = &lc898219_core_ops,
};

static void lc898219_subdev_cleanup(struct lc898219_device *dev)
{
	v4l2_async_unregister_subdev(&dev->sd);
	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrls_vcm);
}

static int lc898219_probe(struct i2c_client *client)
{
	struct lc898219_device *dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	v4l2_ctrl_handler_init(&dev->ctrls_vcm, 1);
	v4l2_ctrl_new_std(&dev->ctrls_vcm, &lc898219_ctrl_ops,
			  V4L2_CID_FOCUS_ABSOLUTE, 0,
			  LC898219_MAX_FOCUS_POS, LC898219_FOCUS_STEPS, 512);
	if (dev->ctrls_vcm.error) {
		ret = dev->ctrls_vcm.error;
		v4l2_ctrl_handler_free(&dev->ctrls_vcm);
		return ret;
	}

	dev->sd.ctrl_handler = &dev->ctrls_vcm;
	v4l2_i2c_subdev_init(&dev->sd, client, &lc898219_ops);
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->sd.internal_ops = &lc898219_int_ops;
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

	ret = media_entity_pads_init(&dev->sd.entity, 0, NULL);
	if (ret)
		goto err_reg;

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret)
		goto err_media;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_media:
	media_entity_cleanup(&dev->sd.entity);
err_reg:
	if (dev->vcc)
		regulator_disable(dev->vcc);
	v4l2_ctrl_handler_free(&dev->ctrls_vcm);
	return ret;
}

static void lc898219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898219_device *dev = sd_to_lc898219(sd);

	pm_runtime_disable(&client->dev);
	lc898219_subdev_cleanup(dev);
	if (dev->vcc)
		regulator_disable(dev->vcc);
}

static int __maybe_unused lc898219_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898219_device *lc = sd_to_lc898219(sd);
	u16 val;
	int ret;

	for (val = lc->current_val & ~(LC898219_CTRL_STEPS - 1); val > 0;
	     val -= LC898219_CTRL_STEPS) {
		ret = lc898219_write_pos(client, val);
		if (ret)
			break;
		usleep_range(LC898219_CTRL_DELAY_US,
			     LC898219_CTRL_DELAY_US + 10);
	}
	return lc898219_write_pos(client, 0);
}

static int __maybe_unused lc898219_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898219_device *lc = sd_to_lc898219(sd);
	u16 val;
	int ret;

	for (val = 0; val <= lc->current_val; val += LC898219_CTRL_STEPS) {
		ret = lc898219_write_pos(client, val);
		if (ret)
			return ret;
		usleep_range(LC898219_CTRL_DELAY_US,
			     LC898219_CTRL_DELAY_US + 10);
	}
	return lc898219_write_pos(client, lc->current_val);
}

static const struct of_device_id lc898219_of_table[] = {
	{ .compatible = "onsemi,lc898219" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lc898219_of_table);

static const struct dev_pm_ops lc898219_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lc898219_suspend, lc898219_resume)
	SET_RUNTIME_PM_OPS(lc898219_suspend, lc898219_resume, NULL)
};

static struct i2c_driver lc898219_i2c_driver = {
	.driver = {
		.name = LC898219_NAME,
		.pm = &lc898219_pm_ops,
		.of_match_table = lc898219_of_table,
	},
	.probe = lc898219_probe,
	.remove = lc898219_remove,
};

module_i2c_driver(lc898219_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898219 voice coil motor driver");
MODULE_LICENSE("GPL");
