// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung sec_ts (Y771) touchscreen driver - mainline minimal port
 *
 * Based on the vendor driver from Samsung SM-N975F (Exynos 9825) OSS:
 *   drivers/input/touchscreen/sec_ts/y771/sec_ts.c
 * Reduced to the core probe/event path: no firmware update, no Samsung
 * sysfs/ioctl interface, no secure-TUI, no input-booster.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/err.h>

#define SEC_TS_CMD_SENSE_ON			0x10
#define SEC_TS_CMD_SENSE_OFF			0x11
#define SEC_TS_CMD_SW_RESET			0x12
#define SEC_TS_READ_FIRMWARE_INTEGRITY		0x21
#define SEC_TS_READ_DEVICE_ID			0x22
#define SEC_TS_READ_PANEL_INFO			0x23
#define SEC_TS_CMD_SET_TOUCHFUNCTION		0x30
#define SEC_TS_READ_ID				0x52
#define SEC_TS_READ_BOOT_STATUS			0x55
#define SEC_TS_READ_ONE_EVENT			0x60
#define SEC_TS_READ_ALL_EVENT			0x61
#define SEC_TS_CMD_CLEAR_EVENT_STACK		0x62
#define SEC_TS_READ_TS_STATUS			0xAF

#define SEC_TS_STATUS_BOOT_MODE			0x10
#define SEC_TS_STATUS_APP_MODE			0x20

#define SEC_TS_EVENT_BUFF_SIZE			8
#define SEC_TS_MAX_EVENT_COUNT			31
#define SEC_TS_MAX_TOUCH_COUNT			10

#define SEC_TS_EVENT_COORDINATE			0
#define SEC_TS_EVENT_STATUS			1
#define SEC_TS_EVENT_GESTURE			2
#define SEC_TS_EVENT_EMPTY			3
#define SEC_TS_ACK_BOOT_COMPLETE		0x00
#define TYPE_STATUS_EVENT_INFO			2

#define SEC_TS_COORDINATE_ACTION_NONE		0
#define SEC_TS_COORDINATE_ACTION_PRESS		1
#define SEC_TS_COORDINATE_ACTION_MOVE		2
#define SEC_TS_COORDINATE_ACTION_RELEASE	3

#define SEC_TS_TOUCHTYPE_NORMAL			0
#define SEC_TS_TOUCHTYPE_GLOVE			3
#define SEC_TS_TOUCHTYPE_PALM			5
#define SEC_TS_TOUCHTYPE_WET			6

#define SEC_TS_BIT_SETFUNC_TOUCH		BIT(0)
#define SEC_TS_BIT_SETFUNC_PALM			BIT(5)
#define SEC_TS_BIT_SETFUNC_WET			BIT(6)
#define SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC	(SEC_TS_BIT_SETFUNC_TOUCH | \
						 SEC_TS_BIT_SETFUNC_PALM | \
						 SEC_TS_BIT_SETFUNC_WET)

#define SEC_TS_I2C_RETRY_CNT			3

struct sec_ts_event_coordinate {
	u8 eid:2;
	u8 tid:4;
	u8 tchsta:2;
	u8 x_11_4;
	u8 y_11_4;
	u8 y_3_0:4;
	u8 x_3_0:4;
	u8 major;
	u8 minor;
	u8 z:6;
	u8 ttype_3_2:2;
	u8 left_event:5;
	u8 max_energy_flag:1;
	u8 ttype_1_0:2;
} __packed;

struct sec_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *dvdd;
	struct regulator *avdd;
	struct gpio_desc *reset_gpio;
	u16 max_x;
	u16 max_y;
	u8 touch_count;
};

static int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	struct i2c_msg msg;
	u8 *buf;
	int retry;
	int ret;

	buf = kmalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg;
	if (len > 0 && data)
		memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1)
			break;
		usleep_range(1000, 1100);
	}

	kfree(buf);

	if (retry == SEC_TS_I2C_RETRY_CNT)
		return -EIO;

	return 0;
}

static int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	struct i2c_msg msg[2];
	u8 cmd;
	int retry;
	int ret;

	cmd = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret == 2)
			break;
		usleep_range(1000, 1100);
	}

	if (retry == SEC_TS_I2C_RETRY_CNT)
		return -EIO;

	return 0;
}

static void sec_ts_release_all_fingers(struct sec_ts_data *ts)
{
	int i;

	for (i = 0; i < SEC_TS_MAX_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);
	ts->touch_count = 0;
}

static void sec_ts_process_coordinate(struct sec_ts_data *ts, u8 *event)
{
	struct sec_ts_event_coordinate *coord =
		(struct sec_ts_event_coordinate *)event;
	u8 t_id = coord->tid - 1;
	u8 action = coord->tchsta;
	u8 ttype = (coord->ttype_3_2 << 2) | coord->ttype_1_0;
	u16 x = (coord->x_11_4 << 4) | coord->x_3_0;
	u16 y = (coord->y_11_4 << 4) | coord->y_3_0;

	if (coord->tid == 0) {
		/* tid 0 is reserved; not a valid finger slot */
		dev_dbg(&ts->client->dev, "sec_ts: invalid tid 0\n");
		return;
	}

	if (t_id >= SEC_TS_MAX_TOUCH_COUNT) {
		dev_dbg(&ts->client->dev, "sec_ts: tid %d out of range\n", t_id);
		return;
	}

	if (ttype != SEC_TS_TOUCHTYPE_NORMAL && ttype != SEC_TS_TOUCHTYPE_GLOVE &&
	    ttype != SEC_TS_TOUCHTYPE_PALM && ttype != SEC_TS_TOUCHTYPE_WET)
		return;

	input_mt_slot(ts->input, t_id);

	switch (action) {
	case SEC_TS_COORDINATE_ACTION_RELEASE:
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
		if (ts->touch_count > 0)
			ts->touch_count--;
		if (ts->touch_count == 0) {
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_report_key(ts->input, BTN_TOOL_FINGER, 0);
		}
		break;
	case SEC_TS_COORDINATE_ACTION_PRESS:
		ts->touch_count++;
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 1);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, coord->major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, coord->minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, coord->z & 0x3f);
		break;
	case SEC_TS_COORDINATE_ACTION_MOVE:
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 1);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, coord->major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, coord->minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, coord->z & 0x3f);
		break;
	default:
		return;
	}
}

static void sec_ts_process_event(struct sec_ts_data *ts, u8 *event)
{
	u8 event_id = event[0] & 0x3;
	u8 stype = (event[0] >> 2) & 0xf;
	int ret;

	if (event_id == SEC_TS_EVENT_COORDINATE) {
		sec_ts_process_coordinate(ts, event);
	} else if (event_id == SEC_TS_EVENT_STATUS) {
		/*
		 * Boot-complete after an IC reset (e.g. watchdog reset while
		 * suspended): release any stale fingers, re-apply the touch
		 * function mask and re-enable sensing, mirroring the vendor
		 * handler.
		 */
		if (stype == TYPE_STATUS_EVENT_INFO &&
		    event[1] == SEC_TS_ACK_BOOT_COMPLETE &&
		    event[2] == SEC_TS_STATUS_APP_MODE) {
			u8 setfunc[2] = {
				SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC & 0xff,
				(SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC >> 8) & 0xff,
			};

			sec_ts_release_all_fingers(ts);
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
					       setfunc, sizeof(setfunc));
			if (ret < 0)
				dev_warn(&ts->client->dev,
					 "sec_ts: failed to restore touch function: %d\n",
					 ret);
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
			if (ret < 0)
				dev_warn(&ts->client->dev,
					 "sec_ts: failed to restore sensing: %d\n",
					 ret);
		} else {
			dev_dbg(&ts->client->dev,
				"sec_ts: status event %02X %02X %02X %02X\n",
				event[0], event[1], event[2], event[3]);
		}
	} else {
		dev_dbg(&ts->client->dev, "sec_ts: event %02X %02X %02X %02X %02X %02X %02X %02X\n",
			event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);
	}
}

static irqreturn_t sec_ts_irq_thread(int irq, void *dev_id)
{
	struct sec_ts_data *ts = dev_id;
	u8 event_buff[SEC_TS_MAX_EVENT_COUNT][SEC_TS_EVENT_BUFF_SIZE];
	u8 left_event_count;
	int ret;
	int i;

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, event_buff[0],
			      SEC_TS_EVENT_BUFF_SIZE);
	if (ret < 0) {
		dev_err(&ts->client->dev, "sec_ts: i2c read one event failed\n");
		return IRQ_HANDLED;
	}

	if (event_buff[0][0] == 0)
		return IRQ_HANDLED;

	left_event_count = event_buff[0][7] & 0x1f;

	if (left_event_count > SEC_TS_MAX_EVENT_COUNT - 1) {
		dev_err(&ts->client->dev, "sec_ts: event buffer overflow\n");
		sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		sec_ts_release_all_fingers(ts);
		return IRQ_HANDLED;
	}

	if (left_event_count > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ALL_EVENT,
				      &event_buff[1][0],
				      SEC_TS_EVENT_BUFF_SIZE * left_event_count);
		if (ret < 0) {
			dev_err(&ts->client->dev, "sec_ts: i2c read all event failed\n");
			return IRQ_HANDLED;
		}
	}

	for (i = 0; i <= left_event_count; i++)
		sec_ts_process_event(ts, event_buff[i]);

	input_sync(ts->input);

	return IRQ_HANDLED;
}

static int sec_ts_power_on(struct sec_ts_data *ts)
{
	int ret;

	if (ts->avdd) {
		ret = regulator_enable(ts->avdd);
		if (ret)
			return ret;
	}
	if (ts->dvdd) {
		ret = regulator_enable(ts->dvdd);
		if (ret) {
			if (ts->avdd)
				regulator_disable(ts->avdd);
			return ret;
		}
	}
	if (ts->reset_gpio) {
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
		usleep_range(10000, 11000);
		gpiod_set_value_cansleep(ts->reset_gpio, 1);
		usleep_range(10000, 11000);
	}
	/*
	 * Without a reset GPIO the IC is left as the bootloader put it
	 * (rails are regulator-boot-on in the vendor DT); the probe's
	 * retry + boot-complete wait handles a still-booting IC.
	 */

	return 0;
}

static void sec_ts_power_off(struct sec_ts_data *ts)
{
	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
	if (ts->dvdd)
		regulator_disable(ts->dvdd);
	if (ts->avdd)
		regulator_disable(ts->avdd);
}

static int sec_ts_wait_for_boot_complete(struct sec_ts_data *ts)
{
	u8 event[SEC_TS_EVENT_BUFF_SIZE];
	int attempts = 50;
	int ret;

	while (attempts-- > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, event,
				      SEC_TS_EVENT_BUFF_SIZE);
		if (ret == 0 && event[0] != 0) {
			/* stype == INFO, status_id == BOOT_COMPLETE, app mode */
			if ((event[0] & 0x3) == SEC_TS_EVENT_STATUS &&
			    ((event[0] >> 2) & 0xf) == TYPE_STATUS_EVENT_INFO &&
			    event[1] == SEC_TS_ACK_BOOT_COMPLETE &&
			    event[2] == SEC_TS_STATUS_APP_MODE)
				return 0;
		}
		msleep(100);
	}

	return -ETIMEDOUT;
}

static int sec_ts_probe(struct i2c_client *client)
{
	struct sec_ts_data *ts;
	struct device *dev = &client->dev;
	u8 data[13] = { 0 };
	u32 max_coords[2] = { 4096, 4096 };
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	device_property_read_u32(dev, "touchscreen-size-x", &max_coords[0]);
	device_property_read_u32(dev, "touchscreen-size-y", &max_coords[1]);
	if (max_coords[0] > 1)
		ts->max_x = max_coords[0] - 1;
	else
		ts->max_x = 4095;
	if (max_coords[1] > 1)
		ts->max_y = max_coords[1] - 1;
	else
		ts->max_y = 4095;

	ts->dvdd = devm_regulator_get_optional(dev, "dvdd");
	if (IS_ERR(ts->dvdd)) {
		if (PTR_ERR(ts->dvdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->dvdd),
					     "failed to get dvdd\n");
		ts->dvdd = NULL;
	}

	ts->avdd = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		if (PTR_ERR(ts->avdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->avdd),
					     "failed to get avdd\n");
		ts->avdd = NULL;
	}

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	ret = sec_ts_power_on(ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	/* Try to read device ID; the IC may still be booting. */
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, data, 5);
	if (ret < 0) {
		ret = sec_ts_wait_for_boot_complete(ts);
		if (ret < 0) {
			dev_err(dev, "sec_ts: device did not become ready\n");
			goto err_power;
		}
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, data, 5);
		if (ret < 0) {
			dev_err(dev, "sec_ts: failed to read device ID\n");
			goto err_power;
		}
	}
	dev_info(dev, "sec_ts: device ID: %02X %02X %02X %02X %02X\n",
		 data[0], data[1], data[2], data[3], data[4]);

	memset(data, 0, sizeof(data));
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret == 0 && data[0] == SEC_TS_STATUS_BOOT_MODE) {
		dev_info(dev, "sec_ts: IC in boot mode, waiting for app mode\n");
		ret = sec_ts_wait_for_boot_complete(ts);
		if (ret < 0) {
			dev_err(dev, "sec_ts: IC stuck in boot mode\n");
			goto err_power;
		}
	} else if (ret == 0 && data[0] != SEC_TS_STATUS_APP_MODE) {
		dev_warn(dev, "sec_ts: unexpected boot status 0x%02x\n", data[0]);
	}

	/* Panel info: bytes 0-1 = max x, 2-3 = max y from the IC. */
	memset(data, 0, sizeof(data));
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_PANEL_INFO, data, 11);
	if (ret == 0) {
		if (((data[0] << 8) | data[1]) > 0)
			ts->max_x = ((data[0] << 8) | data[1]) - 1;
		if (((data[2] << 8) | data[3]) > 0)
			ts->max_y = ((data[2] << 8) | data[3]) - 1;
		dev_info(dev, "sec_ts: panel %d x %d, tx %d rx %d\n",
			 ts->max_x, ts->max_y, data[8], data[9]);
	} else {
		dev_warn(dev, "sec_ts: failed to read panel info, using DT max\n");
	}

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input) {
		ret = -ENOMEM;
		goto err_power;
	}

	ts->input->name = "sec_touchscreen";
	ts->input->id.bustype = BUS_I2C;
	ts->input->dev.parent = dev;

	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->input, EV_KEY, BTN_TOOL_FINGER);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	ret = input_mt_init_slots(ts->input, SEC_TS_MAX_TOUCH_COUNT,
				  INPUT_MT_DIRECT);
	if (ret) {
		dev_err(dev, "sec_ts: failed to init MT slots\n");
		goto err_power;
	}

	input_set_drvdata(ts->input, ts);

	ret = input_register_device(ts->input);
	if (ret) {
		dev_err(dev, "sec_ts: failed to register input device\n");
		goto err_power;
	}

	/* Enable touch + palm + wet handling, then start sensing. */
	data[0] = SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC & 0xff;
	data[1] = (SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC >> 8) & 0xff;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, data, 2);
	if (ret < 0)
		dev_warn(dev, "sec_ts: failed to set touch function\n");

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		dev_warn(dev, "sec_ts: failed to write sense on\n");
		goto err_input;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					sec_ts_irq_thread,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"sec_ts", ts);
	if (ret) {
		dev_err(dev, "sec_ts: failed to request irq %d\n", client->irq);
		goto err_input;
	}

	dev_info(dev, "sec_ts: probed (%d x %d), irq %d\n",
		 ts->max_x, ts->max_y, client->irq);

	return 0;

err_input:
	input_unregister_device(ts->input);
err_power:
	sec_ts_power_off(ts);
	return ret;
}

static void sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	disable_irq(client->irq);
	input_unregister_device(ts->input);
	sec_ts_power_off(ts);
}

static int sec_ts_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	disable_irq(ts->client->irq);
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		enable_irq(ts->client->irq);
		return ret;
	}
	sec_ts_release_all_fingers(ts);

	return 0;
}

static int sec_ts_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	/*
	 * If the IC reset itself while the system was suspended it will send
	 * a boot-complete status event, which re-arms sensing; otherwise
	 * simply resume sensing from where suspend left it.
	 */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		return ret;
	enable_irq(ts->client->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sec_ts_pm_ops, sec_ts_suspend, sec_ts_resume);

static const struct of_device_id sec_ts_of_match[] = {
	{ .compatible = "sec,sec_ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_ts_of_match);

static const struct i2c_device_id sec_ts_id[] = {
	{ "sec_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_ts_id);

static struct i2c_driver sec_ts_driver = {
	.driver = {
		.name = "sec_ts",
		.of_match_table = sec_ts_of_match,
		.pm = pm_sleep_ptr(&sec_ts_pm_ops),
	},
	.probe = sec_ts_probe,
	.remove = sec_ts_remove,
	.id_table = sec_ts_id,
};
module_i2c_driver(sec_ts_driver);

MODULE_AUTHOR("d2s bring-up");
MODULE_DESCRIPTION("Samsung sec_ts touchscreen driver (mainline port)");
MODULE_LICENSE("GPL");
