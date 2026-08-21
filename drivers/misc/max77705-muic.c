// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal Maxim MAX77705 MUIC bootstrap for the Samsung Galaxy Note 10+
 * (d2s/SM-N975F) mainline bring-up.
 *
 * The MAX77705 MUIC sits on the USI_CMGP03 pins (gpm12/gpm13) and
 * routes the USB D+/D- lines between the AP PHY, UART and "OPEN" states.
 * Samsung's bootloader leaves the switch at COM_OPEN (0x3f), so without a
 * MUIC driver a mainline kernel can never enumerate as a USB peripheral.
 *
 * This driver only forces the switch to COM_USB (0x09) using the vendor
 * "opcode" register protocol (verified against the live chip in TWRP):
 *   write: block-write reg 0x21 = [opcode, data...], then reg 0x41 = 0x00
 *   read : 2-byte read from reg 0x51 = [opcode echo, value]
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#define MAX77705_OPCODE_WRITE		0x21
#define MAX77705_OPCODE_WRITE_END	0x41
#define MAX77705_OPCODE_READ		0x51
#define MAX77705_OPCODE_CTRL1_R		0x05
#define MAX77705_OPCODE_CTRL1_W		0x06
#define MAX77705_COM_USB		0x09
#define MAX77705_COM_OPEN		0x3f

struct max77705_muic_data {
	struct i2c_client *client;
	struct delayed_work ctrl1_work;
	int ctrl1_checks;
};

static int max77705_muic_read_ctrl1(struct i2c_client *client, u8 *val)
{
	u8 wbuf = MAX77705_OPCODE_READ;
	u8 rbuf[2] = { 0, 0 };
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 1, .buf = &wbuf },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 2,
		  .buf = rbuf },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_dbg(&client->dev, "d2s-muic: ctrl1 read failed (%d)\n", ret);
		return ret < 0 ? ret : -EIO;
	}
	*val = rbuf[1];
	return 0;
}

static int max77705_muic_write_ctrl1(struct i2c_client *client, u8 val)
{
	u8 wbuf[3] = { MAX77705_OPCODE_WRITE, MAX77705_OPCODE_CTRL1_W, val };
	u8 ebuf[2] = { MAX77705_OPCODE_WRITE_END, 0x00 };
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 3, .buf = wbuf },
		{ .addr = client->addr, .flags = 0, .len = 2, .buf = ebuf },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_dbg(&client->dev, "d2s-muic: ctrl1 write failed (%d)\n", ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static void max77705_muic_ctrl1_check(struct work_struct *work)
{
	struct max77705_muic_data *data =
		container_of(work, struct max77705_muic_data, ctrl1_work.work);
	u8 val = 0;
	int ret;

	data->ctrl1_checks++;
	ret = max77705_muic_read_ctrl1(data->client, &val);
	if (ret) {
		dev_warn(&data->client->dev,
			 "d2s-muic: ctrl1 check %d failed: %d\n",
			 data->ctrl1_checks, ret);
	} else if (val != MAX77705_COM_USB) {
		dev_warn(&data->client->dev,
			 "d2s-muic: ctrl1 changed to %02x, restoring USB route\n",
			 val);
		ret = max77705_muic_write_ctrl1(data->client,
						MAX77705_COM_USB);
		if (ret)
			dev_err(&data->client->dev,
				"d2s-muic: failed to restore USB route: %d\n",
				ret);
	}

	if (data->ctrl1_checks < 2)
		schedule_delayed_work(&data->ctrl1_work, 50 * HZ);
}

static int max77705_muic_probe(struct i2c_client *client)
{
	struct max77705_muic_data *data;
	u8 val = 0;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	INIT_DELAYED_WORK(&data->ctrl1_work, max77705_muic_ctrl1_check);

	ret = max77705_muic_read_ctrl1(client, &val);
	dev_dbg(&client->dev, "d2s-muic: ctrl1_before=%02x%s%s\n", val,
		ret ? " rd-err" : "",
		!ret && val == MAX77705_COM_OPEN ? " (open)" : "");

	/* Force the D+/D- switch to the AP USB PHY */
	ret = max77705_muic_write_ctrl1(client, MAX77705_COM_USB);
	if (ret)
		return ret;

	ret = max77705_muic_read_ctrl1(client, &val);
	dev_dbg(&client->dev, "d2s-muic: ctrl1_after=%02x%s%s\n", val,
		ret ? " rd-err" : "",
		!ret && val == MAX77705_COM_USB ? " (usb)" : "");
	if (ret)
		return ret;
	if (val != MAX77705_COM_USB)
		return dev_err_probe(&client->dev, -EIO,
				     "failed to select USB route (ctrl1=%02x)\n",
				     val);

	/* Re-verify the switch stays in USB mode (10s, 60s) */
	data->ctrl1_checks = 0;
	schedule_delayed_work(&data->ctrl1_work, 10 * HZ);

	return 0;
}

static void max77705_muic_remove(struct i2c_client *client)
{
	struct max77705_muic_data *data = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&data->ctrl1_work);
}

static const struct of_device_id max77705_muic_of_match[] = {
	{ .compatible = "maxim,max77705-muic" },
	{ },
};
MODULE_DEVICE_TABLE(of, max77705_muic_of_match);

static struct i2c_driver max77705_muic_driver = {
	.probe = max77705_muic_probe,
	.remove = max77705_muic_remove,
	.driver = {
		.name = "max77705-muic",
		.of_match_table = max77705_muic_of_match,
	},
};
module_i2c_driver(max77705_muic_driver);

MODULE_AUTHOR("d2s mainline bring-up");
MODULE_DESCRIPTION("Minimal MAX77705 MUIC bootstrap (d2s)");
MODULE_LICENSE("GPL");
