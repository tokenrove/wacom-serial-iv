/*
 *
 */

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/slab.h>
#define DRIVER_AUTHOR	"Julian Squires <julian@cipht.net>"
#define DRIVER_DESC	"Wacom protocol 4 serial tablet driver"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define REQUEST_START_SENDING_PACKETS "ST"
#define REQUEST_STOP_SENDING_PACKETS "SP"
#define REQUEST_MODEL_AND_ROM_VERSION "~#"
#define REQUEST_MAX_COORDINATES "~C"
#define REQUEST_CONFIGURATION_STRING "~R"

#define PACKET_LENGTH 7

/* device IDs from wacom_wac.h */
#define STYLUS_DEVICE_ID        0x02
#define TOUCH_DEVICE_ID         0x03
#define CURSOR_DEVICE_ID        0x06
#define ERASER_DEVICE_ID        0x0A
#define PAD_DEVICE_ID           0x0F

struct wacom {
	struct input_dev *dev;
	int idx;
	char data[32];
	char phys[32];
};



static void handle_response(struct wacom *wacom)
{
	if (wacom->data[0] != '~' || wacom->idx < 2) {
		wacom->data[wacom->idx] = 0;
		dev_dbg(&wacom->dev->dev, "got a garbled response of length %d.\n", wacom->idx);
		return;
	}

	wacom->data[wacom->idx] = 0;
	switch (wacom->data[1]) {
	case '#':
		dev_info(&wacom->dev->dev, "model: Wacom tablet: %s\n", wacom->data);
		input_set_abs_params(wacom->dev, ABS_X, 0, 10240, 4, 0);
		input_set_abs_params(wacom->dev, ABS_Y, 0, 7680, 4, 0);
		input_set_abs_params(wacom->dev, ABS_PRESSURE, 0, 255, 0, 0);
		return;
	case 'R':
		dev_dbg(&wacom->dev->dev, "configuration: %s\n", wacom->data);
		return;
	case 'C':
		dev_dbg(&wacom->dev->dev, "coordinates: %s\n", wacom->data);
		return;
	default:
		dev_dbg(&wacom->dev->dev, "got a response we don't understand: %s\n", wacom->data);
		return;
	}
}

static void handle_packet(struct wacom *wacom)
{
	int in_proximity_p, stylus_p, button, x, y, z;
	int tool, device;

	in_proximity_p = wacom->data[0] & 0x40;
	stylus_p = wacom->data[0] & 0x20;
	button = (wacom->data[3] & 0x78) >> 3;
	x = (wacom->data[0] & 3) << 14 | wacom->data[1]<<7 | wacom->data[2];
	y = (wacom->data[3] & 3) << 14 | wacom->data[4]<<7 | wacom->data[5];
	z = (wacom->data[6] & 0x7f) << 1 | (wacom->data[3] & 0x4) >> 2;
	z = (z ^ 0x80) - 0x80;

	device = stylus_p ? STYLUS_DEVICE_ID : CURSOR_DEVICE_ID;
	/* XXX UNTESTED: Graphire eraser (according to old wcmSerial code) */
	if (button & 8)
		device = ERASER_DEVICE_ID;
	input_report_key(wacom->dev, ABS_MISC, device);
	input_report_abs(wacom->dev, ABS_X, x);
	input_report_abs(wacom->dev, ABS_Y, y);
	input_report_abs(wacom->dev, ABS_PRESSURE, z+127);
	input_report_key(wacom->dev, BTN_TOOL_MOUSE, in_proximity_p && !stylus_p);
	input_report_key(wacom->dev, BTN_TOOL_RUBBER, in_proximity_p && stylus_p && button&4);
	input_report_key(wacom->dev, BTN_TOOL_PEN, in_proximity_p && stylus_p && !(button&4));
	input_report_key(wacom->dev, BTN_TOUCH, button & 1);
	input_report_key(wacom->dev, BTN_STYLUS, button & 2);
	/* input_report_key(wacom->dev, BTN_STYLUS2, button & 2); */
	input_sync(wacom->dev);
}


static irqreturn_t wacom_interrupt(struct serio *serio, unsigned char data,
				   unsigned int flags)
{
	struct wacom *wacom = serio_get_drvdata(serio);

	if (data & 0x80)
		wacom->idx = 0;
	if (wacom->idx >= sizeof(wacom->data)) {
		dev_dbg(&wacom->dev->dev, "throwing away %d bytes of garbage\n", wacom->idx);
		wacom->idx = 0;
	}

	wacom->data[wacom->idx++] = data;

	/* we're either expecting a carriage return-terminated ASCII
	 * response string, or a seven-byte packet with the MSB set on
	 * the first byte */
	if (wacom->idx == PACKET_LENGTH && (wacom->data[0] & 0x80))
		handle_packet(wacom);
	else if (data == '\r' && !(wacom->data[0] & 0x80))
		handle_response(wacom);
	return IRQ_HANDLED;
}

static void wacom_disconnect(struct serio *serio)
{
	struct wacom *wacom = serio_get_drvdata(serio);

	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_unregister_device(wacom->dev);
	kfree(wacom);
}

static void wacom_send(struct serio *serio, const char *command)
{
	for (; *command; command++)
		serio_write(serio, *command);
	serio_write(serio, '\r');
}

static int wacom_connect(struct serio *serio, struct serio_driver *drv)
{
	struct wacom *wacom;
	struct input_dev *input_dev;
	int err = -ENOMEM;

	wacom = kzalloc(sizeof(struct wacom), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!wacom || !input_dev)
		goto fail0;

	wacom->dev = input_dev;
	snprintf(wacom->phys, sizeof(wacom->phys), "%s/input0", serio->phys);

	input_dev->name = "Wacom protocol IV serial tablet";
	input_dev->phys = wacom->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor  = SERIO_WACOM_IV;
	input_dev->id.product = serio->id.extra;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
	input_dev->keybit[BIT_WORD(BTN_TOOL_RUBBER)] |= BIT_MASK(BTN_TOOL_RUBBER);
	input_dev->keybit[BIT_WORD(BTN_TOOL_MOUSE)] |= BIT_MASK(BTN_TOOL_MOUSE);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	input_dev->keybit[BIT_WORD(BTN_STYLUS)] |= BIT_MASK(BTN_STYLUS);
	input_dev->keybit[BIT_WORD(BTN_STYLUS2)] |= BIT_MASK(BTN_STYLUS2);

	serio_set_drvdata(serio, wacom);

	err = serio_open(serio, drv);
	if (err)
		goto fail1;

	wacom_send(serio, REQUEST_STOP_SENDING_PACKETS);
	wacom_send(serio, REQUEST_MODEL_AND_ROM_VERSION);
	wacom_send(serio, REQUEST_START_SENDING_PACKETS);
	input_set_abs_params(wacom->dev, ABS_X, 0, 10240, 0, 0);
	input_set_abs_params(wacom->dev, ABS_Y, 0, 7680, 0, 0);
	input_set_abs_params(wacom->dev, ABS_PRESSURE, 0, 255, 0, 0);
	/* It seems that wcmUSB does something stupid with this, so let's not set it presently. */
	/* input_abs_set_res(wacom->dev, ABS_X, 1270);
	   input_abs_set_res(wacom->dev, ABS_Y, 1270); */

	err = input_register_device(wacom->dev);
	if (err)
		goto fail2;

	return 0;

 fail2:	serio_close(serio);
 fail1:	serio_set_drvdata(serio, NULL);
 fail0:	input_free_device(input_dev);
	kfree(wacom);
	return err;
}

static struct serio_device_id wacom_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_WACOM_IV,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, wacom_serio_ids);

static struct serio_driver wacom_drv = {
	.driver		= {
		.name	= "wacom_serial",
	},
	.description	= DRIVER_DESC,
	.id_table	= wacom_serio_ids,
	.interrupt	= wacom_interrupt,
	.connect	= wacom_connect,
	.disconnect	= wacom_disconnect,
};

static int __init wacom_init(void)
{
	return serio_register_driver(&wacom_drv);
}

static void __exit wacom_exit(void)
{
	serio_unregister_driver(&wacom_drv);
}

module_init(wacom_init);
module_exit(wacom_exit);
