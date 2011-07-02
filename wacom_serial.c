/*
 *
 * Sections I have been unable to test personally due to lack of available
 * hardware are marked UNTESTED.
 */

#define DEBUG

#include <linux/string.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/slab.h>
#include <linux/completion.h>
#define DRIVER_AUTHOR	"Julian Squires <julian@cipht.net>"
#define DRIVER_DESC	"Wacom protocol 4 serial tablet driver"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define REQUEST_MODEL_AND_ROM_VERSION "~#"
#define REQUEST_MAX_COORDINATES "~C"
#define REQUEST_CONFIGURATION_STRING "~R"
#define REQUEST_RESET_TO_PROTOCOL_IV "\r#"

#define REQUEST_START_SENDING_PACKETS "ST"
#define REQUEST_STOP_SENDING_PACKETS "SP"

#define PACKET_LENGTH 7

/* device IDs from wacom_wac.h */
#define STYLUS_DEVICE_ID        0x02
#define TOUCH_DEVICE_ID         0x03
#define CURSOR_DEVICE_ID        0x06
#define ERASER_DEVICE_ID        0x0A
#define PAD_DEVICE_ID           0x0F

struct wacom {
	struct input_dev *dev;
	struct completion cmd_done;
	int extra_z_bits;
	int idx;
	unsigned char data[32];
	char phys[32];
};


enum {
	MODEL_CINTIQ		= 0x504C, /* PL */
	MODEL_CINTIQ2		= 0x4454, /* DT */
	MODEL_DIGITIZER_II	= 0x5544, /* UD */
	MODEL_GRAPHIRE		= 0x4554, /* ET */
	MODEL_INTUOS		= 0x4744, /* GD */
	MODEL_INTUOS2		= 0x5844, /* XD */
	MODEL_PENPARTNER	= 0x4354, /* CT */
	MODEL_UNKNOWN		= 0
};

static void handle_model_response(struct wacom *wacom)
{
	int major_v, minor_v, max_z;
	char *p;

	major_v = minor_v = 0;
	p = strrchr(wacom->data, 'V');
	if (p)
		sscanf(p+1, "%u.%u", &major_v, &minor_v);

	switch (wacom->data[2] << 8 | wacom->data[3]) {
	case MODEL_INTUOS:	/* UNTESTED */
	case MODEL_INTUOS2:
		dev_info(&wacom->dev->dev, "Intuos tablets are not supported by"
			 " this driver.\n");
		p = "Intuos";
		wacom->dev->id.version = MODEL_INTUOS;
		break;
	case MODEL_CINTIQ:	/* UNTESTED */
	case MODEL_CINTIQ2:
		p = "Cintiq";
		wacom->dev->id.version = MODEL_CINTIQ;
		switch (wacom->data[5]<<8 | wacom->data[6]) {
		case 0x3731: /* PL-710 */
			/* wcmSerial sets res to 2540x2540 in this case. */
			/* fall through */
		case 0x3535: /* PL-550 */
		case 0x3830: /* PL-800 */
			wacom->extra_z_bits = 2;
		}
		break;
	case MODEL_PENPARTNER:	/* UNTESTED */
		p = "Penpartner";
		wacom->dev->id.version = MODEL_PENPARTNER;
		/* wcmSerial sets res 1000x1000 in this case. */
		break;
	case MODEL_GRAPHIRE:	/* UNTESTED */
		p = "Graphire";
		wacom->dev->id.version = MODEL_GRAPHIRE;
		input_set_abs_params(wacom->dev, ABS_X, 0, 5103, 0, 0);
		input_set_abs_params(wacom->dev, ABS_Y, 0, 3711, 0, 0);
		/*
                 * input_abs_set_res(wacom->dev, ABS_X, 0, 1016);
		 * input_abs_set_res(wacom->dev, ABS_Y, 0, 1016);
                 */
		wacom->extra_z_bits = 2;
		break;
	case MODEL_DIGITIZER_II:
		p = "Digitizer II";
		wacom->dev->id.version = MODEL_DIGITIZER_II;
		if (major_v == 1 && minor_v <= 2)
			wacom->extra_z_bits = 0; /* UNTESTED */
		break;
	default:		/* UNTESTED */
		dev_dbg(&wacom->dev->dev, "Didn't understand Wacom model "
			                  "string: %s\n", wacom->data);
		p = "Unknown Protocol IV";
		wacom->dev->id.version = MODEL_UNKNOWN;
		break;
	}
	max_z = (1<<(7+wacom->extra_z_bits))-1;
	dev_info(&wacom->dev->dev, "Wacom tablet: %s, version %u.%u\n", p,
		 major_v, minor_v);
	dev_dbg(&wacom->dev->dev, "Max pressure: %d.\n", max_z);
	input_set_abs_params(wacom->dev, ABS_PRESSURE, 0, max_z, 0, 0);
}


static void handle_configuration_response(struct wacom *wacom)
{
	int x, y, skip;

	dev_dbg(&wacom->dev->dev, "Configuration string: %s\n", wacom->data);
	sscanf(wacom->data, "~R%x,%u,%u,%u,%u", &skip, &skip, &skip, &x, &y);
	input_abs_set_res(wacom->dev, ABS_X, x);
	input_abs_set_res(wacom->dev, ABS_Y, y);
}

static void handle_coordinates_response(struct wacom *wacom)
{
	int x, y;

	sscanf(wacom->data, "~C%u,%u", &x, &y);
	input_set_abs_params(wacom->dev, ABS_X, 0, x, 0, 0);
	input_set_abs_params(wacom->dev, ABS_Y, 0, y, 0, 0);
}

static void handle_response(struct wacom *wacom)
{
	if (wacom->data[0] != '~' || wacom->idx < 2) {
		dev_dbg(&wacom->dev->dev, "got a garbled response of length "
			                  "%d.\n", wacom->idx);
		return;
	}

	switch (wacom->data[1]) {
	case '#':
		handle_model_response(wacom);
		break;
	case 'R':
		handle_configuration_response(wacom);
		break;
	case 'C':
		handle_coordinates_response(wacom);
		break;
	default:
		dev_dbg(&wacom->dev->dev, "got an unexpected response: %s\n",
			wacom->data);
		break;
	}

	complete(&wacom->cmd_done);
}

static void handle_packet(struct wacom *wacom)
{
	int in_proximity_p, stylus_p, button, x, y, z;
	int device;

	in_proximity_p = wacom->data[0] & 0x40;
	stylus_p = wacom->data[0] & 0x20;
	button = (wacom->data[3] & 0x78) >> 3;
	x = (wacom->data[0] & 3) << 14 | wacom->data[1]<<7 | wacom->data[2];
	y = (wacom->data[3] & 3) << 14 | wacom->data[4]<<7 | wacom->data[5];
	z = wacom->data[6] & 0x7f;
	if(wacom->extra_z_bits >= 1)
		z = z << 1 | (wacom->data[3] & 0x4) >> 2;
	if(wacom->extra_z_bits > 1)
		z = z << 1 | (wacom->data[0] & 0x4);
	z = z ^ (0x40 << wacom->extra_z_bits);

	device = stylus_p ? STYLUS_DEVICE_ID : CURSOR_DEVICE_ID;
	/* UNTESTED Graphire eraser (according to old wcmSerial code) */
	if (button & 8)
		device = ERASER_DEVICE_ID;
	input_report_key(wacom->dev, ABS_MISC, device);
	input_report_abs(wacom->dev, ABS_X, x);
	input_report_abs(wacom->dev, ABS_Y, y);
	input_report_abs(wacom->dev, ABS_PRESSURE, z);
	input_report_key(wacom->dev, BTN_TOOL_MOUSE, in_proximity_p &&
			                             !stylus_p);
	input_report_key(wacom->dev, BTN_TOOL_RUBBER, in_proximity_p &&
			                              stylus_p && button&4);
	input_report_key(wacom->dev, BTN_TOOL_PEN, in_proximity_p && stylus_p &&
                                                   !(button&4));
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
		dev_dbg(&wacom->dev->dev, "throwing away %d bytes of garbage\n",
			wacom->idx);
		wacom->idx = 0;
	}

	wacom->data[wacom->idx++] = data;

	/* we're either expecting a carriage return-terminated ASCII
	 * response string, or a seven-byte packet with the MSB set on
	 * the first byte */
	if (wacom->idx == PACKET_LENGTH && (wacom->data[0] & 0x80)) {
		handle_packet(wacom);
		wacom->idx = 0;
	} else if (data == '\r' && !(wacom->data[0] & 0x80)) {
		wacom->data[wacom->idx] = 0;
		handle_response(wacom);
		wacom->idx = 0;
	}
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

static void send_setup_string(struct wacom *wacom, struct serio *serio)
{
	wacom_send(serio, REQUEST_START_SENDING_PACKETS);
}

static void wacom_setup(struct wacom *wacom, struct serio *serio)
{
	/* Note that setting the link speed is the job of inputattach.
	 * We assume that reset negotiation has already happened,
	 * here. */
	wacom_send(serio, REQUEST_STOP_SENDING_PACKETS);
	init_completion(&wacom->cmd_done);
	wacom_send(serio, REQUEST_MODEL_AND_ROM_VERSION);
	wait_for_completion_timeout(&wacom->cmd_done, HZ);

	/* It seems that wcmUSB does something stupid with this,
	   so let's not query it presently. */
	/* init_completion(&wacom->cmd_done);
	   wacom_send(serio, REQUEST_CONFIGURATION_STRING);
	   wait_for_completion_timeout(&wacom->cmd_done, HZ); */

	/* Note UNTESTED: Apparently Graphire models do not answer
	   coordinate requests. */
	if (wacom->dev->id.version != MODEL_GRAPHIRE) {
		init_completion(&wacom->cmd_done);
		wacom_send(serio, REQUEST_MAX_COORDINATES);
		wait_for_completion_timeout(&wacom->cmd_done, HZ);
	}

	send_setup_string(wacom, serio);
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
	wacom->extra_z_bits = 1;
	snprintf(wacom->phys, sizeof(wacom->phys), "%s/input0", serio->phys);

	input_dev->name = "Wacom protocol IV serial tablet";
	input_dev->phys = wacom->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor  = SERIO_WACOM_IV;
	input_dev->id.product = serio->id.extra;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(BTN_TOOL_PEN, input_dev->keybit);
	__set_bit(BTN_TOOL_RUBBER, input_dev->keybit);
	__set_bit(BTN_TOOL_MOUSE, input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_STYLUS, input_dev->keybit);
	__set_bit(BTN_STYLUS2, input_dev->keybit);

	serio_set_drvdata(serio, wacom);

	err = serio_open(serio, drv);
	if (err)
		goto fail1;

	wacom_setup(wacom, serio);

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
