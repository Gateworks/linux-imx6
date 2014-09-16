/*
 * LED driver for NXP PCA9685 16 channel PWM LED driver
 *
 * Copyright 2014 Gateworks Corporation <tharvey@gateworks.com>
 *
 * Licensed under the GPL-2 or later.
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/i2c/pca9685.h>

/* Registers */
#define PCA9685_MODE1	0x00
#define PCA9685_MODE2	0x01

#define PCA9685_LED_ON_L(n)	((n*4)+0x06)
#define PCA9685_LED_ON_H(n)	((n*4)+0x07)
#define PCA9685_LED_OFF_L(n)	((n*4)+0x08)
#define PCA9685_LED_OFF_H(n)	((n*4)+0x09)

#define MODE1_SLEEP		(1 << 4)
#define MODE1_EXTCLK		(1 << 6)
#define MODE2_INVERT		(1 << 4)
#define MODE2_TOTEM_POLE	(1 << 2)

struct pca9685 {
	struct i2c_client	*client;
	struct mutex lock;
	struct pca9685_platform_data *pdata;
	struct pca9685_led *led;
};

struct pca9685_led {
	struct led_classdev	cdev;
	struct work_struct	work;
	struct i2c_client	*client;
	enum led_brightness	new_brightness;
	int			id;
	int			flags;
};

static int pca9685_read(struct i2c_client *client, int reg, uint8_t *val)
{
        int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&client->dev, "<< 0x%02x=0x%02x\n", reg, *val);
	return 0;
}

static int pca9685_write(struct i2c_client *client, u8 reg, u8 val)
{
	dev_dbg(&client->dev, ">> 0x%02x=0x%02x\n", reg, val);
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int pca9685_set_bits(struct i2c_client *client, int reg, uint8_t bit_mask)
{
	struct pca9685 *data = i2c_get_clientdata(client);
	uint8_t reg_val;
	int ret;

	mutex_lock(&data->lock);

	ret = pca9685_read(client, reg, &reg_val);

	if (!ret && ((reg_val & bit_mask) == 0)) {
		reg_val |= bit_mask;
		ret = pca9685_write(client, reg, reg_val);
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int pca9685_clr_bits(struct i2c_client *client, int reg, uint8_t bit_mask)
{
	struct pca9685 *data = i2c_get_clientdata(client);
	uint8_t reg_val;
	int ret;

	mutex_lock(&data->lock);

	ret = pca9685_read(client, reg, &reg_val);

	if (!ret && (reg_val & bit_mask)) {
		reg_val &= ~bit_mask;
		ret = pca9685_write(client, reg, reg_val);
	}

	mutex_unlock(&data->lock);
	return ret;
}

static void pca9685_set_pwm(struct pca9685_led *led)
{
	struct i2c_client *client = led->client;
	u8 brightness = led->new_brightness;
	u16 duty;

	if (led->flags & PCA9685_FLAGS_ACTIVE_LOW)
		brightness = LED_FULL - brightness;
	if (brightness)
		duty = 4095 * brightness / LED_FULL;
	else
		duty = 0;
	duty &= 0xfff;

	dev_dbg(&client->dev, "%s led%d brightness=%d active-%s duty=0x%03x\n",
		__func__, led->id, led->new_brightness,
		led->flags & PCA9685_FLAGS_ACTIVE_LOW ? "low" : "high", duty);

	pca9685_write(client, PCA9685_LED_OFF_L(led->id), duty & 0xff);
	pca9685_write(client, PCA9685_LED_OFF_H(led->id), duty >> 8);
}

static void pca9685_led_work(struct work_struct *work)
{
	struct pca9685_led *led = container_of(work, struct pca9685_led, work);

	pca9685_set_pwm(led);
}

static void pca9685_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct pca9685_led *led;

	led = container_of(led_cdev, struct pca9685_led, cdev);
	led->new_brightness = value;
	/*
	 * Use workqueue for IO since I2C operations can sleep.
	 */
	schedule_work(&led->work);
}

static int pca9685_led_setup(struct pca9685_led *led)
{
	int ret = 0;

	/* set on count to zero (no delay) */
	pca9685_write(led->client, PCA9685_LED_ON_L(led->id), 0);
	pca9685_write(led->client, PCA9685_LED_ON_H(led->id), 0);

	/* set off count to zero (initially off) */
	pca9685_write(led->client, PCA9685_LED_OFF_L(led->id), 0);
	pca9685_write(led->client, PCA9685_LED_OFF_H(led->id), 0);

	return ret;
}

static int __devinit pca9685_led_probe(struct i2c_client *client)
{
	struct pca9685_platform_data *pdata = client->dev.platform_data;
	struct pca9685 *data = i2c_get_clientdata(client);
	struct pca9685_led *led, *led_dat;
	struct led_info *cur_led;
	int ret, i;

	led = kcalloc(pdata->num_leds, sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&client->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < pdata->num_leds; ++i) {
		cur_led = &pdata->leds[i];
		led_dat = &led[i];

		led_dat->id = cur_led->flags & PCA9685_FLAGS_IDMASK;

		if (led_dat->id > 15) {
			dev_err(&client->dev, "Invalid LED ID %d\n",
					led_dat->id);
			goto err;
		}

		led_dat->cdev.name = cur_led->name;
		led_dat->cdev.default_trigger = cur_led->default_trigger;
		led_dat->cdev.brightness_set = pca9685_led_set;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->flags = cur_led->flags >> PCA9685_FLAGS_SHIFT;
		led_dat->client = client;
		led_dat->new_brightness = LED_OFF;
		INIT_WORK(&led_dat->work, pca9685_led_work);

		ret = led_classdev_register(&client->dev, &led_dat->cdev);
		if (ret) {
			dev_err(&client->dev, "failed to register LED %d\n",
					led_dat->id);
			goto err;
		}

		ret = pca9685_led_setup(led_dat);
		if (ret) {
			dev_err(&client->dev, "failed to write\n");
			i++;
			goto err;
		}
		pca9685_set_pwm(led_dat);
	}

	data->led = led;

	return 0;

err:
	for (i = i - 1; i >= 0; --i) {
		led_classdev_unregister(&led[i].cdev);
		cancel_work_sync(&led[i].work);
	}

	return ret;
}

static int __devexit pca9685_led_remove(struct i2c_client *client)
{
	struct pca9685_platform_data *pdata =
		client->dev.platform_data;
	struct pca9685 *data = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&data->led[i].cdev);
		cancel_work_sync(&data->led[i].work);
	}

	kfree(data->led);
	return 0;
}

static int __devinit pca9685_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca9685 *data;
	struct pca9685_platform_data *pdata =
		client->dev.platform_data;
	int ret;
	u8 reg;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
	dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (!pdata->num_leds) {
		dev_err(&client->dev, "no leds?\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->client = client;
	data->pdata = pdata;
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	reg = 0;
	if (pdata->extclk)
		reg |= MODE1_EXTCLK;
	pca9685_write(client, PCA9685_MODE1, reg);

	reg = 0;
	if (pdata->invert)
		reg |= MODE2_INVERT;
	if (!pdata->open_drain)
		reg |= MODE2_TOTEM_POLE;
	pca9685_write(client, PCA9685_MODE2, reg);

	dev_info(&client->dev, "%s registered\n", client->name);

	ret = pca9685_led_probe(client);
	if (ret)
		goto out;
	return 0;

out:
	kfree(data);
	return ret;
}

static int __devexit pca9685_remove(struct i2c_client *client)
{
	struct pca9685 *data = i2c_get_clientdata(client);

	pca9685_set_bits(client, PCA9685_MODE1, MODE1_SLEEP);

	if (data->led)
		pca9685_led_remove(client);

	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int pca9685_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	pca9685_set_bits(client, PCA9685_MODE1, MODE1_SLEEP);

	return 0;
}

static int pca9685_i2c_resume(struct i2c_client *client)
{
	pca9685_clr_bits(client, PCA9685_MODE1, MODE1_SLEEP);

	return 0;
}
#else
#define pca9685_i2c_suspend NULL
#define pca9685_i2c_resume NULL
#endif

static const struct i2c_device_id pca9685_id[] = {
	{ "pca9685", 0x40 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9685_id);

static struct i2c_driver pca9685_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = pca9685_probe,
	.remove = __devexit_p(pca9685_remove),
	.suspend = pca9685_i2c_suspend,
	.resume = pca9685_i2c_resume,
	.id_table = pca9685_id,
};

static int __init pca9685_init(void)
{
	return i2c_add_driver(&pca9685_driver);
}
module_init(pca9685_init);

static void __exit pca9685_exit(void)
{
	i2c_del_driver(&pca9685_driver);
}
module_exit(pca9685_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_DESCRIPTION("PCA9685 LED driver");
MODULE_ALIAS("i2c:pca9685-leds");
