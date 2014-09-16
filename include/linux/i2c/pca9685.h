/*
 * Definitions and platform data for NXP PCA9685 16 channel LED driver
 *
 * Copyright 2014 Gateworks Corporation
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_I2C_PCA9685_H
#define __LINUX_I2C_PCA9685_H

/* Flags */
#define PCA9685_FLAGS_IDMASK		0xf	/* id is lower 4 bits */
#define PCA9685_FLAGS_SHIFT		4
#define PCA9685_FLAGS_ACTIVE_LOW	(1<<0)	/* LED is active low */

struct pca9685_platform_data {
	bool open_drain;	/* open-drain or totem-pole */
	bool invert;
	bool extclk;
	int num_leds;
	struct led_info *leds;
};

#endif /* __LINUX_I2C_PCA9685_H */
