/*
 * Copyright (C) 2013 Gateworks Corporation All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/pfuze.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include "crm_regs.h"
#include "regs-anadig.h"
#include "cpu_op-mx6.h"

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#define PFUZE100_I2C_DEVICE_NAME  "pfuze100"
/* 7-bit I2C bus slave address */
#define PFUZE100_I2C_ADDR		(0x08)
#define PFUZE100_DEVICEID		(0x0)
#define PFUZE100_REVID			(0x3)
#define PFUZE100_SW1AMODE		(0x23)
#define PFUZE100_SW1AVOL       32
#define PFUZE100_SW1AVOL_VSEL_M        (0x3f<<0)
#define PFUZE100_SW1CVOL       46
#define PFUZE100_SW1CVOL_VSEL_M        (0x3f<<0)
#define PFUZE100_SW1ACON		36
#define PFUZE100_SW1ACON_SPEED_VAL	(0x1<<6)	/*default */
#define PFUZE100_SW1ACON_SPEED_M	(0x3<<6)
#define PFUZE100_SW1CCON		49
#define PFUZE100_SW1CCON_SPEED_VAL	(0x1<<6)	/*default */
#define PFUZE100_SW1CCON_SPEED_M	(0x3<<6)
#define PFUZE100_SWBSTCTL		102
#define PFUZE100_SWBSTCTL_MODE_M  (0x3<<2)

extern u32 arm_max_freq;
extern u32 enable_ldo_mode;

static struct regulator_consumer_supply sw1_consumers[] = {
	{
		.supply	   = "VDDCORE", // VDD_ARM on schem
	}
};
static struct regulator_consumer_supply sw1c_consumers[] = {
	{
		.supply	   = "VDDSOC",
	},
};

static struct regulator_consumer_supply sw2_consumers[] = {
	{
		.supply	   = "VDD_HIGH",  // 3.0V 
	}
};
static struct regulator_consumer_supply sw4_consumers[] = {
	/* Analog Video Decoder ADV7180 */
	{
		.supply	   = "VDDA",
		.dev_name   = "2-0020",
	},
	{
		.supply	   = "PVDD",
		.dev_name   = "2-0020",
	},
	/* Digital Video Decoder (HDMI Receiver) ADV7611 */
	{
		.supply	   = "DVDD",
		.dev_name   = "2-004c",
	},
	{
		.supply	   = "PVDD",
		.dev_name   = "2-004c",
	},
	{
		.supply	   = "CVDD",
		.dev_name   = "2-004c",
	},
	/* Analog Video Encoder ADV7393 */
	{
		.supply	   = "VDD",
		.dev_name   = "2-002a",
	},
	{
		.supply	   = "VAA",
		.dev_name   = "2-002a",
	},
};
static struct regulator_consumer_supply swbst_consumers[] = {
	/* Fan controller, HDMI In/Out Term, MIPI Exp, LVDS Exp */
	{
		.supply = "VDD_5P0_SWBST",
	}
};
static struct regulator_consumer_supply vgen1_consumers[] = {
	/* PCIE (top sockets) */
	{
		.supply = "VDD_PEXA_1P5_VGEN1",
	}
};
static struct regulator_consumer_supply vgen2_consumers[] = {
	/* PCIE (bot sockets) */
	{
		.supply = "VDD_PEXB_1P5_VGEN2",
	}
};
static struct regulator_consumer_supply vgen4_consumers[] = {
	/* VDD_AUD_1P8 - 1.8V LDO used for Audio Codec */
	{
		.supply	   = "VDDA",
		.dev_name   = "2-000a",
	},
};
static struct regulator_consumer_supply vgen5_consumers[] = {
	/* VDD_2P5 - 2.5V used for CPU ENET (NVCC_ENET/PHY_VDD) and PCIe Switch */
	{
		.supply = "VDD_2P5_VGEN5",
	}
};
static struct regulator_consumer_supply vgen6_consumers[] = {
	/* VDD_2P8 - Delayed 3P3 FET enable */
	{
		.supply = "VDD_2P8_VGEN6",
	}
};

/* VCore (ganged with sw1b) */
static struct regulator_init_data sw1a_init = {
	.constraints = {
			.name = "PFUZE100_SW1A",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			.initial_state = PM_SUSPEND_MEM,
			.state_mem = {
				.uV = 975000,/*0.9V+6%*/
				.mode = REGULATOR_MODE_NORMAL,
				.enabled = 1,
			},
	},

	.num_consumer_supplies = ARRAY_SIZE(sw1_consumers),
	.consumer_supplies = sw1_consumers,
};

/* Vcore (ganged with sw1a) */
static struct regulator_init_data sw1b_init = {
	.constraints = {
			.name = "PFUZE100_SW1B",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

/* VSoC */
static struct regulator_init_data sw1c_init = {
	.constraints = {
			.name = "PFUZE100_SW1C",
			.min_uV = 300000,
			.max_uV = 1875000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			.initial_state = PM_SUSPEND_MEM,
			.state_mem = {
				.uV = 975000,/*0.9V+6%*/
				.mode = REGULATOR_MODE_NORMAL,
				.enabled = 1,
			},
	},
	.num_consumer_supplies = ARRAY_SIZE(sw1c_consumers),
	.consumer_supplies = sw1c_consumers,
};

static struct regulator_init_data sw2_init = {
	.constraints = {
			.name = "PFUZE100_SW2",
#if PFUZE100_SW2_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(sw2_consumers),
	.consumer_supplies = sw2_consumers,
};

/* SW3 - DDR */
static struct regulator_init_data sw3a_init = {
	.constraints = {
			.name = "PFUZE100_SW3A",
#if PFUZE100_SW3_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

/* SW3 - DDR */
static struct regulator_init_data sw3b_init = {
	.constraints = {
			.name = "PFUZE100_SW3B",
#if PFUZE100_SW3_VOL6
			.min_uV = 800000,
			.max_uV = 3950000,
#else
			.min_uV = 400000,
			.max_uV = 1975000,
#endif
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

/* 1.8V HDMI Receiver, Video Dec, Video Enc */
static struct regulator_init_data sw4_init = {
	.constraints = {
			.name = "PFUZE100_SW4",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(sw4_consumers),
	.consumer_supplies = sw4_consumers,
};

/* 5V Boost - MIPI, LVDS, HDMI rec, HDMI trans, Fan */
static struct regulator_init_data swbst_init = {
	.constraints = {
			.name = "PFUZE100_SWBST",
			.min_uV = 5000000,
//			.max_uV = 5150000,
			.max_uV = 5000000,
			.apply_uV = 1,
			.always_on = 1,
//			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			//.initial_state = 0, // dont need to setup initial suspend state
			// need to enable supply from suspended state
			.initial_state = PM_SUSPEND_MEM,
			.state_mem = {
				.uV = 5000000,
				.enabled = 1,
			},
			},
	.num_consumer_supplies = ARRAY_SIZE(swbst_consumers),
	.consumer_supplies = swbst_consumers,
};

/* unused */
static struct regulator_init_data vsnvs_init = {
	.constraints = {
			.name = "PFUZE100_VSNVS",
			.min_uV = 1200000,
			.max_uV = 3000000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data vrefddr_init = {
	.constraints = {
			.name = "PFUZE100_VREFDDR",
			.always_on = 1,
			.boot_on = 1,
			},
};

/* PCI 1.5V (top sockets) */
static struct regulator_init_data vgen1_init = {
	.constraints = {
			.name = "PFUZE100_VGEN1",
			.min_uV = 1500000,
			.max_uV = 1500000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen1_consumers),
	.consumer_supplies = vgen1_consumers,
};

/* PCI 1.5V (bot sockets) */
static struct regulator_init_data vgen2_init = {
	.constraints = {
			.name = "PFUZE100_VGEN2",
			.min_uV = 1500000,
			.max_uV = 1500000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen2_consumers),
	.consumer_supplies = vgen2_consumers,

};

/* currently unused 1.8V - 3.05V 100mA LDO */
static struct regulator_init_data vgen3_init = {
	.constraints = {
			.name = "PFUZE100_VGEN3",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			},
};

/* Audio Codec */
static struct regulator_init_data vgen4_init = {
	.constraints = {
			.name = "PFUZE100_VGEN4",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen4_consumers),
	.consumer_supplies = vgen4_consumers,
};

/* 2.5V PCIe */
static struct regulator_init_data vgen5_init = {
	.constraints = {
			.name = "PFUZE100_VGEN5",
			.min_uV = 2500000,
			.max_uV = 2500000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen5_consumers),
	.consumer_supplies = vgen5_consumers,
};

/* used for delayed 3.3 FET enable */
static struct regulator_init_data vgen6_init = {
	.constraints = {
			.name = "PFUZE100_VGEN6",
			.min_uV = 2800000,
			.max_uV = 2800000,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.initial_state = 0, // dont need to setup initial suspend state
			},
	.num_consumer_supplies = ARRAY_SIZE(vgen6_consumers),
	.consumer_supplies = vgen6_consumers,
};

static int pfuze100_init(struct mc_pfuze *pfuze)
{
	int ret, i;
	unsigned char value;

	printk(KERN_INFO "%s Internal LDO:%s\n", __func__,
		(enable_ldo_mode == LDO_MODE_DEFAULT)?"default/bypassed":"");
	/*use default mode(ldo bypass) if no param from cmdline*/
	if (enable_ldo_mode == LDO_MODE_DEFAULT)
		enable_ldo_mode = LDO_MODE_BYPASSED;
	/*read Device ID*/
	ret = pfuze_reg_read(pfuze, PFUZE100_DEVICEID, &value);
	if (ret)
		goto err;
	switch (value & 0xf) {
		case 0x0:
		/* Freescale misprogramed 1-3% of parts prior to week 8 of 2013 this ID */
		case 0x8:
			break;
		default:
			printk(KERN_ERR "wrong device id:%x!\n", value);
			goto err;
	}

	/*read Revision ID*/
	ret = pfuze_reg_read(pfuze, PFUZE100_REVID, &value);
	if (ret)
		goto err;
	/*set all switches APS in normal and PFM mode in standby*/
	for (i = 0; i < 7; i++) {
		value = 0xc;
		ret = pfuze_reg_write(pfuze,
				PFUZE100_SW1AMODE + (i * 7),
				value);
		if (ret)
			goto err;
	}
	/*use ldo active mode if use 1.2GHz,otherwise use ldo bypass mode*/
	if (arm_max_freq == CPU_AT_1_2GHz) {
		printk(KERN_INFO "%s: 1.2GHz CPU - using internal LDO\n", __func__);
		/*VDDARM_IN 1.425*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x2d);
		if (ret)
			goto err;
		/*VDDSOC_IN 1.425V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x2d);
		if (ret)
			goto err;
		enable_ldo_mode = LDO_MODE_ENABLED;
	} else if (enable_ldo_mode == LDO_MODE_BYPASSED) {
		printk(KERN_INFO "%s: 1GHz or less CPU - bypassing internal LDO\n", __func__);
		/*decrease VDDARM_IN/VDDSOC_IN,since we will use ldo bypass mode*/
		/*VDDARM_IN 1.3V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x28);
		if (ret)
			goto err;
		/*VDDSOC_IN 1.3V*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x28);
		if (ret)
			goto err;
		/*set SW1AB/1C DVSPEED as 25mV step each 4us,quick than 16us before.*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1ACON,
				    PFUZE100_SW1ACON_SPEED_M,
				    PFUZE100_SW1ACON_SPEED_VAL);
		if (ret)
			goto err;
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CCON,
				    PFUZE100_SW1CCON_SPEED_M,
				    PFUZE100_SW1CCON_SPEED_VAL);
		if (ret)
			goto err;
	} else if (enable_ldo_mode != LDO_MODE_BYPASSED) {
		printk(KERN_INFO "%s internal LDO active mode\n", __func__);
		/*Increase VDDARM_IN/VDDSOC_IN to 1.375V in ldo active mode*/
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1AVOL,
					PFUZE100_SW1AVOL_VSEL_M,
					0x2b);
		if (ret)
			goto err;
		ret = pfuze_reg_rmw(pfuze, PFUZE100_SW1CVOL,
					PFUZE100_SW1CVOL_VSEL_M,
					0x2b);
		if (ret)
			goto err;
	}

	/* enable SWBST */
	ret = pfuze_reg_rmw(pfuze, PFUZE100_SWBSTCTL,
		PFUZE100_SWBSTCTL_MODE_M, 0x2<<2);

	return 0;
err:
	printk(KERN_ERR "pfuze100 init error!\n");
	return -1;
}

static struct pfuze_regulator_init_data mx6q_ventana_pfuze100_regulators[] = {
	{.id = PFUZE100_SW1A,	.init_data = &sw1a_init},
	{.id = PFUZE100_SW1B,	.init_data = &sw1b_init},
	{.id = PFUZE100_SW1C,	.init_data = &sw1c_init},
	{.id = PFUZE100_SW2,	.init_data = &sw2_init},
	{.id = PFUZE100_SW3A,	.init_data = &sw3a_init},
	{.id = PFUZE100_SW3B,	.init_data = &sw3b_init},
	{.id = PFUZE100_SW4,	.init_data = &sw4_init},
	{.id = PFUZE100_SWBST,	.init_data = &swbst_init},
	{.id = PFUZE100_VSNVS,	.init_data = &vsnvs_init},
	{.id = PFUZE100_VREFDDR,	.init_data = &vrefddr_init},
	{.id = PFUZE100_VGEN1,	.init_data = &vgen1_init},
	{.id = PFUZE100_VGEN2,	.init_data = &vgen2_init},
	{.id = PFUZE100_VGEN3,	.init_data = &vgen3_init},
	{.id = PFUZE100_VGEN4,	.init_data = &vgen4_init},
	{.id = PFUZE100_VGEN5,	.init_data = &vgen5_init},
	{.id = PFUZE100_VGEN6,	.init_data = &vgen6_init},
};

static struct pfuze_platform_data pfuze100_plat = {
	.flags = PFUZE_USE_REGULATOR,
	.num_regulators = ARRAY_SIZE(mx6q_ventana_pfuze100_regulators),
	.regulators = mx6q_ventana_pfuze100_regulators,
	.pfuze_init = pfuze100_init,
};

static struct i2c_board_info __initdata pfuze100_i2c_device = {
	I2C_BOARD_INFO(PFUZE100_I2C_DEVICE_NAME, PFUZE100_I2C_ADDR),
	.platform_data = &pfuze100_plat,
};

int __init mx6q_ventana_init_pfuze100(u32 int_gpio)
{
	pfuze100_i2c_device.irq = gpio_to_irq(int_gpio);
	return i2c_register_board_info(1, &pfuze100_i2c_device, 1);
}
