/*
 * Copyright (C) 2013 Gateworks Corporation All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/at24.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/mfd/tda1997x-core.h>
#include <linux/pci.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "ventana_eeprom.h"

/* GPIO pins */
#define MX6Q_VENTANA_CAN_STBY       IMX_GPIO_NR(1, 2)
#define MX6Q_VENTANA_HDMIIN_IRQ     IMX_GPIO_NR(1, 7)
#define MX6Q_VENTANA_ECSPI1_CS1     IMX_GPIO_NR(3, 19)
#define MX6Q_VENTANA_USB_OTG_PWR    IMX_GPIO_NR(3, 22)
#define MX6Q_VENTANA_SD3_CD         IMX_GPIO_NR(7, 0)
#define MX6Q_VENTANA_CAP_TCH_INT    IMX_GPIO_NR(7, 12)
#define MX6Q_VENTANA_ACCEL_IRQ      IMX_GPIO_NR(7, 13)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern char *gp_reg_id;
extern char *soc_reg_id;
static int caam_enabled;

#define MX6Q_VENTANA_SD1_CMD_PADCFG (PAD_CTL_DSE_240ohm)
#define MX6Q_VENTANA_DISP0_DIO_PADCFG (PAD_CTL_DSE_240ohm)
#define MX6Q_VENTANA_USBOTGPEN_PADCFG (PAD_CTL_DSE_40ohm|PAD_CTL_PUE|PAD_CTL_PUS_100K_UP)

/* AUDMUX4 */
static iomux_v3_cfg_t mx6q_ventana_audmux4_pads[] = {
	MX6Q_PAD_SD2_DAT0__AUDMUX_AUD4_RXD,
	MX6Q_PAD_SD2_DAT3__AUDMUX_AUD4_TXC,
	MX6Q_PAD_SD2_DAT2__AUDMUX_AUD4_TXD,
	MX6Q_PAD_SD2_DAT1__AUDMUX_AUD4_TXFS,
};
static iomux_v3_cfg_t mx6dl_ventana_audmux4_pads[] = {
	MX6DL_PAD_SD2_DAT0__AUDMUX_AUD4_RXD,
	MX6DL_PAD_SD2_DAT3__AUDMUX_AUD4_TXC,
	MX6DL_PAD_SD2_DAT2__AUDMUX_AUD4_TXD,
	MX6DL_PAD_SD2_DAT1__AUDMUX_AUD4_TXFS,
};

/* AUDMUX5 */
static iomux_v3_cfg_t mx6q_ventana_audmux5_pads[] = {
	MX6Q_PAD_EIM_D25__AUDMUX_AUD5_RXC,
	MX6Q_PAD_DISP0_DAT19__AUDMUX_AUD5_RXD,
	MX6Q_PAD_EIM_D24__AUDMUX_AUD5_RXFS,
};
static iomux_v3_cfg_t mx6dl_ventana_audmux5_pads[] = {
	MX6DL_PAD_EIM_D25__AUDMUX_AUD5_RXC,
	MX6DL_PAD_DISP0_DAT19__AUDMUX_AUD5_RXD,
	MX6DL_PAD_EIM_D24__AUDMUX_AUD5_RXFS,
};

/* ECSPI1 */
static iomux_v3_cfg_t mx6q_ventana_spi_pads[] = {
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	 /* CS1 */
};
static iomux_v3_cfg_t mx6dl_ventana_spi_pads[] = {
	MX6DL_PAD_EIM_D17__ECSPI1_MISO,
	MX6DL_PAD_EIM_D18__ECSPI1_MOSI,
	MX6DL_PAD_EIM_D16__ECSPI1_SCLK,
	MX6DL_PAD_EIM_D19__GPIO_3_19,	 /* CS1 */
};

/* NAND */
static iomux_v3_cfg_t mx6q_ventana_nand_pads[] = {
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,

	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
  MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
  MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
};
static iomux_v3_cfg_t mx6dl_ventana_nand_pads[] = {
	MX6DL_PAD_NANDF_D0__RAWNAND_D0,
	MX6DL_PAD_NANDF_D1__RAWNAND_D1,
	MX6DL_PAD_NANDF_D2__RAWNAND_D2,
	MX6DL_PAD_NANDF_D3__RAWNAND_D3,
	MX6DL_PAD_NANDF_D4__RAWNAND_D4,
	MX6DL_PAD_NANDF_D5__RAWNAND_D5,
	MX6DL_PAD_NANDF_D6__RAWNAND_D6,
	MX6DL_PAD_NANDF_D7__RAWNAND_D7,

	MX6DL_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6DL_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6DL_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6DL_PAD_NANDF_WP_B__RAWNAND_RESETN,
	MX6DL_PAD_NANDF_RB0__RAWNAND_READY0,
  MX6DL_PAD_SD4_CMD__RAWNAND_RDN,
  MX6DL_PAD_SD4_CLK__RAWNAND_WRN,
};

/* CANbus */
static iomux_v3_cfg_t mx6q_ventana_flexcan_pads[] = {
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_GPIO_2__GPIO_1_2,
};
static iomux_v3_cfg_t mx6dl_ventana_flexcan_pads[] = {
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6DL_PAD_GPIO_2__GPIO_1_2,
};

/* UART */
static iomux_v3_cfg_t mx6q_gw5400a_uart_pads[] = {
	/* UART1: RS485  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2: Console */
	MX6Q_PAD_SD4_DAT7__UART2_TXD,
	MX6Q_PAD_SD4_DAT4__UART2_RXD,

	/* UART3: GPS */
	MX6Q_PAD_SD4_CMD__UART3_TXD,
	MX6Q_PAD_SD4_CLK__UART3_RXD,
};

static iomux_v3_cfg_t mx6q_gw5400b_uart_pads[] = {
	/* UART1: RS485  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2: Console */
	MX6Q_PAD_SD4_DAT7__UART2_TXD,
	MX6Q_PAD_SD4_DAT4__UART2_RXD,

	/* UART5: GPS */
	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,
};

static iomux_v3_cfg_t mx6dl_gw51xx_uart_pads[] = {
	/* UART1: GPS */
	MX6DL_PAD_SD3_DAT7__UART1_TXD,
	MX6DL_PAD_SD3_DAT6__UART1_RXD,

	/* UART2: Console */
	MX6DL_PAD_SD4_DAT7__UART2_TXD,
	MX6DL_PAD_SD4_DAT4__UART2_RXD,

	/* UART3: App header */
	MX6DL_PAD_EIM_D24__UART3_TXD,  // J11.1
	MX6DL_PAD_EIM_D25__UART3_RXD,  // J11.2

	/* UART5: App header */
	MX6DL_PAD_KEY_COL1__UART5_TXD, // J11.3
	MX6DL_PAD_KEY_ROW1__UART5_RXD, // J11.4
};


/* IPU2_DISP0 */
static iomux_v3_cfg_t mx6q_disp0_pads[] = {
	/* Analog Video Out (NB: can be muxed to either IPU1_DISP0 or IPU2_DISP0) */
	MX6Q_PAD_DI0_DISP_CLK__IPU2_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN2__IPU2_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU2_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DISP0_DAT0__IPU2_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU2_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU2_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU2_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU2_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU2_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU2_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU2_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU2_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU2_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU2_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU2_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU2_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU2_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU2_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU2_DISP0_DAT_15,
};

/* general shared pinmux
 * - anything board-specific is registered dynamically
 * - pinmux done by bootloader is not repeated (ENET, USDHC, some GPIOs)
 */
static iomux_v3_cfg_t mx6q_ventana_pads[] = {
	/* CCM */
	MX6Q_PAD_GPIO_0__CCM_CLKO,     /* SGTL500 sys_mckl */

	/* I2C3 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,
	MX6Q_PAD_GPIO_6__I2C3_SDA,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* USBOTG PWR EN */
	NEW_PAD_CTRL(MX6Q_PAD_EIM_D22__GPIO_3_22, MX6Q_VENTANA_USBOTGPEN_PADCFG),

	/* USBOTG OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
};
static iomux_v3_cfg_t mx6dl_ventana_pads[] = {
	/* I2C3 */
	MX6DL_PAD_GPIO_3__I2C3_SCL,
	MX6DL_PAD_GPIO_6__I2C3_SDA,

	/* USBOTG ID pin */
	MX6DL_PAD_GPIO_1__USBOTG_ID,

	/* USBOTG PWR EN */
	NEW_PAD_CTRL(MX6DL_PAD_EIM_D22__GPIO_3_22, MX6Q_VENTANA_USBOTGPEN_PADCFG),

	/* USBOTG OC pin */
	MX6DL_PAD_KEY_COL4__USBOH3_USBOTG_OC,
};

/* IPU1_CSI0 - Digital Video in */
static iomux_v3_cfg_t mx6q_ventana_csi0_sensor_pads[] = {
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20,       /* DATA_EN not used */
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
};
static iomux_v3_cfg_t mx6dl_ventana_csi0_sensor_pads[] = {
	MX6DL_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6DL_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6DL_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6DL_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6DL_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6DL_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6DL_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6DL_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6DL_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6DL_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6DL_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
};

/* IPU2_CSI1 */
static iomux_v3_cfg_t mx6q_ventana_csi1_sensor_pads[] = {
	MX6Q_PAD_EIM_EB2__IPU2_CSI1_D_19,
	MX6Q_PAD_EIM_D16__IPU2_CSI1_D_18,
	MX6Q_PAD_EIM_D18__IPU2_CSI1_D_17,
	MX6Q_PAD_EIM_D19__IPU2_CSI1_D_16,
	MX6Q_PAD_EIM_D20__IPU2_CSI1_D_15,
	MX6Q_PAD_EIM_D26__IPU2_CSI1_D_14,
	MX6Q_PAD_EIM_D27__IPU2_CSI1_D_13,
	MX6Q_PAD_EIM_A17__IPU2_CSI1_D_12,
	MX6Q_PAD_EIM_D23__GPIO_3_23,        /* DATA_EN not used */
	MX6Q_PAD_EIM_D29__IPU2_CSI1_VSYNC,
	MX6Q_PAD_EIM_EB3__IPU2_CSI1_HSYNC,
	MX6Q_PAD_EIM_A16__IPU2_CSI1_PIXCLK,
};

static iomux_v3_cfg_t mx6q_ventana_hdmi_ddc_pads[] = {
  MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL,
  MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA,
};

static iomux_v3_cfg_t mx6q_ventana_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,
};

static iomux_v3_cfg_t mx6dl_ventana_hdmi_ddc_pads[] = {
  MX6DL_PAD_KEY_COL3__HDMI_TX_DDC_SCL,
  MX6DL_PAD_KEY_ROW3__HDMI_TX_DDC_SDA,
};

static iomux_v3_cfg_t mx6dl_ventana_i2c2_pads[] = {
	MX6DL_PAD_KEY_COL3__I2C2_SCL,
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

/* plt_sd_pad_change
 *  reconfig SDHC pads per SD bus frequency (called from esdhc_set_clock)
 */
static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 2:
		sd_pads_200mhz = mx6q_sd3_200mhz;
		sd_pads_100mhz = mx6q_sd3_100mhz;
		sd_pads_50mhz = mx6q_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6q_ventana_sd3_data __initconst = {
	.cd_gpio = MX6Q_VENTANA_SD3_CD,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd_pad_change,
	.delay_line = 0,
};


static int __init gpmi_nand_platform_init(void)
{
	if (cpu_is_mx6q()) {
		return mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_nand_pads,
			ARRAY_SIZE(mx6q_ventana_nand_pads));
	} else if (cpu_is_mx6dl()) {
		return mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_nand_pads,
			ARRAY_SIZE(mx6dl_ventana_nand_pads));
	}
	return -EINVAL;
}

static struct mtd_partition imx6_ventana_nand_partitions[] = {
	{
	 .name = "uboot",
	 .offset = 0,
	 .size = SZ_1M * 16,
	},
	{
	 .name = "env",
	 .offset = MTDPART_OFS_APPEND,
	 .size = SZ_1M,
	},
	{
	 .name = "rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,

	/* Medium information. */
	.partitions = imx6_ventana_nand_partitions,
	.partition_count = ARRAY_SIZE(imx6_ventana_nand_partitions),
	.enable_bbt = 0, // do not store BBT in FLASH
	.enable_ddr = 0, // Enable PoP DDR
};

static const struct anatop_thermal_platform_data
	mx6q_ventana_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static int mx6q_ventana_fec_phy_init(struct phy_device *phydev)
{
	// NB: 88E1510 PHY configured in bootloader
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_ventana_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_ventana_spi_cs[] = {
	MX6Q_VENTANA_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_ventana_spi_data __initconst = {
	.chipselect     = mx6q_ventana_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_ventana_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_ventana_spi_nor_partitions[] = {
	{
	 .name = "uboot",
	 .offset = 0,
	 .size = SZ_512K,
	},
	{
	 .name = "env",
	 .offset = MTDPART_OFS_APPEND,
	 .size = SZ_8K,
	},
	{
	 .name = "linux",
	 .offset = MTDPART_OFS_APPEND,
	 .size = SZ_2M,
	},
	{
	 .name = "rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_ventana__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_ventana_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_ventana_spi_nor_partitions),
	.type = "w25q256",
};
#endif

static struct spi_board_info imx6_ventana_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_ventana__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_ventana_spi_nor_device,
				ARRAY_SIZE(imx6_ventana_spi_nor_device));
}

/* Analog Audio Codec */
static struct mxc_audio_platform_data mx6_ventana_analog_audio_data;

static int mx6_ventana_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_ventana_analog_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_ventana_ssi1_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_ventana_analog_audio_data = {
	/* codec-->aud4-->ssi2 */
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_ventana_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_ventana_analog_audio_device = {
	.name = "imx-sgtl5000",
};

/* Digital Audio In */
static struct imx_ssi_platform_data mx6_ventana_ssi2_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_ventana_digital_audio_data = {
	/* codec-->aud5-->ssi1 */
	.src_port = 1,
	.ext_port = 5,
	.hp_gpio = -1,
};

static struct platform_device mx6_ventana_digital_audio_device = {
	.name = "imx-tda1997x",
};

#ifdef CONFIG_PPS_CLIENT_GPIO
/* GPS PPS */
#include <linux/pps-gpio.h>
static struct pps_gpio_platform_data mx6_ventana_pps_data = {
	.gpio_pin = -1,
	.gpio_label = "GPS_PPS",
	.assert_falling_edge = 0,
	.capture_clear = 0,
};

static struct platform_device mx6_ventana_pps_device = {
	.name = "pps-gpio",
	.id = -1,
	.dev.platform_data = &mx6_ventana_pps_data,
};
#endif

/* GPIO LEDs */
static struct gpio_led mx6_ventana_gpio_leds[] = {
	{
		.name = "user1",
		.default_trigger = "heartbeat",
		.gpio = -1,
	},
	{
		.name = "user2",
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.gpio = -1,
	},
	{
		.name = "user3",
		.active_low = 1,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.gpio = -1,
	},
};

static struct gpio_led_platform_data mx6_ventana_led_pdata = {
	.leds = mx6_ventana_gpio_leds,
	.num_leds = ARRAY_SIZE(mx6_ventana_gpio_leds),
};

static struct platform_device mx6_ventana_leds_gpio_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &mx6_ventana_led_pdata,
	},
};

/* PWM LEDs */
static struct led_pwm mx6_ventana_pwm_leds[] = {
	{
		.name = "dio0",
		.pwm_id = 0,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
	{
		.name = "dio1",
		.pwm_id = 1,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
	{
		.name = "dio2",
		.pwm_id = 1,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
	{
		.name = "dio3",
		.pwm_id = 1,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
};

static struct led_pwm_platform_data mx6_ventana_pwm_pdata = {
	.num_leds = ARRAY_SIZE(mx6_ventana_pwm_leds),
	.leds = mx6_ventana_pwm_leds,
};

static struct platform_device mx6_ventana_leds_pwm_device = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &mx6_ventana_pwm_pdata,
	},
};

/* I2C */
static struct imxi2c_platform_data mx6q_ventana_i2c_data = {
	.bitrate = 100000,
};

static struct ventana_board_info ventana_board_info;

static void ventana_eeprom_setup(struct memory_accessor *mem_acc, void *context)
{
	int i, rz;
	int chksum;
	struct ventana_board_info *info = &ventana_board_info;
	char *buf = (char *) &ventana_board_info;

	rz = mem_acc->read(mem_acc, buf, 0x100, sizeof(ventana_board_info));
	if (rz != sizeof(ventana_board_info)) {
		printk(KERN_ERR "failed to read Ventana EEPROM: %d\n", rz);
		return;
	}

	/* validate checksum */
	for (chksum = 0, i = 0; i < sizeof(*info)-2; i++)  chksum += buf[i];
	if ( info->chksum[0] != (chksum>>8)
	  || info->chksum[1] != (chksum&0xff))
	{
		printk(KERN_ERR "Ventana EEPROM data corrupt\n");
		info->chksum[0] = 0;
		info->chksum[1] = 0;
		return;
	}
};

void ventana_getmacaddr(char *addr, int dev)
{
	char *buf;

	if (dev < 4) {
		buf = (char *) (ventana_board_info.mac0 + (ETH_ALEN * dev));
		memcpy(addr, buf, ETH_ALEN);
	}
}
EXPORT_SYMBOL(ventana_getmacaddr);

static struct at24_platform_data ventana_eeprom_info = {
  .byte_len = 1024,
  .page_size = 16,
  .flags = AT24_FLAG_READONLY,
  .setup = ventana_eeprom_setup,
};

/* Analog Video Out - IPU2_DISP0 */
static struct fsl_mxc_lcd_platform_data adv7393_pdata = {
	.ipu_id = 1,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_BT656,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("gsp", 0x29),
	},{
		I2C_BOARD_INFO ("24c08",0x50),
		.platform_data = &ventana_eeprom_info,
	},{
		I2C_BOARD_INFO("ds1672", 0x68),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
};


/* position 0-7 (alignment?) */
static int mma8451_position = 1;

void mx6_csi0_io_init(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_ventana_csi0_sensor_pads));
		/* set IPU1 Mux to parallel interface
		 * IOMUX_GPR1 bit19 and bit20 meaning:
		 * Bit19:       0 - Enable mipi to IPU1 CSI0
		 *                      virtual channel is fixed to 0
		 *              1 - Enable parallel interface to IPU1 CSI0
		 * Bit20:       0 - Enable mipi to IPU2 CSI1
		 *                      virtual channel is fixed to 3
		 *              1 - Enable parallel interface to IPU2 CSI1
		 * IPU1 CSI1 directly connect to mipi csi2,
		 *      virtual channel is fixed to 1
		 * IPU2 CSI0 directly connect to mipi csi2,
		 *      virtual channel is fixed to 2
		 *
		 */
		mxc_iomux_set_gpr_register(1, 19, 1, 1); /* GPR1[19] = 1 */
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_ventana_csi0_sensor_pads));
		/* IPU_CSI0_MUX (GRP13[0:2]):
		 *   0 - MIPI_CSI0
		 *   1 - MIPI_CSI1
		 *   2 - MIPI_CSI2
		 *   3 - MIPI_CSI3
		 *   4 - IPU_CSI0
		 */
		mxc_iomux_set_gpr_register(13, 0, 3, 4); /* GPR13[0:2] = 4 */
	}
}
EXPORT_SYMBOL(mx6_csi0_io_init);

static void mx6_csi1_io_init(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_csi1_sensor_pads,
			ARRAY_SIZE(mx6q_ventana_csi1_sensor_pads));
		/* set IPU1 Mux to parallel interface
		 * IOMUX_GPR1 bit19 and bit20 meaning:
		 * Bit19:       0 - Enable mipi to IPU1 CSI0
		 *                      virtual channel is fixed to 0
		 *              1 - Enable parallel interface to IPU1 CSI0
		 * Bit20:       0 - Enable mipi to IPU2 CSI1
		 *                      virtual channel is fixed to 3
		 *              1 - Enable parallel interface to IPU2 CSI1
		 * IPU1 CSI1 directly connect to mipi csi2,
		 *      virtual channel is fixed to 1
		 * IPU2 CSI0 directly connect to mipi csi2,
		 *      virtual channel is fixed to 2
		 *
		 */
		mxc_iomux_set_gpr_register(1, 20, 1, 1); /* GPR1[20] = 1 */
	} else if (cpu_is_mx6dl()) {
#if 0
		mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_csi1_sensor_pads,
			ARRAY_SIZE(mx6dl_ventana_csi1_sensor_pads));
		/* IPU_CSI1_MUX (GRP13[3:5]):
		 *   0 - MIPI_CSI0
		 *   1 - MIPI_CSI1
		 *   2 - MIPI_CSI2
		 *   3 - MIPI_CSI3
		 *   4 - IPU_CSI1
		 */
		mxc_iomux_set_gpr_register(13, 3, 3, 4); /* GPR13[3:5] = 4 */
#else
		printk(KERN_ERR "%s: not supported for IMX6SDL\n", __func__);
#endif
	}
}

static void adv7180_pdwn(int powerdown) {
	pr_info("%s: powerdown=%d gpio=%d\n", __func__, powerdown,
		IMX_GPIO_NR(5,20));
	gpio_set_value(IMX_GPIO_NR(5,20), powerdown ? 0 : 1);
}

/* Analog Video In - IPU2_CSI1 */
static struct fsl_mxc_tvin_platform_data adv7180_pdata = {
	.dvddio_reg = "VDD_DLY_3P3",
	.dvdd_reg = "DVDD", // VDD_1P8
	.avdd_reg = "AVDD", // VDD_1P8
	.pvdd_reg = "PVDD", // VDD_1P8
	.pwdn = adv7180_pdwn,
	.cvbs = true,
	.io_init = mx6_csi1_io_init,
	.csi = 1,
};

/* Digital Video In - IPU1_CSI0 */
static struct fsl_mxc_tvin_platform_data mxc_tda1997x_video_pdata = {
	.csi = 0,
	.io_init = mx6_csi0_io_init,
};
static struct tda1997x_platform_data tda1997x_pdata = {
	/* regulators */
	.dvddio_reg = "VDD_DLY_3P3",
	.dvdd_reg = "DVDD", /* VDD_1P8 */
	.avdd_reg = "AVDD", /* VDD_1P8 */

	/* Misc */
	.hdcp = 1, /* enable HDCP */
	.ddc_slave = 0x74, /* slave address of DDC channel */

	/* Video Output */
	.vidout_format = VIDEOFMT_422_CCIR, /* BT656 */
	.vidout_trc = 1, /* insert timing codes (SAV/EAV) in stream */
	.vidout_blc = 1, /* isnert blanking codes in stream */
	.vidout_clkmode = CLOCK_SINGLE_EDGE, /* 1x clock */
	/* Port Config: bits[0:12] hooked up to csi_data[8:16] */
	.vidout_port_config = {
		0x00, /* VP35_32_CTRL */
		0x00, /* VP31_28_CTRL */
		0x00, /* VP27_24_CTRL */
		0x82, /* VP23_23_CTRL */
		0x81, /* VP19_16_CTRL */
		0x00, /* VP15_12_CTRL */
		0x00, /* VP11_08_CTRL */
		0x00, /* VP07_04_CTRL */
		0x00, /* VP03_00_CTRL */
	},
	.vidout_port_config_no = 9,

	/* Audio output */
	.audout_format = AUDIO_FMT_I2S16,       /* I2S bus, 16bit per channel */
	.audout_sysclk = AUDIO_SYSCLK_128FS,    /* 128fs clkmode */
	.audout_layout = AUDIO_LAYOUT_FORCED_0, /* AP0,WS,A_CLK for up to 2ch audio */
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	/* Audio Codec */
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},

	/* Accelerometer */
	{
		I2C_BOARD_INFO("mma8451", 0x1c),
		.irq = gpio_to_irq(MX6Q_VENTANA_ACCEL_IRQ), // not used by driver
		.platform_data = (void *)&mma8451_position,
	},

	/* HDMI Video Decoder (Digital Video In) */
	{
		I2C_BOARD_INFO("tda1997x", 0x48),
		.irq = gpio_to_irq(MX6Q_VENTANA_HDMIIN_IRQ),
		.platform_data = (void *)&tda1997x_pdata,
	},

	/* Analog Video Decoder (Analog Video In) */
	{
		I2C_BOARD_INFO("adv7180", 0x20),
		.platform_data = (void *)&adv7180_pdata,
	},

	/* Analog Video Encoder (Video Out) */
	{
		I2C_BOARD_INFO("mxc_adv739x", 0x2a),
		.platform_data = (void *)&adv7393_pdata,
	},

	/* HDMI Monitor (DDC channel) */
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},

	/* Touchscreen controller */
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_VENTANA_CAP_TCH_INT),
	},
};

static void imx6q_ventana_usbotg_vbus(bool on)
{
	printk(KERN_DEBUG "%s: %s\n", __func__, on?"on":"off");
	if (on)
		gpio_set_value(MX6Q_VENTANA_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_VENTANA_USB_OTG_PWR, 0);
}

static void __init imx6q_ventana_init_usb(void)
{
	int ret = 0;

/* 10.4.1.1.3 and 50.4 USB LDO:
 * USB_LDO regulates down USB VBUS to 3.0V to power the USB core
 * The regulator has a built-in mux that allows it to be run from either one
 * of the VBUS supplies (USB_H1_VBUS, USB_OTG_VBUS) when both are present.
 * If only 1 is present it automatically selects that one.
 *
 * 10.3.2.3 PLLs
{
	int pmu_reg_3p0 = __raw_readl(MXC_PLL_BASE + 0x120);

	printk("%s: pmu_reg_3p0=0x%08x\n", __func__, pmu_reg_3p0);
	pmu_reg_3p0 &= ~(1<<7); // select VBUS OTG1
	pmu_reg_3p0 |= 1<<7;    // select VBUS OTG2
	__raw_writel(pmu_reg_3p0, MXC_PLL_BASE + 0x120);
}
 */

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_VENTANA_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_VENTANA_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_export(MX6Q_VENTANA_USB_OTG_PWR, 0);
	gpio_direction_output(MX6Q_VENTANA_USB_OTG_PWR, 0);
	/* IOMUX_GPR1[13]=1 selects GPIO_1 for USB_OTG_ID */
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_ventana_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_ventana_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_ventana_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_ventana_sata_data = {
	.init = mx6q_ventana_sata_init,
	.exit = mx6q_ventana_sata_exit,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6q_ventana_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6Q_VENTANA_CAN_STBY, 0);
	} else {
		gpio_set_value(MX6Q_VENTANA_CAN_STBY, 1);
	}
}

static const struct flexcan_platform_data
		mx6q_ventana_flexcan0_pdata __initconst = {
		.transceiver_switch = mx6q_ventana_flexcan0_switch,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(IMX_GPIO_NR(1,11), 1);
	gpio_set_value(IMX_GPIO_NR(1,21), 1);
	udelay(10);
	gpio_set_value(IMX_GPIO_NR(1,21), 0);
	udelay(50);
	gpio_set_value(IMX_GPIO_NR(1,21), 1);

	/* TRULY-WVGA needs to delay 120ms min for reset complete */
	msleep(120);
}

/* MIPI Display Out - IPU1_DISP1 */
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id   = 0,
	.disp_id  = 1,
	.lcd_panel  = "TRULY-WVGA",
	.reset    = mx6_reset_mipi_dsi,
};

/* mxc Framebuffer devices
 *
 * i.MX6Q has 2 IPU's each with 2 display ports thus can have 4 simultaneous
 * displays.  The defaults can be changed with video= kernel commandline.
 * For single IPU processors only 2 can be registered.  The first
 * device registered per IPU uses the Display Processor (DP) providing
 * combining of 2 video/graphic planes thus 2 framebuffer devices are
 * registered for those devices.
 *
 *  @disp_dev - name of output interface used for display
 *              (matching a registered mxc_display driver)
 *              ldb     = LVDS interface
 *              hdmi    = HDMI interface
 *              adv7393 = CVBS interface
 *  @interface_pix_fmt - pixel format at the IPU transmitter
 *              RGB666 - 18bit devices
 *              RGB24  - 24bit devices
 *              BT656  - 8bit BT656 devices
 *  @fb_pix_fmt - pixel format of the framebuffer device
 *              IPU_PIX_FMT_RGB565
 *  @default_bpp - default bit-depth of the framebuffer
 *  @mode_str - named panel string or a resolution in the VESA coordinated video
 *              timings format (see Documentation/fb/modedb.txt)
 *
 *  Valid Examples:
 *    HDMI out on /dev/fb0,1:
 *       video=mxcfb0:dev=hdmi,1280x720M@60,if=RGB24
 *    Freescale MXC-LVDS1 10" 1024x768 on /dev/fb2,3:
 *       video=mxcfb1:dev=ldb,LDB-XGA,if=RGB666
 *    CVBS on /dev/fb0,1 + HDMI on /dev/fb1,2:
 *       video=mxcfb0:dev=adv739x,BT656-NTSC,if=BT656,fbpix=RGB565 \
 *             mxcfb1:dev=ldb,LDB-XGA,if=RGB666 \
 *             mxcfb2:off
 */
static struct ipuv3_fb_platform_data ventana_fb_data[] = {
	{
		/* HDMI */
		.disp_dev = "hdmi",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "1280x720M@60",
		.default_bpp = 16,
		.int_clk = false,
	}, {
		/* Analog Video */
		.disp_dev = "adv739x",
		.interface_pix_fmt = IPU_PIX_FMT_BT656,
		.fb_pix_fmt = IPU_PIX_FMT_RGB565,
		.mode_str = "BT656-NTSC",
		.default_bpp = 16,
		.int_clk = false,
	}, {
		/* Freescale MXC-LVDS1 */
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "LDB-XGA",
		.default_bpp = 16,
		.int_clk = false,
	}, {
		/* MIPI DSI */
		.disp_dev = "mipi_dsi",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "TRULY-WVGA",
		.default_bpp = 24,
		.int_clk = false,
	},
};


/* Initialize HDMI */
static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

	printk(KERN_INFO "%s IPU%d_DISP%d\n", __func__, ipu_id+1, disp_id);
	if ((ipu_id > max_ipu_id) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 * when hdcp enable, the pins should work at ddc function
 */
static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_ventana_hdmi_ddc_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_ventana_hdmi_ddc_pads));
	}
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_i2c2_pads,
			ARRAY_SIZE(mx6q_ventana_i2c2_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_i2c2_pads,
			ARRAY_SIZE(mx6dl_ventana_i2c2_pads));
	}
}

/* Platform data for mxc_hdmi driver */
static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

/* HDMI out: IPU1_DISP1 */
static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 1,
};

/* LCD Video Out: IPU2_DISP1 */
static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

/* LVDS out: IPU1_DISP1/DISP0 */
static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	/* secondary LVDS port not used but not clear how to disable it */
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

/* Analog Video Out: IPU2_DISP0 */
static struct fsl_mxc_lcd_platform_data bt656_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_BT656,
};

/* bypass_reset - check this - its not set false on sabrelite
 */
static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		/* IPU1 */
		.rev = 4,
		.csi_clk[0] = "clko_clk",
		.bypass_reset = false,
	}, {
		/* IPU2 */
		.rev = 4,
		.csi_clk[0] = "clko_clk",
		.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		/* /dev/video0: IPU1_CSI0 - HDMI In Digital Receiver */
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		/* /dev/video1: IPU2_CSI1 - ADV7180 Analog Video Decoder */
		.csi = 1,
		.ipu = 1,
		.mclk_source = 0,
		.is_mipi = 0,
	},
};


static void ventana_suspend_enter(void)
{
	/* suspend preparation */
	// TODO: disable SWBST
}

static void ventana_suspend_exit(void)
{
	/* resume restore */
	// TODO: resume SWBST
}

static const struct pm_platform_data mx6q_ventana_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = ventana_suspend_enter,
	.suspend_exit = ventana_suspend_exit,
};


/* VMMC Regulator (internal fixed) */
static struct regulator_consumer_supply ventana_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data ventana_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ventana_vmmc_consumers),
	.consumer_supplies = ventana_vmmc_consumers,
};

static struct fixed_voltage_config ventana_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ventana_vmmc_init,
};

static struct platform_device ventana_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &ventana_vmmc_reg_config,
	},
};


/* VDD_DLY_3P3 - Delayed 3.3V fixed from main power supply
 *  enabled via VGEN6 of PFUZE
 */
static struct regulator_consumer_supply ventana_vdd_dly_3p3_consumers[] = {
	REGULATOR_SUPPLY("VDDIO", "2-000a"), // SGTL5000 audio codec
	REGULATOR_SUPPLY("VDDIO", "2-0020"), // ADV7180 video decoder 
	REGULATOR_SUPPLY("VDDIO", "2-004c"), // ADV7611 hdmi decoder 
	REGULATOR_SUPPLY("VDDIO", "2-002a"), // ADV7393 video encoder
//	REGULATOR_SUPPLY("VDDIO", "2-00XX"), // MIPI Expansion
//	REGULATOR_SUPPLY("VDDIO", "2-00XX"), // LVDS Expansion
	REGULATOR_SUPPLY("VDDIO", "1-006b"), // SI52147 PCIe clock generator
};

static struct regulator_init_data ventana_vdd_dly_3p3_init = {
	.num_consumer_supplies = ARRAY_SIZE(ventana_vdd_dly_3p3_consumers),
	.consumer_supplies = ventana_vdd_dly_3p3_consumers,
};

static struct fixed_voltage_config ventana_vdd_dly_3p3_reg_config = {
	.supply_name = "VDD_DLY_3P3",
	.microvolts  = 3300000,
	.gpio        = -1,
	.init_data   = &ventana_vdd_dly_3p3_init,
};

static struct platform_device mx6_ventana_vdd_dly_3p3_device = {
	.name = "reg-fixed-voltage",
	.id = 4,
	.dev = {
		.platform_data = &ventana_vdd_dly_3p3_reg_config,
	},
};


/* initialize audio
 */
static int imx6_init_audio(void)
{
	mxc_register_device(&mx6_ventana_analog_audio_device,
			    &mx6_ventana_analog_audio_data);
	imx6q_add_imx_ssi(1, &mx6_ventana_ssi1_pdata);

	mxc_register_device(&mx6_ventana_digital_audio_device,
			    &mx6_ventana_digital_audio_data);
	imx6q_add_imx_ssi(0, &mx6_ventana_ssi2_pdata);

	return 0;
}


/* PWM4_PWMO (LVDS display backlight control)
 */
static struct platform_pwm_backlight_data mx6_ventana_pwm_backlight_data = {
	.pwm_id = 3,  // PWM4
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};


/* DVFS regulator setup
 */
static struct mxc_dvfs_platform_data ventana_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id = "VDDSOC",
//	.pu_id = "cpu_vddvpu",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};


/* Misc fixups
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}


/* CSI2 - MIPI Capture
 */
static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};


/* Crypto enable/disable - default disabled as it seems to take over CLKO
 */
static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);


/* PCIe
 */
static struct imx_pcie_platform_data mx6_ventana_pcie_data = {
	/* GPIO for enabling PCIE 3P3 */
	.pcie_pwr_en	= -EINVAL,
	/* GPIO to miniPCIe PERST# (pin22)     - Power Good / Reset */
	.pcie_rst	= -1,
	/* GPIO to miniPCIe WAKE# (pin1)       - Wake event */
	.pcie_wake_up	= -EINVAL,
	/* GPIO to miniPCIe W_DISABLE# (pin20) - Wireless disable */
	.pcie_dis	= -EINVAL,
};


/* fixup for PEX 8909 bridge to configure GPIO1-7 as output High
 * as they are used for slots1-7 PERST#
 */
static void mx6_ventana_pciesw_early_fixup(struct pci_dev *dev)
{
	u32 dw;

	if (dev->devfn != 0)
		return;

	pci_read_config_dword(dev, 0x62c, &dw);
	dw |= 0xaaa8; // GPIO1-7 outputs
	pci_write_config_dword(dev, 0x62c, dw);	

	pci_read_config_dword(dev, 0x644, &dw);
	dw |= 0xfe;  // GPIO1-7 output high 
	pci_write_config_dword(dev, 0x644, dw);	
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8609,
	mx6_ventana_pciesw_early_fixup);


/*!
 * Board specific initialization.
 */
static void __init mx6_ventana_board_init(void)
{
	struct clk *clko2;
	struct clk *new_parent;
	int rate;

	memset(&ventana_board_info, 0, sizeof(ventana_board_info));

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_pads,
						ARRAY_SIZE(mx6q_ventana_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_pads,
						ARRAY_SIZE(mx6dl_ventana_pads));
	}

	/* Delayed 3.3V supply */
	platform_device_register(&mx6_ventana_vdd_dly_3p3_device);

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	/* regulators for DVS */
	gp_reg_id = ventana_dvfscore_data.reg_id;
	soc_reg_id = ventana_dvfscore_data.soc_id;

	/* HDMI output core */
// TODO:can not dynamically adjust hdmi_core_data per model
//      with this early registration
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	/* I2C */
	imx6q_add_imx_i2c(0, &mx6q_ventana_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	// TODO: move i2c-1 and i2c-2 after model detect so the device-list is dynamic
	imx6q_add_imx_i2c(1, &mx6q_ventana_i2c_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	imx6q_add_imx_i2c(2, &mx6q_ventana_i2c_data);
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* Clocks */
	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	imx6q_add_busfreq();

	/* PMIC */
	mx6q_ventana_init_pfuze100(IMX_GPIO_NR(1,8));

	/* USB */
	imx6q_ventana_init_usb();

	/* SATA - must be registered early as uses platform_driver_probe */
	imx6q_add_ahci(0, &mx6q_ventana_sata_data);

	/* RTC */
	//imx6q_add_imx_snvs_rtc();

	/* thermal sensor */
	imx6q_add_anatop_thermal_imx(1, &mx6q_ventana_anatop_thermal_data);

	/* Power Management */
	imx6q_add_pm_imx(0, &mx6q_ventana_pm_data);

	/* GPU */
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

	/* PWM */
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&ventana_dvfscore_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

static int __init ventana_model_setup(void)
{
	struct ventana_board_info *info = &ventana_board_info;
	int i;

	printk("Running on Gateworks Ventana %s\n", info->model);

	if (strncmp(info->model, "GW", 2) == 0) {
		iomux_v3_cfg_t pad;
		int pwm_leds = 0;

		/*
		 * model specific setup
		 */
		if (strncmp(info->model, "GW54", 4) == 0) {
			if (strncmp(info->model, "GW5400-A", 8) == 0) {

				/* UARTs */
				mxc_iomux_v3_setup_multiple_pads(mx6q_gw5400a_uart_pads,
					ARRAY_SIZE(mx6q_gw5400b_uart_pads));
				imx6q_add_imx_uart(0, NULL);
				imx6q_add_imx_uart(1, NULL);
				imx6q_add_imx_uart(2, NULL);

				/* SPI FLASH */
				if (info->config_spifl0 || info->config_spifl1) {
					if (cpu_is_mx6q()) {
						mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_spi_pads,
							ARRAY_SIZE(mx6q_ventana_spi_pads));
					} else if (cpu_is_mx6dl()) {
						mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_spi_pads,
							ARRAY_SIZE(mx6dl_ventana_spi_pads));
					}
					imx6q_add_ecspi(0, &mx6q_ventana_spi_data);
					spi_device_init();
				}

				/* User LEDs */
				mx6_ventana_led_pdata.num_leds = 3;
				mx6_ventana_gpio_leds[0].gpio = IMX_GPIO_NR(4,6);    // user1:panledg
				mx6_ventana_gpio_leds[1].gpio = IMX_GPIO_NR(4,10);   // user2:panledr
				mx6_ventana_gpio_leds[2].gpio = IMX_GPIO_NR(4,15);   // user3:locled

				/* VIDDEC_IRQ# */
				//mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_2__GPIO_1_2);

				/* MIPI DSI */
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD1_DAT3__GPIO_1_21); // MIPI GP DIO	
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_CMD__GPIO_1_11);  // MIPI BK_EN
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD1_DAT1__PWM3_PWMO); // BL brightness

				/* UART1 Transmit Enable */
				gpio_request(IMX_GPIO_NR(3,24), "rs485-txen");
				gpio_export(IMX_GPIO_NR(3,24), 0);
				gpio_direction_output(IMX_GPIO_NR(3,24), 0);

				/* Mezzanine Expansion */
				gpio_request(IMX_GPIO_NR(4,7), "exp_pwren#"); // Power Enable
				gpio_export(IMX_GPIO_NR(4,7), 1);
				gpio_direction_output(IMX_GPIO_NR(4,7), 0);
				gpio_request(IMX_GPIO_NR(4,9), "exp_irq#");   // IRQ
				gpio_export(IMX_GPIO_NR(4,9), 1);
				gpio_direction_input(IMX_GPIO_NR(4,9));
			} /* end GW5400-A */

			else if ( (strncmp(info->model, "GW5400", 6) == 0)
		         || (strncmp(info->model, "GW5410", 6) == 0)
			) {
				/* UARTs */
				mxc_iomux_v3_setup_multiple_pads(mx6q_gw5400b_uart_pads,
					ARRAY_SIZE(mx6q_gw5400b_uart_pads));
				imx6q_add_imx_uart(0, NULL);
				imx6q_add_imx_uart(1, NULL);
				imx6q_add_imx_uart(4, NULL);

				/* User LEDs */
				mx6_ventana_led_pdata.num_leds = 3;
				mx6_ventana_gpio_leds[0].gpio = IMX_GPIO_NR(4,6);  // user1:panledg
				mx6_ventana_gpio_leds[1].gpio = IMX_GPIO_NR(4,7);  // user2:panledr
				mx6_ventana_gpio_leds[2].gpio = IMX_GPIO_NR(4,15); // user3:locled

				/* i2s audio */
				if (info->config_ssi1) {
					mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_audmux5_pads,
						ARRAY_SIZE(mx6q_ventana_audmux5_pads));
				}

				/* PWM4 (backlight control) */
				mxc_iomux_v3_setup_pad(NEW_PAD_CTRL(MX6Q_PAD_SD1_CMD__PWM4_PWMO,
					MX6Q_VENTANA_SD1_CMD_PADCFG));

				/* VIDDEC_IRQ# */
				//mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_A17__GPIO_2_21);

				/* UART1 Transmit Enable */
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD3_DAT4__GPIO_7_1),
				gpio_request(IMX_GPIO_NR(7,1), "rs485-txen");
				gpio_export(IMX_GPIO_NR(7,1), 0);
				gpio_direction_output(IMX_GPIO_NR(7,1), 0);

				/* Mezzanine Expansion */
				gpio_request(IMX_GPIO_NR(2,19), "exp_pwren#"); // Power Enable
				gpio_export(IMX_GPIO_NR(2,19), 0);
				gpio_direction_output(IMX_GPIO_NR(2,19), 0);
				gpio_request(IMX_GPIO_NR(2,18), "exp_irq#");   // IRQ
				gpio_export(IMX_GPIO_NR(2,18), 0);
				gpio_direction_input(IMX_GPIO_NR(2,18));

				/* NAND */
				if (info->config_nand)
					imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

				/* Video out: Modify mxcfb array to better suit this board */
				sprintf(ventana_fb_data[3].disp_dev, "off");

			} /* end GW5400-B/GW5410-B */

			/* Touchscreen IRQ */
			mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_17__GPIO_7_12);

			/* Digital Video Decoder (HDMI IN) IRQ */
			mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_7__GPIO_1_7);

			/* release USB Hub reset */
			gpio_set_value(IMX_GPIO_NR(1, 16), 1);

			/*
			 * Disable HannStar touch panel CABC function,
			 * this function turns the panel's backlight automatically
			 * according to the content shown on the panel which
			 * may cause annoying unstable backlight issue.
			 */
			mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_CLK__GPIO_1_10);
			gpio_request(IMX_GPIO_NR(1,10), "cabc-en0");
			gpio_export(IMX_GPIO_NR(1,10), 0);
			gpio_direction_output(IMX_GPIO_NR(1,10), 0);

			/* User IO's are configured by bootloader per hwconfig as GPIO or PWM
			 *  - determine which and register properly
			 */ 
			pr_info("Registering UserIO:\n");
			/* DIO0 */
			pad = MX6Q_PAD_GPIO_9__PWM1_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 4) // ALT4
			{
				pr_info("DIO0: PWM1\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio0";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 0;
				pwm_leds++;
			} else {
				pr_info("DIO0: GPIO%d\n", IMX_GPIO_NR(1,9));
				gpio_request(IMX_GPIO_NR(1,9), "dio0");
				gpio_export(IMX_GPIO_NR(1,9), 1);
				gpio_direction_input(IMX_GPIO_NR(1,9));
			}
			/* DIO1 */
			pad = MX6Q_PAD_SD1_DAT2__PWM2_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 3) // ALT3
			{
				pr_info("DIO1: PWM2\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio1";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 1;
				pwm_leds++;
			} else {
				pr_info("DIO1: GPIO%d\n", IMX_GPIO_NR(1,19));
				gpio_request(IMX_GPIO_NR(1,19), "dio1");
				gpio_export(IMX_GPIO_NR(1,19), 1);
				gpio_direction_input(IMX_GPIO_NR(1,19));
			}
			/* DIO2 */
			pad = MX6Q_PAD_SD4_DAT1__PWM3_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 2) // ALT2
			{
				pr_info("DIO2: PWM3\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio2";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 2;
				pwm_leds++;
			} else {
				pr_info("DIO2: GPIO%d\n", IMX_GPIO_NR(2,9));
				gpio_request(IMX_GPIO_NR(2,9), "dio2");
				gpio_export(IMX_GPIO_NR(2,9), 1);
				gpio_direction_input(IMX_GPIO_NR(2,9));
			}
			/* DIO2 */
			pad = MX6Q_PAD_SD4_DAT2__PWM4_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 2) // ALT2
			{
				pr_info("DIO3: PWM4\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio3";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 3;
				pwm_leds++;
			} else {
				pr_info("DIO3: GPIO%d\n", IMX_GPIO_NR(2,10));
				gpio_request(IMX_GPIO_NR(2,10), "dio3");
				gpio_export(IMX_GPIO_NR(2,10), 1);
				gpio_direction_input(IMX_GPIO_NR(2,10));
			}

			/* PWM based LEDs */
			if (pwm_leds) {
				mx6_ventana_pwm_pdata.num_leds = pwm_leds;
				platform_device_register(&mx6_ventana_leds_pwm_device);
			}

			/* GPIO controller */
			platform_device_register(&mx6_ventana_leds_gpio_device);

#ifdef CONFIG_PPS_CLIENT_GPIO
			/* PPS source from GPS */
			mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_5__GPIO_1_5);
			mx6_ventana_pps_device.dev.platform_data.gpio_pin = IMX_GPIO_NR(1, 5);
			platform_device_register(&mx6_ventana_pps_device);
#endif

			/* serial console rs232 driver enable */
			gpio_request(IMX_GPIO_NR(2,11), "uart2_en");
			gpio_export(IMX_GPIO_NR(2,11), 0);
			gpio_direction_output(IMX_GPIO_NR(2,11), 0);

			/* i2c switch enable */
			mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_19__GPIO_4_5);
			gpio_request(IMX_GPIO_NR(4,5), "i2c_dis#");
			gpio_export(IMX_GPIO_NR(4,5), 0);
			gpio_direction_output(IMX_GPIO_NR(4,5), 0);

			/* backlight pwm */
			if (info->config_lvds0 || info->config_lvds1)
				imx6q_add_mxc_pwm_backlight(3, &mx6_ventana_pwm_backlight_data);

			/* analog display out */
			mxc_iomux_v3_setup_multiple_pads(mx6q_disp0_pads,
					ARRAY_SIZE(mx6q_disp0_pads));

			/* PCIe */
			mx6_ventana_pcie_data.pcie_rst	= IMX_GPIO_NR(1, 29);
		} /* end GW54xx */

		else if (strncmp(info->model, "GW51", 4) == 0) {

			/* UARTs */
			mxc_iomux_v3_setup_multiple_pads(mx6dl_gw51xx_uart_pads,
				ARRAY_SIZE(mx6dl_gw51xx_uart_pads));
			imx6q_add_imx_uart(0, NULL);
			imx6q_add_imx_uart(1, NULL);
			imx6q_add_imx_uart(2, NULL);
			imx6q_add_imx_uart(4, NULL);

			/* User LEDs */
			mx6_ventana_led_pdata.num_leds = 2;
			mx6_ventana_gpio_leds[0].gpio = IMX_GPIO_NR(4,6); // user1:panledg
			mx6_ventana_gpio_leds[1].gpio = IMX_GPIO_NR(4,7); // user2:panledr

			/* Mezzanine Expansion */
			gpio_request(IMX_GPIO_NR(2,19), "exp_pwren#"); // Power Enable
			gpio_export(IMX_GPIO_NR(2,19), 0);
			gpio_direction_output(IMX_GPIO_NR(2,19), 0);
			gpio_request(IMX_GPIO_NR(2,18), "exp_irq#");   // IRQ
			gpio_export(IMX_GPIO_NR(2,18), 0);
			gpio_direction_input(IMX_GPIO_NR(2,18));

			/* Video Decoder Powerdown */
			if (info->config_vid_in) {
				/* use IPU1_CSI0 for adv7180 */
				adv7180_pdata.csi = 0;
				adv7180_pdata.io_init = mx6_csi0_io_init;
			}

			/* NAND */
			if (info->config_nand)
				imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

			/* User IO's are configured by bootloader per hwconfig as GPIO or PWM
			 *  - determine which and register properly
			 */ 
			pr_info("Registering UserIO:\n");
			/* DIO0 (can only be GPIO) */
			pr_info("DIO0: GPIO%d\n", IMX_GPIO_NR(1,16));
			gpio_request(IMX_GPIO_NR(1,16), "dio0");
			gpio_export(IMX_GPIO_NR(1,16), 1);
			gpio_direction_input(IMX_GPIO_NR(1,16));
			/* DIO1 */
			pad = MX6DL_PAD_SD1_DAT2__PWM2_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 3) // ALT3
			{
				pr_info("DIO1: PWM2\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio1";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 1;
				pwm_leds++;
			} else {
				pr_info("DIO1: GPIO%d\n", IMX_GPIO_NR(1,19));
				gpio_request(IMX_GPIO_NR(1,19), "dio1");
				gpio_export(IMX_GPIO_NR(1,19), 1);
				gpio_direction_input(IMX_GPIO_NR(1,19));
			}
			/* DIO2 */
			pad = MX6DL_PAD_SD1_DAT1__PWM3_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 2) // ALT2
			{
				pr_info("DIO2: PWM3\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio2";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 2;
				pwm_leds++;
			} else {
				pr_info("DIO2: GPIO%d\n", IMX_GPIO_NR(1,17));
				gpio_request(IMX_GPIO_NR(1,17), "dio2");
				gpio_export(IMX_GPIO_NR(1,17), 1);
				gpio_direction_input(IMX_GPIO_NR(1,17));
			}
			/* DIO3 */
			pad = MX6DL_PAD_SD1_CMD__PWM4_PWMO;
			mxc_iomux_v3_get_pad(&pad);
			if ( ((pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT) == 2) // ALT2
			{
				pr_info("DIO3: PWM4\n");
				mx6_ventana_pwm_leds[pwm_leds].name = "dio3";
				mx6_ventana_pwm_leds[pwm_leds].pwm_id = 3;
				pwm_leds++;
			} else {
				pr_info("DIO3: GPIO%d\n", IMX_GPIO_NR(1,18));
				gpio_request(IMX_GPIO_NR(1,18), "dio3");
				gpio_export(IMX_GPIO_NR(1,18), 1);
				gpio_direction_input(IMX_GPIO_NR(1,18));
			}

			/* PWM based LEDs */
			if (pwm_leds) {
				mx6_ventana_pwm_pdata.num_leds = pwm_leds;
				platform_device_register(&mx6_ventana_leds_pwm_device);
			}

			/* GPIO controller */
			platform_device_register(&mx6_ventana_leds_gpio_device);

#ifdef CONFIG_PPS_CLIENT_GPIO
			/* PPS source from GPS */
			mx6_ventana_pps_device.dev.platform_data.gpio_pin = IMX_GPIO_NR(1, 7);
			platform_device_register(&mx6_ventana_pps_device);
#endif

			/* PCI Reset */
			mx6_ventana_pcie_data.pcie_rst = IMX_GPIO_NR(1, 0);

			/* Video out: Modify mxcfb array to better suit this board */
			sprintf(ventana_fb_data[1].disp_dev, "off");
			sprintf(ventana_fb_data[2].disp_dev, "off");
			sprintf(ventana_fb_data[3].disp_dev, "off");
		}

		else {
			printk("Invalid Ventana configuration\n");
		}

		/* MIPI DSI Output */
		if (info->config_mipi_dsi)
			imx6q_add_mipi_dsi(&mipi_dsi_pdata);

		/* Sync Display device output */
		if (info->config_lvds0 || info->config_lvds1)
			imx6q_add_ldb(&ldb_data);     // LVDS interface
		if (info->config_lcd)
			imx6q_add_lcdif(&lcdif_data); // parallel RGB interface
		if (info->config_vid_out)
			imx6q_add_bt656(&bt656_data); // BT656 interface
		/* HDMI output */
		if (info->config_hdmi_out)
			imx6q_add_mxc_hdmi(&hdmi_data);

		/* IPU (imx6q has 2, imx6dl has 1) */
		if (info->config_ipu0)
			imx6q_add_ipuv3(0, &ipu_data[0]);
		if (info->config_ipu1 && cpu_is_mx6q())
			imx6q_add_ipuv3(1, &ipu_data[1]);
		if (info->config_ipu0 || info->config_ipu1)
			imx6q_add_vdoa();            // Video Data Order Adapter

		/* MXC Framebuffer device */
		if (info->config_ipu1 && cpu_is_mx6q()) {
			for (i = 0; i < 4 && i < ARRAY_SIZE(ventana_fb_data); i++)
				imx6q_add_ipuv3fb(i, &ventana_fb_data[i]);
		} else if (info->config_ipu0) {
			for (i = 0; i < 2 && i < ARRAY_SIZE(ventana_fb_data); i++)
				imx6q_add_ipuv3fb(i, &ventana_fb_data[i]);
		}

		/* V4L2 output */
		if ( info->config_lvds0 || info->config_lvds1
		  || info->config_vid_out || info->config_hdmi_out)
		{
			imx6q_add_v4l2_output(0);
		}

		/* /dev/video0 HDMI Receiver */
		if (info->config_hdmi_in) {
			/* add video driver */
			platform_device_register_resndata(NULL, "tda1997x-video", 0, NULL, 0,
				&mxc_tda1997x_video_pdata, sizeof(mxc_tda1997x_video_pdata));
			/* add audio driver */
			platform_device_register_resndata(NULL, "tda1997x_codec", 0, NULL, 0,
				NULL, 0);
			imx_add_platform_device("imx-tda1997x-dai", 0, NULL, 0, NULL, 0);
		}

		/* /dev/video1 ADV7180 Analog Video Decoder */
		if (info->config_csi0)
			imx6q_add_v4l2_capture(0, &capture_data[0]);
		if (info->config_csi1)
			imx6q_add_v4l2_capture(1, &capture_data[1]);

		/* MIPI input */
		// NB: need to register pdata even if not using mipi as ipu_csi_enc looks for it
		//if (info->config_mipi_csi)
			imx6q_add_mipi_csi2(&mipi_csi2_pdata);

		/* this clobbers CLKO used for sgtl150000 */
		if (info->config_caam && 1 == caam_enabled)
			imx6q_add_imx_caam();

		/* GigE MAC */
		if (info->config_eth0) {
			memcpy(&fec_data.mac, info->mac0, ETH_ALEN);
			imx6_init_fec(fec_data);
		}

		/* MMC */
		if (info->config_sd0 || info->config_sd1
		  || info->config_sd2 || info->config_sd3)
			platform_device_register(&ventana_vmmc_reg_devices);
		if (info->config_sd2 /* 0-based */)
			imx6q_add_sdhci_usdhc_imx(2, &mx6q_ventana_sd3_data);

		/* VPU */
		if (info->config_vpu)
			imx6q_add_vpu();

		/* Audio */
		if (info->config_ssi0) {
			if (cpu_is_mx6q()) {
				mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_audmux4_pads,
					ARRAY_SIZE(mx6q_ventana_audmux4_pads));
			} else if (cpu_is_mx6dl()) {
				mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_audmux4_pads,
					ARRAY_SIZE(mx6dl_ventana_audmux4_pads));
			}
		}
		if (info->config_ssi1) {
			if (cpu_is_mx6q()) {
				mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_audmux5_pads,
					ARRAY_SIZE(mx6q_ventana_audmux5_pads));
			} else if (cpu_is_mx6dl()) {
				mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_audmux5_pads,
					ARRAY_SIZE(mx6dl_ventana_audmux5_pads));
			}
		}
		if (info->config_ssi0 || info->config_ssi1) {
			imx6_init_audio();
			imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
			imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
			imx6q_add_asrc(&imx_asrc_data);
		}

		/* CANbus */
		if (info->config_flexcan) {
			if (cpu_is_mx6q()) {
					mxc_iomux_v3_setup_multiple_pads(mx6q_ventana_flexcan_pads,
						ARRAY_SIZE(mx6q_ventana_flexcan_pads));
			} else if (cpu_is_mx6dl()) {
					mxc_iomux_v3_setup_multiple_pads(mx6dl_ventana_flexcan_pads,
						ARRAY_SIZE(mx6dl_ventana_flexcan_pads));
			}
			gpio_request(MX6Q_VENTANA_CAN_STBY, "can0_stby");
			gpio_direction_output(MX6Q_VENTANA_CAN_STBY, 0);
			gpio_set_value(MX6Q_VENTANA_CAN_STBY, 1);
			imx6q_add_flexcan0(&mx6q_ventana_flexcan0_pdata);
		}

		/* HDMI audio out */
		if (info->config_hdmi_out) {
			imx6q_add_hdmi_soc();
			imx6q_add_hdmi_soc_dai();
		}

		/* PCIe RC interface */
		if (info->config_pcie)
			imx6q_add_pcie(&mx6_ventana_pcie_data);

	} else {
	}

	return 0;
}
late_initcall(ventana_model_setup);

extern void __iomem *twd_base;
static void __init mx6_ventana_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_ventana_timer = {
	.init   = mx6_ventana_timer_init,
};

static void __init mx6q_ventana_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_GWVENTANA data structure.
 */
MACHINE_START(GWVENTANA, "Gateworks i.MX6Q/DL Ventana Board")
	/* Maintainer: Gateworks Corporation */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_ventana_board_init,
	.timer = &mx6_ventana_timer,
	.reserve = mx6q_ventana_reserve,
MACHINE_END
