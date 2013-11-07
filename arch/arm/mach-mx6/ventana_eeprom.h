/*
 * ventana_eeprom.h - Gateworks Ventana EEPROM Configuration
 * v1.00
 */
#ifndef _VENTANA_EEPROM_
#define _VENTANA_EEPROM_

struct ventana_board_info {
	u8 mac0[6];          // 0x00: MAC1
	u8 mac1[6];          // 0x06: MAC2
	u8 res0[12];         // 0x0C: reserved
	u32 serial;          // 0x18: Serial Number (read only)
	u8 console;          // 0x1C: console UART 
	u8 res1[3];          // 0x1D: reserved
	u8 mfgdate[4];       // 0x20: MFG date (read only)
	// GPIO config
	u8 dio0;             // 0x24: 0=GPIO|1=PWM
	u8 dio1;             // 0x25: 0=GPIO|1=PWM
	u8 dio2;             // 0x26: 0=GPIO|1=PWM
	u8 dio3;             // 0x27: 0=GPIO|1=PWM
	u8 msata_en;         // 0x28: 0=PCIe|1=mSATA
	u8 rs232_en;         // 0x29: RS232 Enable 
	u8 res2[1];          // 0x30
	// sdram config
	u8 sdram_size;       // 0x2B: enum (512,1024,2048) MB
	u8 sdram_speed;      // 0x2C: enum (100,133,166,200,267,333,400) MHz
	u8 sdram_width;      // 0x2D: enum (32,64) bit
	// cpu config
	u8 cpu_speed;        // 0x2E: enum (800,1000,1200) MHz
	u8 cpu_type;         // 0x2F: enum (imx6q,imx6d,imx6dl,imx6s)
	u8 model[16];        // 0x30: model string (affects kernel board setup)
	// FLASH config
	u8 nand_flash_size;  // 0x40: enum (4,8,16,32,64,128) MB
	u8 spi_flash_size;   // 0x41: enum (4,8,16,32,64,128) MB

	// Config1: SoC Peripherals
	u8 config_eth0:1;    // 0: 0x42
	u8 config_eth1:1;    // 1
	u8 config_hdmi_out:1;// 2
	u8 config_sata:1;    // 3
	u8 config_pcie:1;    // 4
	u8 config_ssi0:1;    // 5
	u8 config_ssi1:1;    // 6
	u8 config_lcd:1;     // 7

	u8 config_lvds0:1;   // 0: 0x43
	u8 config_lvds1:1;   // 1
	u8 config_usb0:1;    // 2 (USB EHCI)
	u8 config_usb1:1;    // 3 (USB OTG)
	u8 config_sd0:1;     // 4
	u8 config_sd1:1;     // 5
	u8 config_sd2:1;     // 6
	u8 config_sd3:1;     // 7

	u8 config_uart0:1;   // 0: 0x44
	u8 config_uart1:1;   // 1
	u8 config_uart2:1;   // 2
	u8 config_uart3:1;   // 3
	u8 config_uart4:1;   // 4
	u8 config_ipu0:1;    // 5
	u8 config_ipu1:1;    // 6
	u8 config_flexcan:1; // 7

	u8 config_mipi_dsi:1;// 0: 0x45
	u8 config_mipi_csi:1;// 1
	u8 config_tzasc0:1;  // 2
	u8 config_tzasc1:1;  // 3
	u8 config_i2c0:1;    // 4
	u8 config_i2c1:1;    // 5
	u8 config_i2c2:1;    // 6
	u8 config_vpu:1;     // 7

	u8 config_csi0:1;    // 0: 0x46
	u8 config_csi1:1;    // 1
	u8 config_caam:1;    // 2
	u8 config_mezz:1;    // 3
	u8 config_res1:1;    // 4
	u8 config_res2:1;    // 5
	u8 config_res3:1;    // 6
	u8 config_res4:1;    // 7

	u8 config_espci0:1;  // 0: 0x47
	u8 config_espci1:1;  // 1
	u8 config_espci2:1;  // 2
	u8 config_espci3:1;  // 3
	u8 config_espci4:1;  // 4
	u8 config_espci5:1;  // 5
	u8 config_res5:1;    // 6
	u8 config_res6:1;    // 7

	// Config2: Other Peripherals 
	u8 config_gps:1;     // 0: 0x48
	u8 config_spifl0:1;  // 1
	u8 config_spifl1:1;  // 2
	u8 config_gspbatt:1; // 3
	u8 config_hdmi_in:1; // 4
	u8 config_vid_out:1; // 5
	u8 config_vid_in:1;  // 6
	u8 config_nand:1;    // 7

	u8 config_res8:1;    // 0: 0x49
	u8 config_res9:1;    // 1
	u8 config_res10:1;   // 2
	u8 config_res11:1;   // 3
	u8 config_res12:1;   // 4
	u8 config_res13:1;   // 5
	u8 config_res14:1;   // 6
	u8 config_res15:1;   // 7

	u8 res3[4];          // 0x4A

	u8 chksum[2];        // 0x4E
};

#endif
