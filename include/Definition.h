#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#define ADS1299_PIN_RESET 25
#define ADS1299_PIN_DRDY 27

#define ADS1299_PIN_SCK 14
#define ADS1299_PIN_MISO 12
#define ADS1299_PIN_MOSI 13
#define ADS1299_PIN_SS 15  //17

#define OPENBCI_DATA_BUFFER_SIZE 50

#define OPENBCI_NAME "OpenBCI-FFFF"
#define OPENBCI_VERSION "v2.0.5"

#define SOFT_AP_SSID "OpenBCI WiFi AP"
#define SOFT_AP_PASSWORD "12345678"

#define JSON_BUFFER_SIZE 1024
#define ADS_ID  0x3E  // product ID for ADS1299
#define ID_REG  0x00  // this register contains ADS_ID
#define BOARD_ADS  15  // ADS chip select
#define _SDATAC 0x11 // Stop Read Data Continuous modeff
#define _RDATA 0x12 // Read data by command supports multiple read back
#define _RESET 0x06 // Reset the device registers to default
// Register Addresses
#define ID         0x00
#define CONFIG1    0x01
#define CONFIG2    0x02
#define CONFIG3    0x03
#define LOFF       0x04
#define CH1SET     0x05
#define CH2SET     0x06
#define CH3SET     0x07
#define CH4SET     0x08
#define CH5SET     0x09
#define CH6SET     0x0A
#define CH7SET     0x0B
#define CH8SET     0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP  0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO       0x14
#define MISC1      0x15
#define MISC2      0x16
#define CONFIG4    0x17

// Test Signal Choices - p41
#define ADS1299_TEST_INT              0x10 //(0b00010000)
#define ADS1299_TESTSIGNAL_AMP_1X     0x00 //(0b00000000)
#define ADS1299_TESTSIGNAL_AMP_2X     0x40 //(0b00000100)
#define ADS1299_TESTSIGNAL_PULSE_SLOW 0x00 //(0b00000000)
#define ADS1299_TESTSIGNAL_PULSE_FAST 0x01 //(0b00000001)
#define ADS1299_TESTSIGNAL_DCSIG      0x03 //(0b00000011)
#define ADS1299_TESTSIGNAL_NOCHANGE   0xff //(0b11111111)
// SPI Command Definitions (Datasheet, 35)
#define _WAKEUP  0x02 // Wake-up from standby mode
#define _STANDBY 0x04 // Enter Standby mode
#define _RESET   0x06 // Reset the device registers to default
#define _START   0x08 // Start and restart (synchronize) conversions
#define _STOP    0x0A // Stop conversion
#define _RDATAC  0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC  0x11 // Stop Read Data Continuous mode
#define _RDATA   0x12 // Read data by command; supports multiple read back
#define _RREG    0x20 // Read Register
#define _WREG    0x40 // Write to Register

// Gains
#define ADS1299_PGA_GAIN01 0x00 //(0b00000000)
#define ADS1299_PGA_GAIN02 0x10 //(0b00010000)
#define ADS1299_PGA_GAIN04 0x20 //(0b00100000)
#define ADS1299_PGA_GAIN06 0x30 //(0b00110000)
#define ADS1299_PGA_GAIN08 0x40 //(0b01000000)
#define ADS1299_PGA_GAIN12 0x50 //(0b01010000)
#define ADS1299_PGA_GAIN24 0x60 //(0b01100000)

// Input Modes - Channels

#define ADS1299_INPUT_PWR_DOWN   0x80 //(0b10000000)
#define ADS1299_INPUT_PWR_UP     0x00 //(0b00000000)

#define ADS1299_INPUT_NORMAL     0x00 //(0b00000000)
#define ADS1299_INPUT_SHORTED    0x01 //(0b00000001)
#define ADS1299_INPUT_MEAS_BIAS  0x02 //(0b00000010)
#define ADS1299_INPUT_SUPPLY     0x03 //(0b00000011)
#define ADS1299_INPUT_TEMP       0x04 //(0b00000100)
#define ADS1299_INPUT_TESTSIGNAL 0x05 //(0b00000101)
#define ADS1299_INPUT_SET_BIASP  0x06 //(0b00000110)
#define ADS1299_INPUT_SET_BIASN  0x07 //(0b00000111)

//Lead-off Signal Choices
#define LOFF_MAG_6NA      0x00 //(0b00000000)
#define LOFF_MAG_24NA     0x04 //(0b00000100)
#define LOFF_MAG_6UA      0x08 //(0b00001000)
#define LOFF_MAG_24UA     0x0c //(0b00001100)
#define LOFF_FREQ_DC      0x00 //(0b00000000)
#define LOFF_FREQ_7p8HZ   0x01 //(0b00000001)
#define LOFF_FREQ_31p2HZ  0x02 //(0b00000010)
#define LOFF_FREQ_FS_4    0x03 //(0b00000011)
#define PCHAN (1)
#define NCHAN (2)
#define BOTHCHAN (3)

#endif