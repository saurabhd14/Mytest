// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015

#ifndef DW1000_H
#define DW1000_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "mraa.h"
#include "../utils/utils.h"

#define DW_CS 8

// register addresses
//      Mnemonic                    Address Bytes Description
#define DW1000_DEV_ID               0x00 //     4 Device Identifier – includes device type and revision information
#define DW1000_EUI                  0x01 //     8 Extended Unique Identifier
#define DW1000_PANADR               0x03 //     4 PAN Identifier and Short Address
#define DW1000_SYS_CFG              0x04 //     4 System Configuration bitmap
#define DW1000_SYS_TIME             0x06 //     5 System Time Counter (40-bit)
#define DW1000_TX_FCTRL             0x08 //     5 Transmit Frame Control
#define DW1000_TX_BUFFER            0x09 //  1024 Transmit Data Buffer
#define DW1000_DX_TIME              0x0A //     5 Delayed Send or Receive Time (40-bit)
#define DW1000_RX_FWTO              0x0C //     2 Receive Frame Wait Timeout Period
#define DW1000_SYS_CTRL             0x0D //     4 System Control Register
#define DW1000_SYS_MASK             0x0E //     4 System Event Mask Register
#define DW1000_SYS_STATUS           0x0F //     5 System Event Status Register
#define DW1000_RX_FINFO             0x10 //     4 RX Frame Information                (in double buffer set)
#define DW1000_RX_BUFFER            0x11 //  1024 Receive Data Buffer                 (in double buffer set)
#define DW1000_RX_FQUAL             0x12 //     8 Rx Frame Quality information        (in double buffer set)
#define DW1000_RX_TTCKI             0x13 //     4 Receiver Time Tracking Interval     (in double buffer set)
#define DW1000_RX_TTCKO             0x14 //     5 Receiver Time Tracking Offset       (in double buffer set)
#define DW1000_RX_TIME              0x15 //    14 Receive Message Time of Arrival     (in double buffer set)
#define DW1000_TX_TIME              0x17 //    10 Transmit Message Time of Sending    (in double buffer set)
#define DW1000_TX_ANTD              0x18 //     2 16-bit Delay from Transmit to Antenna
#define DW1000_SYS_STATE            0x19 //     5 System State information
#define DW1000_ACK_RESP_T           0x1A //     4 Acknowledgement Time and Response Time
#define DW1000_RX_SNIFF             0x1D //     4 Pulsed Preamble Reception Configuration
#define DW1000_TX_POWER             0x1E //     4 TX Power Control
#define DW1000_CHAN_CTRL            0x1F //     4 Channel Control
#define DW1000_USR_SFD              0x21 //    41 User-specified short/long TX/RX SFD sequences
#define DW1000_AGC_CTRL             0x23 //    32 Automatic Gain Control configuration
#define DW1000_EXT_SYNC             0x24 //    12 External synchronisation control.
#define DW1000_ACC_MEM              0x25 //  4064 Read access to accumulator data
#define DW1000_GPIO_CTRL            0x26 //    44 Peripheral register bus 1 access - GPIO control
#define DW1000_DRX_CONF             0x27 //    44 Digital Receiver configuration
#define DW1000_RF_CONF              0x28 //    58 Analog RF Configuration
#define DW1000_TX_CAL               0x2A //    52 Transmitter calibration block
#define DW1000_FS_CTRL              0x2B //    21 Frequency synthesiser control block
#define DW1000_AON                  0x2C //    12 Always-On register set
#define DW1000_OTP_IF               0x2D //    18 One Time Programmable Memory Interface
#define DW1000_LDE_CTRL             0x2E //     - Leading edge detection control block
#define DW1000_DIGI_DIAGNOSTICS     0x2F //    41 Digital Diagnostics Interface
#define DW1000_PMSC                 0x36 //    48 Power Management System Control Block

#define DW1000_WRITE_FLAG           0x80 // First Bit of the address has to be 1 to indicate we want to write
#define DW1000_SUBADDRESS_FLAG      0x40 // if we have a sub address second Bit has to be 1
#define DW1000_2_SUBADDRESS_FLAG    0x80 // if we have a long sub adress (more than 7 Bit) we set this Bit in the first part

#define DW1000_SPI_CLOCK_SPEED    3000000 // In hz

class DW1000
{

public:

	DW1000(SRadioConfig *_config);

	/* TODO calibrate this values for our system */
	/* Will calculate and set TX Power setting for 0x1E register based on selected channel and PRF assuming 1.5db antenna loss */

	// Device API
	uint32_t GetDeviceID();                                                                 // gets the Device ID which should be 0xDECA0130 (good for testing SPI!)
	uint64_t GetEUI();                                                                      // gets 64 bit Extended Unique Identifier according to IEEE standard
	void     SetEUI(uint64_t EUI);                                                          // sets 64 bit Extended Unique Identifier according to IEEE standard
	float    GetVoltage();                                                                  // gets the current chip voltage measurement form the A/D converter
	uint64_t GetStatus();
	uint64_t GetRXTimestamp();
	uint64_t GetTXTimestamp();

	void SendString(char* message);                                                         // to send String with arbitrary length
	void ReceiveString(char* message);                                                      // to receive char string (length of the buffer must be 1021 to be safe)
	void SendFrame(uint8_t* message, uint16_t length, bool delayed = false);                                      // send a raw frame (length in bytes)
	void SendDelayedFrame(uint8_t* message, uint16_t length, uint64_t tx_timestamp);
	void StartRX();                                                                         // start listening for frames
	void StopTRX();                                                                         // disable tranceiver go back to idle mode

	//private:
	void LoadLDE();                                                                         // load the leading edge detection algorithm to RAM, [IMPORTANT because receiving malfunction may occur] see User Manual LDELOAD on p22 & p158
	void ResetRX();                                                                         // soft reset only the tranciever part of DW1000
	void ResetAll();                                                                        // soft reset the entire DW1000 (some registers stay as they were see User Manual)

	void SetInterrupt(bool RX, bool TX);                                                    // set Interrupt for received a good frame (CRC ok) or transmission done
	uint16_t GetFramelength();                                                              // to get the framelength of the received frame from the PHY header

	// SPI interface
	mraa_spi_context spi;
	mraa_gpio_context cs;
	SRadioConfig *config;
	bool long_frame_mode_enabled;
	/* Chip select happens automatically */

	uint8_t Read8BitFromRegister(uint8_t reg, uint16_t subaddress);                                // expressive methods to read or write the number of bits written in the name
	uint16_t Read16BitFromRegister(uint8_t reg, uint16_t subaddress);
	uint64_t Read40BitFromRegister(uint8_t reg, uint16_t subaddress);
	void Write8BitToRegister(uint8_t reg, uint16_t subaddress, uint8_t buffer);
	void Write16BitToRegister(uint8_t reg, uint16_t subaddress, uint16_t buffer);
	void Write32BitToRegister(uint8_t reg, uint16_t subaddress, uint32_t buffer);
	void Write40BitToRegister(uint8_t reg, uint16_t subaddress, uint64_t buffer);

	void ReadFromRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);       // reads the selected part of a slave register into the buffer memory
	void WriteToRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length);      // writes the buffer memory to the selected slave register
	void SetupTransaction(uint8_t reg, uint16_t subaddress, bool write);                    // sets up an SPI read or write transaction with correct register address and offset

	void PrintSystemStatusInformation();
	void PrintDiagnosticInformation();

	/* Just relevant in calibration mode -> Tells you the required calibration distance between the devices given on page 195 table 54 of the user manual*/
	double GiveCalibrationDistance();



};

#endif
