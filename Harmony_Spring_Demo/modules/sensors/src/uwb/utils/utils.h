#ifndef __UTILS_H_
#define __UTILS_H_

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* Global defines */
#define TIMEUNITS_TO_US       (1/(128*499.2))               // conversion between the decawave timeunits (ca 15.65ps) to microseconds.
#define US_TO_TIMEUNITS       (128*499.2)                   // conversion between microseconds to the decawave timeunits (ca 15.65ps).
#define DECA_TIMER_MAX_VALUE  1099511627776                 // decimal value of 2^40 to correct timeroverflow between timestamps
#define MASK_40BIT            0xFFFFFFFFFF
#define SYSTEM_WIDE_PAN_ID    1
//DEBUG MODE
//#define DEBUG_EN
#ifdef DEBUG_EN
#define Debug(x) printf x
#else
#define Debug(x)	
#endif



/* MAC-Frame types */
#define MAC_FRAME_TYPE_BEACON     0
#define MAC_FRAME_TYPE_DATA       1
#define MAC_FRAME_TYPE_ACK        2
#define MAC_FRAME_TYPE_MAC_CMD    3
#define MAC_NO_ADDRESS            0
#define MAC_SHORT_ADDRESS         2
#define MAC_LONG_ADDRESS          3


/* Global variables */

struct STimer
{
	STimer();
	void Start();
	void Stop();
	double Read();

	clock_t start;
	clock_t stop;

};


/* Configuration for decawave module */
struct SRadioConfig
{
	SRadioConfig();

	int pan_id;
	int address;

	bool energy_scan_mode;                   /* Set to false -> not implemented completly TODO */
	bool smart_tx_power_enable;              /* Warning: Only allowed when running on 6.8 Mbps */

	int channel;                             /* Can be 1, 2, 3, 4, 5, 7 other channels need manual configuration (check the mailing list) */
	bool low_pulse_repetition_frequency;     /* true: 16Mhz / false: 64Mhz */
	int sfd_style;                           /* 0: Decawave style SFD (recommended) / 1: IEEE style sfd / 2: Custom SFD */
	int data_rate;                           /* 0: 110 kbps / 1: 850 kbps / 2: 6.8 Mbps */
	int preamble_length;                     /* Only 16, 64, 1024 and 4096 are standardconform also possible: 128, 256, 512, 1536, 2048 */
	int max_frame_length;                    /* Unit: bytes Can be up standardwise to 127 bytes long (1023 with non standard phr_mode enabled (autoenable)), length contains the two octet crc if uncompressed */

	int sfd_detection_timeout;               /* unit: Symbols / default: 4096 + 64 + 1 TODO: Optimization possible by setting this to preamble_length + 1 */
	int preamble_detection_timeout;          /* Defaults to zero (inactive) and is only applicable in low power preamble hunt mode, unit is pac chunk sizes (max 65535 * current PAC size) */
	bool receiver_auto_reenable;             /* Reenables the receiver in case of an receiving error */

	/* LDE algorithm variables for setting its threshold IMPORTANT */
	int lde_noise_peak_multiplier;           /* Is recomended to 3 which results in a factor of 1.5 */
	int lde_noise_level_multiplier;          /* Is recomended for Line of sight to be 13 and for non line of sight to be 12 */

	double antenna_delay_receiver;           /* Unit is nanoseconds (example: 1.7745 ns)*/
	double antenna_delay_transmitter;        /* Unit is nanoseconds (example: 1.7745 ns)*/
	bool enable_event_counter_diagnostics;    /* Enable the event counter on the chip and reset them -> should be turned on for any debugging */
	bool enable_leds;


	/* *** Optional settings *** */
	/* Frame filtering options */
	bool enable_frame_filtering;             /* Enable frame filtering functionality default: false -> if true please set options below */
	bool accept_all_matching_source_pans;    /* Enables frame reception on matching PAN_ID at source of the frame even without destination address / Also called coordinator mode */
	bool accept_beacon_frames;               /* [default: true] Allow the reception of beacon frames */
	bool accept_data_frames;                 /* [default: true] Allow the reception of data frames */
	bool accept_ack_frames;                  /* [default: true] Allow the reception of acknowlegment frames */
	bool accept_mac_command_frames;          /* [default: true] Allow the reception of MAC-command frames */
	bool accept_unspecified_frames;          /* [default: true] Allow the reception of frame types 4 - 7 (unspecified/reserved) -> Warning: No further frame decoding will be done */
	bool accept_type4_frames;                /* [default: true] Allow the reception of frame type 4 (unspecified/reserved) -> Warning: No further frame decoding will be done */
	bool accept_type5_frames;                /* [default: true] Allow the reception of frame type 5 (unspecified/reserved) -> Warning: No further frame decoding will be done */

	bool host_interrupt_polarity;            /* Polarity of interrupt line default: true (recomended) */
	bool spi_launch_edge_is_sampling;         /* True: use the sampling edge for SPI launch edge / false: use the opposite edge */
	bool disable_frame_check_error_handling; /* Ignore errors appearing at frame checking process */
	bool disable_rx_double_buffering;        /* Disable the receiver side double buffering feature -> default: true */
	bool ignore_phr_errors;                  /* Ignore PHR errors and do not abort reception */
	bool ignore_rsd_errors;                  /* ignores all errors in Reed Salomon processing and therefor does not abort the reception default: false (recommended)*/
	/* PHR mode gets automatically set to extended if max frame length is larger then 127 */
	/* SmartTx power control */
	/* 110kbps mode gets enabled based on the data rate set */
	/* TODO: Receiver Wait Timeout Enable */
	/* Receiver auto reenable gets taken from above */
	bool enable_auto_acknowledgement;        /* Only works in combination with frame filtering -> sends automatic ack frame if activated */
	/* TODO: AACKPEND */

};


extern int GetBits(int bitmask, int inclusive_start, int inclusive_end); /* Bit 0 to 31 */
extern int GetBit(int bitmask, int bit_number); /* Bit 0 to 31 */
extern int GenerateFrameControlField(int mac_frame_type, bool ack_request, int destination_address_type, bool old_standard, int source_address_type); /* TODO: Assure the following! Please refer to the defines given above old standard = IEEE 802.15.4 / not old IEEE 802.15.4-2003 */


#endif
