#include "dw1000.h"

DW1000::DW1000(SRadioConfig *_config)
{
	/* Register values */
	int pan_id_and_address;
	int pac_size;
	int agc_ctrl1;
	int agc_tune1;
	int digi_tune_0b;
	int digi_tune_1a;
	int digi_tune_1b;
	int digi_tune_2;
	int digi_tune_4h;
	int sfd_detection_timeout;
	int preamble_detection_timeout;
	int receiver_channel_control;
	int transmitter_channel_control;
	int pulse_generator_delay;
	int pll_configuration;
	int pll_tune;
	int lde_algorithm_threshold_factor;
	int timeunits_antenna_delay_receiver;
	int timeunits_antenna_delay_transmitter;
	int lde_configuration_2;
	int lde_replica_coefficent_config;
	int transmit_power_configuration;

	int rx_tx_channel_number;
	int deca_sfd_enable_status;
	int receiver_standard_sfd_status;
	int transmitter_standard_sfd_status;
	int pulse_repetition_config;
	int preamble_code;
	int channel_control_configuration;

	int transmit_bit_rate_configuration;
	int transmit_prf_configuration;
	int preamble_symbol_repetition_configuration;
	int frame_control_configuration;
	int frame_filtering_bitmask;
	int sysconfig_bitmask;

	config = _config;

	spi = mraa_spi_init(5);
	mraa_spi_frequency(spi, DW1000_SPI_CLOCK_SPEED);

	cs = mraa_gpio_init(DW_CS);
	mraa_gpio_dir(cs, MRAA_GPIO_OUT);
	mraa_gpio_write(cs, 0x1);

	ResetAll();                         /* We do a soft reset of the DW1000 everytime the driver starts */

	/* Enable leds if defined */
	if(config->enable_leds)
	{
		/* Set up MFIO for LED output */
		Write16BitToRegister(DW1000_GPIO_CTRL, 0x00, 0x1540); /* Set GPIO LED0/1/2/3 */
		Write8BitToRegister(DW1000_PMSC, 0x02, 0xbf);         /* Enable clocks       */
		Write16BitToRegister(DW1000_PMSC, 0x28, 0x0110);      /* Enable blink        */
	}


	/* *************** Interpreting user configuration into register values *************** */

	pan_id_and_address = ((config->pan_id & 0xFFFF) << 16) | (config->address & 0xFFFF);

	rx_tx_channel_number = config->channel | config->channel << 4;
	if(config->sfd_style == 0)
	{
		deca_sfd_enable_status = 1;
		receiver_standard_sfd_status = 0;
		transmitter_standard_sfd_status = 0;
	}
	else if(config->sfd_style == 1)
	{
		deca_sfd_enable_status = 0;
		receiver_standard_sfd_status = 1;
		transmitter_standard_sfd_status = 1;
	}
	else
	{
		deca_sfd_enable_status = 0;
		receiver_standard_sfd_status = 0;
		transmitter_standard_sfd_status = 0;
	}

	if(config->low_pulse_repetition_frequency)
		pulse_repetition_config = 1;
	else
		pulse_repetition_config = 3;

	if(config->low_pulse_repetition_frequency)
	{
		if(config->channel == 1)
			preamble_code = 2;
		else if(config->channel == 2)
			preamble_code = 4;
		else if(config->channel == 3)
			preamble_code = 6;
		else if(config->channel == 4)
			preamble_code = 8;
		else if(config->channel == 5)
			preamble_code = 4;
		else if(config->channel == 7)
			preamble_code = 8;
		else
		{
			Debug(("Warning: No recommended preamble code for this channel\n"));
		}
	}
	else
	{
		if(config->channel == 1)
			preamble_code = 11;
		else if(config->channel == 2)
			preamble_code = 11;
		else if(config->channel == 3)
			preamble_code = 11;
		else if(config->channel == 4)
			preamble_code = 19;
		else if(config->channel == 5)
			preamble_code = 11;
		else if(config->channel == 7)
			preamble_code = 19;
		else
		{
			Debug(("Warning: No recommended preamble code for this channel\n"));
		}
	}

	/* ATTENTION here ! */
	channel_control_configuration = rx_tx_channel_number | (deca_sfd_enable_status << 17) | (pulse_repetition_config << 18) | (transmitter_standard_sfd_status << 20) | (receiver_standard_sfd_status << 21) | (preamble_code << 22) | (preamble_code << 27);

	/* Setting to energy_scan_mode */
	if(config->energy_scan_mode)
		agc_ctrl1 = 0x0;
	else
		agc_ctrl1 = 0x1;

	/* Setting agc_tune1 */
	if(config->low_pulse_repetition_frequency)
		agc_tune1 = 0x8870;
	else
		agc_tune1 = 0x889B;

	/* Setting digi_tune_0b */
	if(config->sfd_style == 1) /* IEEE style */
	{
			if(config->data_rate == 0) /* 110 kbps */
				digi_tune_0b = 0x000A;
			else if(config->data_rate == 1 || config->data_rate == 2)
				digi_tune_0b = 0x0001;
	}
	else /* DW and custom style */
	{
			if(config->data_rate == 0) /* 110 kbps */
				digi_tune_0b = 0x0016;
			else if(config->data_rate == 1) /* 850 kbps */
				digi_tune_0b = 0x0006;
			else if(config->data_rate == 2) /* 6.8 Mpbps */
				digi_tune_0b = 0x0002;
	}

	/* Setting digi_tune_1a */
	if(config->low_pulse_repetition_frequency)
		digi_tune_1a = 0x0087;
	else
		digi_tune_1a = 0x008D;

	/* Setting digi_tune_1b */
	if(config->preamble_length == 64)
		digi_tune_1b = 0x0010;
	else if(config->preamble_length <= 1024)
		digi_tune_1b = 0x0020;
	else if(config->preamble_length > 1024)
		digi_tune_1b = 0x0064;


	/* Selecting PAC size */
	/* We assume the received preamble_length is the same as configured localy */
	if(config->preamble_length <= 128)
		pac_size = 8;
	else if(config->preamble_length <= 512)
		pac_size = 16;
	else if(config->preamble_length == 1024)
		pac_size = 32;
	else
		pac_size = 64;

	/* Setting digi_tune_2 */
	if(config->low_pulse_repetition_frequency) /* 16 Mhz */
	{
		if(pac_size == 8)
			digi_tune_2 = 0x311A002D;
		else if(pac_size == 16)
			digi_tune_2 = 0x331A0052;
		else if(pac_size == 32)
			digi_tune_2 = 0x351A009A;
		else if(pac_size == 64)
			digi_tune_2 = 0x371A011D;
	}
	else /* 64 Mhz */
	{
		if(pac_size == 8)
			digi_tune_2 = 0x313B006B;
		else if(pac_size == 16)
			digi_tune_2 = 0x333B00BE;
		else if(pac_size == 32)
			digi_tune_2 = 0x353B015E;
		else if(pac_size == 64)
			digi_tune_2 = 0x373B0296;
	}


	/* Setting digi_tune_4h */
	/* We assume the received preamble_length is the same as configured localy */
	if(config->preamble_length == 64)
		digi_tune_4h = 0x0010;
	else
		digi_tune_4h = 0x0028; /* 128 or greater */


	sfd_detection_timeout = config->sfd_detection_timeout;
	preamble_detection_timeout = config->preamble_detection_timeout;


	/* Setting receiver_channel_control */
	if(config->channel == 4 || config->channel == 7)
	{
		receiver_channel_control = 0xBC;
	}
	else
	{
		if(config->channel == 6 || config->channel > 7)
			Debug(("Warning: Cant recommend a receiver configuration for this channel\n"));
		receiver_channel_control = 0xD8;
	}

	/* Setting transmitter_channel_control */
	if(config->channel == 1)
		transmitter_channel_control = 0x00005C40;
	else if(config->channel == 2)
		transmitter_channel_control = 0x00045CA0;
	else if(config->channel == 3)
		transmitter_channel_control = 0x00086CC0;
	else if(config->channel == 4)
		transmitter_channel_control = 0x00045C80;
	else if(config->channel == 5)
		transmitter_channel_control = 0x001E3FE0;
	else if(config->channel == 7)
		transmitter_channel_control = 0x001E7DE0;
	else
	{
		Debug(("Warning: Cant recommend a transmitter configuration for this channel\n"));
		transmitter_channel_control = 0x001E7DE0;
	}

	/* Setting pulse_generator_delay */
	if(config->channel == 1)
		pulse_generator_delay = 0xC9;
	else if(config->channel == 2)
		pulse_generator_delay = 0xC2;
	else if(config->channel == 3)
		pulse_generator_delay = 0xC5;
	else if(config->channel == 4)
		pulse_generator_delay = 0x95;
	else if(config->channel == 5)
		pulse_generator_delay = 0xC0;
	else if(config->channel == 7)
		pulse_generator_delay = 0x93;
	else
	{
		Debug(("Warning: Cant recommend a pulse delay for this channel\n"));
		pulse_generator_delay = 0x93;
	}

	/* Setting pll_configuration */
	if(config->channel == 1)
		pll_configuration = 0x09000407;
	else if(config->channel == 2 || config->channel == 4)
		pll_configuration = 0x08400508;
	else if(config->channel == 3)
		pll_configuration = 0x08401009;
	else if(config->channel == 5 || config->channel == 7)
		pll_configuration = 0x0800041D;
	else
	{
		Debug(("Warning: Cant recommend a pll configuration for this channel\n"));
		pll_configuration = 0x0800041D;
	}

	/* Setting pll_tune*/
	if(config->channel == 1)
		pll_tune = 0x1E;
	else if(config->channel == 2 || config->channel == 4)
		pll_tune = 0x26;
	else if(config->channel == 3)
		pll_tune = 0x5E;
	else if(config->channel == 5 || config->channel == 7)
		pll_tune = 0xBE;
	else
	{
		Debug(("Warning: Cant recommend a pll tuning for this channel\n"));
		pll_tune = 0xBE;
	}

	/* Setting lde_algorithm_threshold_factor */
	if(config->lde_noise_peak_multiplier > 7)
		Debug(("Warning: LDE peak multiplier is larger then possible!\n"));
	if(config->lde_noise_level_multiplier > 31)
		Debug(("Warning: LDE level multiplier is larger then possible!\n"));
	lde_algorithm_threshold_factor = (config->lde_noise_peak_multiplier * 32) | config->lde_noise_level_multiplier;

	/* Setting lde_configuration_2 */
	if(config->low_pulse_repetition_frequency)
		lde_configuration_2 = 0x1607;
	else
		lde_configuration_2 = 0x0607;

	/* Setting timeunits_antenna_delay_receiver */
	timeunits_antenna_delay_receiver = config->antenna_delay_receiver * US_TO_TIMEUNITS;
	/* Setting timeunits_antenna_delay_transmitter */
	timeunits_antenna_delay_transmitter = config->antenna_delay_transmitter * US_TO_TIMEUNITS;

	/* Setting lde_replica_coefficent_config */
	if(preamble_code == 1 || preamble_code == 2)
		lde_replica_coefficent_config = 0x5998;
	else if(preamble_code == 3)
		lde_replica_coefficent_config = 0x51EA;
	else if(preamble_code == 4)
		lde_replica_coefficent_config = 0x428E;
	else if(preamble_code == 5)
		lde_replica_coefficent_config = 0x451E;
	else if(preamble_code == 6)
		lde_replica_coefficent_config = 0x2E14;
	else if(preamble_code == 7)
		lde_replica_coefficent_config = 0x8000;
	else if(preamble_code == 8)
		lde_replica_coefficent_config = 0x51EA;
	else if(preamble_code == 9)
		lde_replica_coefficent_config = 0x28F4;
	else if(preamble_code == 10)
		lde_replica_coefficent_config = 0x3332;
	else if(preamble_code == 11)
		lde_replica_coefficent_config = 0x3AE0;
	else if(preamble_code == 12)
		lde_replica_coefficent_config = 0x3D70;
	else if(preamble_code == 13)
		lde_replica_coefficent_config = 0x3AE0;
	else if(preamble_code == 14)
		lde_replica_coefficent_config = 0x35C2;
	else if(preamble_code == 15)
		lde_replica_coefficent_config = 0x2B84;
	else if(preamble_code == 16)
		lde_replica_coefficent_config = 0x35C2;
	else if(preamble_code == 17)
		lde_replica_coefficent_config = 0x3332;
	else if(preamble_code == 18)
		lde_replica_coefficent_config = 0x35C2;
	else if(preamble_code == 19)
		lde_replica_coefficent_config = 0x35C2;
	else if(preamble_code == 20)
		lde_replica_coefficent_config = 0x47AE;
	else if(preamble_code == 21)
		lde_replica_coefficent_config = 0x3AE0;
	else if(preamble_code == 22)
		lde_replica_coefficent_config = 0x3850;
	else if(preamble_code == 23)
		lde_replica_coefficent_config = 0x30A2;
	else if(preamble_code == 24)
		lde_replica_coefficent_config = 0x3850;
	else
	{
		Debug(("Warning: Can not recommend LDE replica avoidance coeficent for this preamble_code\n"));
		lde_replica_coefficent_config = 0x3850;
	}

	if(config->data_rate == 0)
		lde_replica_coefficent_config /= 8;

	/* Setting transmit_power_configuration */
	if(config->channel == 6 || config->channel > 7 || config->channel < 1)
			Debug(("Warning: Invalid channel number detected while calculating tx_power\n"));

	if(config->smart_tx_power_enable)
	{
		if(config->low_pulse_repetition_frequency) /* 16Mhz PRF */
		{
				if(config->channel == 1 || config->channel == 2)
					transmit_power_configuration = 0x15355575;
				else if(config->channel == 3)
					transmit_power_configuration = 0x0F2F4F6F;
				else if(config->channel == 4)
					transmit_power_configuration = 0x1F1F3F5F;
				else if(config->channel == 5)
					transmit_power_configuration = 0x0E082848;
				else
					transmit_power_configuration = 0x32527292;
		}
		else /* 64Mhz PRF */
		{
				if(config->channel == 1 || config->channel == 2)
					transmit_power_configuration = 0x07274767;
				else if(config->channel == 3)
					transmit_power_configuration = 0x2B4B6B8B;
				else if(config->channel == 4)
					transmit_power_configuration = 0x3A5A7A9A;
				else if(config->channel == 5)
					transmit_power_configuration = 0x25456585;
				else
					transmit_power_configuration = 0x5171B1D1;
		}
	}
	else
	{
		if(config->low_pulse_repetition_frequency) /* 16Mhz PRF */
		{
				if(config->channel == 1 || config->channel == 2)
					transmit_power_configuration = 0x75757575;
				else if(config->channel == 3)
					transmit_power_configuration = 0x6F6F6F6F ;
				else if(config->channel == 4)
					transmit_power_configuration = 0x5F5F5F5F ;
				else if(config->channel == 5)
					transmit_power_configuration = 0x48484848 ;
				else
					transmit_power_configuration = 0x92929292 ;
		}
		else /* 64Mhz PRF */
		{
				if(config->channel == 1 || config->channel == 2)
					transmit_power_configuration = 0x67676767;
				else if(config->channel == 3)
					transmit_power_configuration = 0x8B8B8B8B;
				else if(config->channel == 4)
					transmit_power_configuration = 0x9A9A9A9A;
				else if(config->channel == 5)
					transmit_power_configuration = 0x85858585;
				else
					transmit_power_configuration = 0xD1D1D1D1;
		}
	}


	/* Setting frame_control_configuration */
	if(config->data_rate == 0)
		transmit_bit_rate_configuration = 0;
	else if(config->data_rate == 1)
		transmit_bit_rate_configuration = 1;
	else
		transmit_bit_rate_configuration = 2;

	if(config->low_pulse_repetition_frequency)
		transmit_prf_configuration = 1;
	else
		transmit_prf_configuration = 2;

	if(config->preamble_length == 64)
		preamble_symbol_repetition_configuration = 1; /* 0001 */
	else if(config->preamble_length == 128)
		preamble_symbol_repetition_configuration = 5; /* 0101 */
	else if(config->preamble_length == 256)
		preamble_symbol_repetition_configuration = 9; /* 1001 */
	else if(config->preamble_length == 512)
		preamble_symbol_repetition_configuration = 13; /* 1101 */
	else if(config->preamble_length == 1024)
		preamble_symbol_repetition_configuration = 2; /* 0010 */
	else if(config->preamble_length == 1536)
		preamble_symbol_repetition_configuration = 6; /* 0110 */
	else if(config->preamble_length == 2048)
		preamble_symbol_repetition_configuration = 10; /* 1010 */
	else if(config->preamble_length == 4096)
		preamble_symbol_repetition_configuration = 3; /* 0011 */


	if(config->max_frame_length  > 127)
	{
		Debug(("Info: Enabling extended PHR mode\n"));
		long_frame_mode_enabled = true;
	}
	else
	{
		long_frame_mode_enabled = false;
	}

	/* Attention here ! */
	frame_control_configuration = (config->max_frame_length & 1023) | (transmit_bit_rate_configuration << 13) | (transmit_prf_configuration << 16) | (preamble_symbol_repetition_configuration << 18);

	/* Setting bitmask for frame filtering functionality */
	frame_filtering_bitmask = config->enable_frame_filtering | (config->accept_all_matching_source_pans << 1) | (config->accept_beacon_frames << 2) | (config->accept_data_frames << 3) | (config->accept_ack_frames << 4) | (config->accept_mac_command_frames << 5) | (config->accept_unspecified_frames << 6) | (config->accept_type4_frames << 7) | (config->accept_type5_frames << 8);
	sysconfig_bitmask = frame_filtering_bitmask | (config->host_interrupt_polarity << 9) |
																(config->spi_launch_edge_is_sampling << 10) |
																(config->disable_frame_check_error_handling << 11) |
																(config->disable_rx_double_buffering << 12) |
																(config->ignore_phr_errors << 13) |
																(config->ignore_rsd_errors << 14) |
																((config->max_frame_length > 127 ? 3 : 0) << 16) |
																((config->smart_tx_power_enable ? 0 : 1) << 18) |
																((config->data_rate == 0 ? 1 : 0) << 22) |
																(config->receiver_auto_reenable << 29) |
																(config->enable_auto_acknowledgement << 30);

	/* TODO SFD Settings (we stay with the default for now) */
	/* TODO have a look at DRX_SFDTOC                       */


	/* ********* Applying the configuration ********* */

	/* Apply Automatic Gain Control configuration */
	Write16BitToRegister(DW1000_AGC_CTRL, 0x02, agc_ctrl1);  /* Contains DIS_AM (disable agc measurement) byte [default: active] */
	Write16BitToRegister(DW1000_AGC_CTRL, 0x04, agc_tune1);  /* AGC_TUNE1              */
	Write32BitToRegister(DW1000_AGC_CTRL, 0x0C, 0x2502A907); /* AGC_TUNE2 STRICT GIVEN */
	Write16BitToRegister(DW1000_AGC_CTRL, 0x12, 0x0035);     /* AGC_TUNE3 STRICT GIVEN */
	/* FIX used to be 0x0055 but why ? */

	/* Apply digital tuner configuration */
	Write16BitToRegister(DW1000_DRX_CONF, 0x02, digi_tune_0b); /* DRX_TUNE0b */
	Write16BitToRegister(DW1000_DRX_CONF, 0x04, digi_tune_1a);  /* DRX_TUNE1a */
	Write16BitToRegister(DW1000_DRX_CONF, 0x06, digi_tune_1b);  /* DRX_TUNE1b */
	Write32BitToRegister(DW1000_DRX_CONF, 0x08, digi_tune_2);  /* DRX_TUNE2  */
	Write16BitToRegister(DW1000_DRX_CONF, 0x26, digi_tune_4h); /* DRX_TUNE4H */

	/* RX/TX channel related settings */
	Write8BitToRegister(DW1000_RF_CONF,  0x0B, receiver_channel_control);     /* RF_RXCTRLH */
	Write32BitToRegister(DW1000_RF_CONF, 0x0C, transmitter_channel_control); /* RF_TXCTRL  */

	Write8BitToRegister(DW1000_TX_CAL,   0x0B, pulse_generator_delay); /* TC_PGDELAY */
	Write32BitToRegister(DW1000_FS_CTRL, 0x07, pll_configuration);     /* FS_PLLCFG  */
	Write8BitToRegister(DW1000_FS_CTRL,  0x0B, pll_tune);              /* FS_PLLTUNE */

	/* Apply detection timeouts */
	/* ATTENTION: NEVER SET TO ZERO ! */
	if(sfd_detection_timeout)
		Write16BitToRegister(DW1000_DRX_CONF, 0x20, sfd_detection_timeout);   /* DRX_SFDTOC */
	Write16BitToRegister(DW1000_DRX_CONF, 0x24, preamble_detection_timeout); /* DRX_PRETOC */

	Write8BitToRegister(DW1000_LDE_CTRL,  0x0806, lde_algorithm_threshold_factor);    /* LDE_CFG1   */
	Write16BitToRegister(DW1000_LDE_CTRL, 0x1806, lde_configuration_2);              /* LDE_CFG2   */

	/* Apply lde antenna delays TODO: not preserved after sleep! */
	Write16BitToRegister(DW1000_LDE_CTRL, 0x1804, timeunits_antenna_delay_receiver); /* LDE_RXANTD            */
	Write16BitToRegister(DW1000_TX_ANTD,  0, timeunits_antenna_delay_transmitter);    /* TX_ANTD before: 16384 */

	/* At too large clock offset between the two devices there can be replicas of the leading edge in the path -> this corrects it based on the clockskew and RX_PCODE */
	Write16BitToRegister(DW1000_LDE_CTRL, 0x2804, lde_replica_coefficent_config);    /* LDE_REPC  */

	/* Apply power settings */
	Write32BitToRegister(DW1000_TX_POWER, 0, transmit_power_configuration);          /* TX_POWER  */

	/* Apply channel settings */
	Write32BitToRegister(DW1000_CHAN_CTRL, 0, channel_control_configuration);        /* CHAN_CTRL */
	Write32BitToRegister(DW1000_TX_FCTRL,  0, frame_control_configuration);          /* TX_FCTRL  */

	/* Apply system configuration */
	Write32BitToRegister(DW1000_SYS_CFG, 0, sysconfig_bitmask);

	/* Set local PAN_ID and address */
	Write32BitToRegister(DW1000_PANADR, 0, pan_id_and_address);

	// if(config->data_rate == 0 || config->smart_tx_power_enable)
	// 	Write8BitToRegister(DW1000_SYS_CFG, 2, (config->data_rate == 0 ? 64 : 0) | (config->smart_tx_power_enable ? 0 : 4));  /* RF_RXCTRLH set to 0b1000100 (no smartTxPower and 110kbps) */
	// if(config->receiver_auto_reenable)
	// 	Write8BitToRegister(DW1000_SYS_CFG, 3, 0x20);

	if(config->enable_event_counter_diagnostics)
		Write16BitToRegister(DW1000_DIGI_DIAGNOSTICS, 0, 1); /* Attention: has to be 16 bits minimum write referring to manual EVC_CTRL */

	/* TODO: on request sleep mode (ATXSLP, AON, PMSC_SNOZ) */
	/* TODO: look at AON_WCFG */
	/* TODO: for production: put EUI/serial number into OTP memory (Careful!)*/


	LoadLDE(); /* Important: everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned off or there's receiving malfunction see User Manual LDELOAD on p22 & p158 */



}


uint32_t DW1000::GetDeviceID()
{
	uint32_t result;
	/* GetDeviceID! */
	ReadFromRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
	return result;
}


uint64_t DW1000::GetEUI()
{
	uint64_t result;
	ReadFromRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
	return result;
}

void DW1000::SetEUI(uint64_t EUI)
{
	WriteToRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}


float DW1000::GetVoltage()
{
	float voltage;

	uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};   /* Algorithm form User Manual p57 */
	WriteToRegister(DW1000_RF_CONF, 0x11, buffer, 2);
	WriteToRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
	WriteToRegister(DW1000_TX_CAL,  0x00, &buffer[3], 1);
	WriteToRegister(DW1000_TX_CAL,  0x00, &buffer[4], 1);
	ReadFromRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);
	voltage = buffer[5] * 0.0057 + 2.3;
	return voltage;
}


uint64_t DW1000::GetStatus()
{
	return Read40BitFromRegister(DW1000_SYS_STATUS, 0);
}


uint64_t DW1000::GetRXTimestamp()
{
	uint64_t temp = Read40BitFromRegister(DW1000_RX_TIME, 0);
	return temp;
}


uint64_t DW1000::GetTXTimestamp()
{
	return Read40BitFromRegister(DW1000_TX_TIME, 0);
}


void DW1000::SendString(char* message)
{
	SendFrame((uint8_t*)message, strlen(message)+1);
}


void DW1000::ReceiveString(char* message)
{
	ReadFromRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, GetFramelength());  // get data from buffer
}


void DW1000::SendDelayedFrame(uint8_t* message, uint16_t length, uint64_t tx_timestamp)
{
	Write40BitToRegister(DW1000_DX_TIME, 0, tx_timestamp);
	SendFrame(message, length, true);
}


void DW1000::SendFrame(uint8_t* message, uint16_t length, bool delayed)
{
	// printf("Message debug print (octetwise):\n");
	// for(int i = 0; i < length; i++)
	// {
	// 	printf("%d: 0x%x\n", i, message[i]);
	// }
	// printf("\n");

	//printf("0x%x\n", message[0]);
	length += 2; /* -2 (CRC bytes) */

	if(length >= config->max_frame_length)
	{
		Debug(("Warning: Message for transmission too large -> cutting down to max length\n"));
		length = config->max_frame_length;
	}

	if(length > 1023)
	{
		Debug(("Error: Invalid frame length -> Discarding message\n"));
		return;
	}

	if(length > 127 && !long_frame_mode_enabled)
	{
		Debug(("Warning: Not standard conform frame length in use -> Enable PHR_mode!\n"));
		exit(0);
		// Write8BitToRegister(DW1000_SYS_CFG, 2, (config->data_rate == 0 ? 64 : 0) | (config->smart_tx_power_enable ? 0 : 4) | 6); /* RF_RXCTRLH set to 0b00000110 */
	}
	else if(long_frame_mode_enabled)
	{
		Debug(("Info: Unnecessary large package size enabled\n"));
		/* cant be right 15 ? */
		// Write8BitToRegister(DW1000_SYS_CFG, 15, (config->data_rate == 0 ? 64 : 0) | (config->smart_tx_power_enable ? 0 : 4));    /* RF_RXCTRLH */
	}

	WriteToRegister(DW1000_TX_BUFFER, 0, message, length - 2);

	Debug(("Info: Sending message with length: %d\n", length));
	uint8_t backup = Read8BitFromRegister(DW1000_TX_FCTRL, 1);
	length = ((backup & 0xFC) << 8) | (length & 0x03FF); /* Restore previous settings */
	Write16BitToRegister(DW1000_TX_FCTRL, 0, length);


	/* TODO is this necessary ? */
	StopTRX();
	Write8BitToRegister(DW1000_SYS_CTRL, 0, 0x02 | (delayed ? 0x04 : 0x0));

	// if(!config->receiver_auto_reenable)
	StartRX();

	/* TODO: */
	//should really check if TXPUTE or HPDWARN bits are set... as per our EVK code/driver
	//why are you starting RX here? why not use WAIT4RESP - this is what this bit is for?
}


void DW1000::StartRX()
{
	Write8BitToRegister(DW1000_SYS_CTRL, 1, 0x01);
}


void DW1000::StopTRX()
{
	Write8BitToRegister(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}


void DW1000::LoadLDE()
{
	/* TODO: Is this correct ? */
	Write16BitToRegister(DW1000_PMSC, 0, 0x0301);      /* Set clock momentary to XTAL so OTP is reliable ? */
	Write16BitToRegister(DW1000_OTP_IF, 0x06, 0x8000); /* LDELOAD bit to on */
	usleep(150);
	Write16BitToRegister(DW1000_PMSC, 0, 0x0200);      /* Switch back to PLL clock */
}


void DW1000::ResetRX()
{
	Write8BitToRegister(DW1000_PMSC, 3, 0xE0);   /* Set RX reset   */
	Write8BitToRegister(DW1000_PMSC, 3, 0xF0);   /* Clear RX reset */
}


void DW1000::ResetAll()
{
	Write8BitToRegister(DW1000_PMSC, 0, 0x01);   /* Set clock to XTAL    */
	Write8BitToRegister(DW1000_PMSC, 3, 0x00);   /* Set all reset        */
	usleep(10);                                  /* Wait for PLL to lock */
	Write8BitToRegister(DW1000_PMSC, 3, 0xF0);   /* Clear all reset      */
}


void DW1000::SetInterrupt(bool RX, bool TX)
{
	Write16BitToRegister(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  /* RX good frame 0x4000, TX done 0x0080 */
}


uint16_t DW1000::GetFramelength()
{
	uint16_t framelength = Read16BitFromRegister(DW1000_RX_FINFO, 0);
	framelength = (framelength & 0x03FF) - 2;                         /* Take only the right bits and subtract the 2 CRC Bytes */
	return framelength;
}


/* ******************** SPI Interface ******************** */
uint8_t DW1000::Read8BitFromRegister(uint8_t reg, uint16_t subaddress)
{
	uint8_t result;
	ReadFromRegister(reg, subaddress, &result, 1);
	return result;
}


uint16_t DW1000::Read16BitFromRegister(uint8_t reg, uint16_t subaddress)
{
	uint16_t result;
	ReadFromRegister(reg, subaddress, (uint8_t*)&result, 2);
	return result;
}


uint64_t DW1000::Read40BitFromRegister(uint8_t reg, uint16_t subaddress)
{
	uint64_t result;
	ReadFromRegister(reg, subaddress, (uint8_t*)&result, 5);
	result &= 0xFFFFFFFFFF;                                 // only 40-Bit
	return result;
}


void DW1000::Write8BitToRegister(uint8_t reg, uint16_t subaddress, uint8_t buffer)
{
	WriteToRegister(reg, subaddress, &buffer, 1);
}


void DW1000::Write16BitToRegister(uint8_t reg, uint16_t subaddress, uint16_t buffer)
{
	WriteToRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}


void DW1000::Write32BitToRegister(uint8_t reg, uint16_t subaddress, uint32_t buffer)
{
	WriteToRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}


void DW1000::Write40BitToRegister(uint8_t reg, uint16_t subaddress, uint64_t buffer)
{
	WriteToRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}


void DW1000::ReadFromRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{

	uint8_t *request;
	uint8_t *response;
	uint8_t header[3];
	int headerLen = 1;

	if (subaddress > 0) {
		header[0] = reg | DW1000_SUBADDRESS_FLAG;

		if (subaddress > 0x7F) {
			header[1] = (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG;
			header[2] = (uint8_t)(subaddress >> 7);
			headerLen+=2;
		} else {
			header[1] = (uint8_t)subaddress;
			headerLen++;
		}
	} else {
		header[0] = reg;
	}

	request =  (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	response = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	memset(request,  0x00, length + headerLen);
	memset(response, 0x00, length + headerLen);
	memcpy(request, header, headerLen);

	mraa_gpio_write(cs, 0x1);
	mraa_gpio_write(cs, 0x0);
	usleep(100);
	mraa_spi_transfer_buf(spi, request, response, length + headerLen);
	usleep(100);
	mraa_gpio_write(cs, 0x1);

	memcpy (buffer, &response[headerLen], length);

	free(request);
	free(response);
}


void DW1000::WriteToRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{
	uint8_t *packet;
	uint8_t *response;
	uint8_t header[3];
	int headerLen = 1;

	reg |= DW1000_WRITE_FLAG;

	if (subaddress > 0) {
		header[0] = reg | DW1000_SUBADDRESS_FLAG;

		if (subaddress > 0x7F) {
			header[1] = (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG;
			header[2] = (uint8_t)(subaddress >> 7);
			headerLen+=2;
		} else {
			header[1] = (uint8_t)subaddress;
			headerLen++;
		}
	} else {
		header[0] = reg;
	}

	packet = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	response = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	memset (packet, 0x00, length + headerLen);
	memcpy (packet, header, headerLen);
	memcpy (&packet[headerLen], buffer, length);

	mraa_gpio_write(cs, 0x1);
	mraa_gpio_write(cs, 0x0);
	usleep (100);
	mraa_spi_transfer_buf(spi, packet, response, length + headerLen);
	usleep (100);
	mraa_gpio_write(cs, 0x1);

	free(packet);
}


void DW1000::SetupTransaction(uint8_t reg, uint16_t subaddress, bool write)
{
	reg |= (write * DW1000_WRITE_FLAG); /* Set read/write flag */

	if(subaddress > 0) /* There's a subadress, we need to set flag and send second header byte */
	{
		mraa_spi_write(spi, reg | DW1000_SUBADDRESS_FLAG);

		if(subaddress > 0x7F) /* Sub address too long, we need to set flag and send third header byte */
		{
			mraa_spi_write(spi, (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG);
			mraa_spi_write(spi, (uint8_t)(subaddress >> 7));
		}
		else
		{
			mraa_spi_write(spi, (uint8_t)subaddress);
		}

	}
	else
	{
		mraa_spi_write(spi, reg);
	}
}


void DW1000::PrintSystemStatusInformation()
{
	int current_status;

	current_status = GetStatus();

	printf(" +++++++ system status +++++++ \n");

	/* Status of interrupt line IRQS */
	printf(" --- Status of interrupt line: ");
	if(GetBit(current_status, 0))
		printf("active\n");
	else
		printf("inactive\n");

	/* Status of PLL clock CPLOCK */
	printf(" --- Status of pll clock (to be cleared): ");
	if(GetBit(current_status, 1))
		printf("locked\n--- max spi possible/operating at full speed\n");
	else
		printf("unlocked\n");

	/* External Sync Clock Reset ESYNCR */
	printf(" --- External sync event (to be cleared): ");
	if(GetBit(current_status, 2))
		printf("external sync received\n");
	else
		printf("no event\n");

	/* TODO in config: Automatic Acknowledgement ESYNCR */

	/* Transmitter Frame Begins TXFRB */
	printf(" --- Transmitter status: ");
	if(GetBit(current_status, 4))
		printf("transmitting\n");
	else
		printf("idle\n");

	printf(" --- Transmission status:\n");
	if(GetBit(current_status, 5))
		printf("	* preamble sent\n");
	else
		printf("	* sending preamble...\n");
	if(GetBit(current_status, 6))
		printf("	* PHY Header sent\n");
	else
		printf("	* sending PHY header...\n");
	if(GetBit(current_status, 7))
		printf("	* data sent\n");
	else
		printf("	* sending data...\n");

	printf("--- Receiver status:\n");
	if(GetBit(current_status, 8))
		printf("	* preamble detected\n");
	else
		printf("	* scanning for preamble...\n");
	if(GetBit(current_status, 9))
		printf("	* sfd detected\n");
	else
		printf("	* scanning for sfd...\n");
	if(GetBit(current_status, 10))
		printf("	* lde processing done\n");
	else
		printf("	* waiting for lde...\n");
	if(GetBit(current_status, 11))
		printf("	* PHY header detected\n");
	else
		printf("	* scanning for PHY header...\n");
	if(GetBit(current_status, 12))
		printf("	* ERROR: PHY header was invalid\n");
	if(GetBit(current_status, 13))
		printf("	* Frame received\n");
	else
		printf("	* Awaiting frame completion\n");
	if(GetBit(current_status, 14))
		printf("	* CRC is valid (RXFCG)\n");
	else
		printf("	* Error: CRC is invalid(RXFCG)\n");
	/* Second check -> if both keep unset LDE code was not loaded properly */
	if(GetBit(current_status, 15))
		printf("	* Error: CRC is invalid(RXFCE)\n");
	else
		printf("	* CRC is valid (RXFCE)\n");
	if(GetBit(current_status, 16))
		printf("	* Error: Reed Solomon failed\n");
	if(GetBit(current_status, 17))
		printf("	* Warning: Frame timeout\n");
	if(GetBit(current_status, 18))
		printf("	* Error: LDE detection error\n");
	if(GetBit(current_status, 20))
		printf("	* Error: RX buffer overrun\n"); /* RXOVRR */
	if(GetBit(current_status, 18))
		printf("	* Warning: Preamble detection timeout\n");

	/* GPIO Interrupt status GPIOIRQ */
	printf(" --- GPIO Interrupt status: ");
	if(GetBit(current_status, 22))
		printf("active\n");
	else
		printf("no event\n");

	/* SLEEP/DEEPSLEEP to INIT SLP2INIT */
	printf(" --- Wakeup process status (not valid if LDE auto download): ");
	if(GetBit(current_status, 23))
		printf("completed\n");
	else
		printf("unfinished\n");

	/* RF PLL Locking Issue RFPLL_LL */
	if(GetBit(current_status, 24))
		printf("Error: RF PLL clock has locking issues\n");

	/* CLK PLL Locking Issue CLKPLL_LL */
	if(GetBit(current_status, 25))
		printf("Error: Digital PLL clock has locking issues\n");

	/* Receive SFD Timeout RXSFDTO */
	if(GetBit(current_status, 26))
		printf("Warning: SFD timout detected\n");

	/* Half Period Delay Warning - Delayed transmission/receiving causing errors (too large) HDPWARN */
	if(GetBit(current_status, 27))
		printf("Error: Sending/Receiving delway causing half period delay warning\n");

	/* Transmit buffer error TXBERR */
	if(GetBit(current_status, 28))
		printf("Error: while writing to Tx buffer\n");

	/* Frame Filtering Rejection event status AFFREJ */
	if(GetBit(current_status, 27))
		printf("Info: A Frame was rejected\n");

	/* TODO: Frame quality information */
	/* TODO first in config double buffer mode flags */

}

void DW1000::PrintDiagnosticInformation()
{
	int phr_error_count;
	int rsd_error_count;
	int fcg_count;
	int fcb_count;
	int ffr_count;
	int rx_overrun_count;
	int sfd_timeout_count;
	int preamble_detection_timeout_count;
	int rx_frame_timeout_count;
	int tx_frame_sent_count;
	int half_period_warning_count;
	int transmitter_power_warning_count;

	printf(" +++++++ system event counter +++++++ \n");

	if(!config->enable_event_counter_diagnostics)
		printf("Warning: Event counter is not enabled!\n");

	phr_error_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x04), 0, 11);                  /* EVC_PHE  */
	rsd_error_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x06), 0, 11);                  /* EVC_RSE  */
	fcg_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x08), 0, 11);                        /* EVC_FCG  */
	fcb_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x0A), 0, 11);                        /* EVC_FCE  */
	ffr_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x0C), 0, 11);                        /* EVC_FFR  */
	rx_overrun_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x0E), 0, 11);                 /* EVC_OVR  */
	sfd_timeout_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x10), 0, 11);                /* EVC_STO  */
	preamble_detection_timeout_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x12), 0, 11); /* EVC_PTO  */
	rx_frame_timeout_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x14), 0, 11);           /* EVC_FWTO */
	tx_frame_sent_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x16), 0, 11);              /* EVC_TXFS */
	half_period_warning_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x18), 0, 11);        /* EVC_HPW  */
	transmitter_power_warning_count = GetBits(Read16BitFromRegister(DW1000_DIGI_DIAGNOSTICS, 0x1A), 0, 11);  /* EVC_TPW  */ 

	printf("PHR error count: %d\n", phr_error_count);
	printf("Reed Solomon Decoder error count: %d\n", rsd_error_count);
	printf("CRC/FCG Good: %d\n", fcg_count);
	printf("CRC/FCG Bad: %d\n", fcb_count);
	printf("Frame filter rejections count: %d\n", ffr_count);
	printf("RX overrun event count: %d\n", rx_overrun_count);
	printf("SFD timeout event count: %d\n", sfd_timeout_count);
	printf("Preamble detection timeout count: %d\n", preamble_detection_timeout_count);
	printf("RX frame wait timeout count: %d\n", rx_frame_timeout_count);
	printf("TX frames sent: %d\n", tx_frame_sent_count);
	printf("Half period warning count: %d\n", half_period_warning_count);
	printf("transmit power up warning count: %d\n", transmitter_power_warning_count);
}


double DW1000::GiveCalibrationDistance()
{
	if(config->channel == 1)
	{
		if(config->low_pulse_repetition_frequency)
			return 14.75;
		else
			return 9.3;
	}
	else if(config->channel == 2)
	{
		if(config->low_pulse_repetition_frequency)
			return 12.9;
		else
			return 8.14;
	}
	else if(config->channel == 3)
	{
		if(config->low_pulse_repetition_frequency)
			return 11.47;
		else
			return 7.24;
	}
	else if(config->channel == 4)
	{
		return 8.68;
	}
	else if(config->channel == 5)
	{
		if(config->low_pulse_repetition_frequency)
			return 7.94;
		else
			return 5.01;
	}
	else if(config->channel == 7)
	{
		return 5.34;
	}

	printf("Warning: can not recommend a calibration distance for this channel!\n");
	return 0;
}
