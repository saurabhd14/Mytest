#include "utils.h"

bool debug_print_enabled;

STimer::STimer()
{
	start = 0;
	stop = 0;
}


void STimer::Start()
{
	start = clock();
}


void STimer::Stop()
{
	stop = clock();
}


double STimer::Read()
{
	if(start)
		return ((double)(clock() - start)) / (double)CLOCKS_PER_SEC;

	return 0.0;
}

SRadioConfig::SRadioConfig()
{

	enable_frame_filtering = false;
	accept_all_matching_source_pans = false;
	accept_beacon_frames = true;
	accept_data_frames = true;
	accept_ack_frames = true;
	accept_mac_command_frames = true;
	accept_unspecified_frames = true;
	accept_type4_frames = true;
	accept_type5_frames = true;

	host_interrupt_polarity = true;
	spi_launch_edge_is_sampling = false;
	disable_frame_check_error_handling = false;
	disable_rx_double_buffering = true;
	ignore_phr_errors = false;
	ignore_rsd_errors = false;
	enable_auto_acknowledgement = false;
}



int GetBits(int bitmask, int inclusive_start, int inclusive_end)
{
	uint64_t extract_mask;

	if(inclusive_end > 31 || inclusive_end < inclusive_start || inclusive_end < 0 || inclusive_start < 0)
	{
		printf("Attention: Invalid bit offset for bitmask extraction given !\n");
		return 0;
	}

	bitmask = bitmask >> inclusive_start;
	extract_mask = (1 << (inclusive_end - inclusive_start + 1)) - 1;

	return bitmask & extract_mask;
}


int GetBit(int bitmask, int bit_number)
{
	return GetBits(bitmask, bit_number, bit_number);
}


int GenerateFrameControlField(int mac_frame_type, bool ack_request, int destination_address_type, bool old_standard, int source_address_type)
{
	return mac_frame_type | (ack_request << 5) | (destination_address_type << 10) | ((old_standard ? 0x01 : 0x00) << 12) | (source_address_type << 14);
}
