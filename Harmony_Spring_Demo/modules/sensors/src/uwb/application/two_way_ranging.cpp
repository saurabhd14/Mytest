#include "two_way_ranging.h"

/* Multi Anchor */


//@return true if one complete cycle of ranging all (3) anchors is complete; else false 
bool CTwoWayRanging::requestRangingAll()
{
	static bool anchor_cycle = false;	// keeps a tab on whether all anchors are ranged once
	static uint8_t active = MAX_CONNECTIONS;	//keeps a tab on the current anchor being ranged; might not need this after IRQ is implemented
	if(acknowledgement[active] || (local_timer.Read() >= RANGING_CYCLE_TIMOUT))
	{
		if(anchor_cycle)
		{
			anchor_cycle = false;
			return true;
		}
		active = (active==MAX_CONNECTIONS) ? 1 : (active + 1);
		acknowledgement[active] = false;	//set the ack to false before beginning a ranging cycle
		if(active==MAX_CONNECTIONS)
			anchor_cycle = true;
		//printf("Timing for Ranging message to node %d exceeded -> Retrying...\n", destination);
		local_timer.Start();
		SendPingFrame(active);
	}
	return false;
}





CTwoWayRanging::CTwoWayRanging(DW1000 *_dw, bool _anchor_mode)
{
	dw = _dw;
	anchor_mode = _anchor_mode;
	calibration_mode = false; /* Default -> gets changed in RequestCalibrationValue() if necessary */
	overflow = false;

	for(int i = 0; i < MAX_CONNECTIONS; i++)
		result_stack_size[i] = 0;

	for(int i = 0; i <= MAX_CONNECTIONS; i++)
		acknowledgement[i] = true;

	dw->StartRX();

	/* Just relevant for calibration mode */
	ranging_count = 1;
	sum_ranges = 0;
}


void CTwoWayRanging::RequestRanging(int destination)
{
	if(!acknowledgement[destination])
	{
		if(local_timer.Read() < RANGING_CYCLE_TIMOUT)
			return;
		else
			Debug(("Timing for Ranging message to node %d exceeded -> Retrying...\n", destination));
	}


	Debug(("Ranging node %d...\n", destination));
	local_timer.Start();
	acknowledgement[destination] = false;

	SendPingFrame(destination);
}


bool CTwoWayRanging::RequestCalibrationValue(int destination, double calibration_distance)
{
	static double avg_delay_value = 0.0;
	calibration_mode = true;
	if(!acknowledgement[destination])
	{
		if(local_timer.Read() < RANGING_CYCLE_TIMOUT)
			return false;
		else
			Debug(("Timing for Ranging message to node %d exceeded -> Retrying...\n", destination));
	}


	if(ranging_count <= 1000)
	{
		printf("Sample No. %d\n",ranging_count);
		ranging_count++;
		Debug(("Calibrating with node %d...\n", destination));
		local_timer.Start();
		acknowledgement[destination] = false;

		SendPingFrame(destination);
	}
	else
	{
		Debug(("************** calibration measurement completed **************\n"));
		avg_delay_value += (sum_ranges / 1000 - (calibration_distance * 3.33564 / 1000 * US_TO_TIMEUNITS)) * TIMEUNITS_TO_US;
		if (destination == MAX_CONNECTIONS)
		{
			Debug(("Overall delay (combined anchor and tag) in nanoseconds: %f\nOn a range of %f\n", avg_delay_value/MAX_CONNECTIONS, calibration_distance));
			dw->config->antenna_delay_receiver = dw->config->antenna_delay_transmitter =  avg_delay_value/(MAX_CONNECTIONS); //removed /2 as anchor will have 0 delay and all of it will be put on the tag side
	
			printf("Antenna Delay: %0.8f\n", dw->config->antenna_delay_receiver);
		}
		return true;
	}
	return false;
}


void CTwoWayRanging::CallbackRX()
{
	double t_prop;
	int64_t t_round1;
	int64_t t_reply2;
	int64_t temp_timestamp;

	dw->ReadFromRegister(DW1000_RX_BUFFER, 0, (uint8_t*)&received_frame, dw->GetFramelength());

	switch (received_frame.payload.type)
	{
		case PING:
			Debug(("[RX] PING\n"));
			temp_timestamp = dw->GetRXTimestamp();
			receiver_timestamps[received_frame.payload.source][0] = temp_timestamp;      //Save the first timestamp on the receiving node/anchor (T_rp)
			SendDelayedAnswer(received_frame.payload.source, ANCHOR_RESPONSE, temp_timestamp);
			break;
		case ANCHOR_RESPONSE:
			Debug(("[RX] ANCHOR_RESPONSE\n"));
			temp_timestamp = dw->GetRXTimestamp();
			sender_timestamps[received_frame.payload.source][1] = temp_timestamp;        //Save the second timestamp on the sending node/beacon (T_rr)
			SendDelayedAnswer(received_frame.payload.source, BEACON_RESPONSE, temp_timestamp);
			break;
		case BEACON_RESPONSE:
			Debug(("[RX] BEACON_RESPONSE\n"));
			temp_timestamp = dw->GetRXTimestamp();
			receiver_timestamps[received_frame.payload.source][2] = temp_timestamp;      //Save the third timestamp on the receiving node/anchor (T_rf)
			SendTransferFrame(received_frame.payload.source, (receiver_timestamps[received_frame.payload.source][1] - receiver_timestamps[received_frame.payload.source][0]) & MASK_40BIT , (receiver_timestamps[received_frame.payload.source][2] - receiver_timestamps[received_frame.payload.source][1])& MASK_40BIT);
			break;
		case TRANSFER_FRAME:
			Debug(("[RX] TRANSFER_FRAME\n"));
			t_round1 = (int64_t)(sender_timestamps[received_frame.payload.source][1] - sender_timestamps[received_frame.payload.source][0]) & MASK_40BIT;
			t_reply2 = (int64_t)(sender_timestamps[received_frame.payload.source][2] - sender_timestamps[received_frame.payload.source][1]) & MASK_40BIT;

			/* Following the DecaWave formula for 3-way ranging */
			/* Watch out for overflow */
			t_prop = (double)(t_round1 * received_frame.payload.t_round2 - received_frame.payload.t_reply1 * t_reply2) / (double)(t_round1 + received_frame.payload.t_round2 + received_frame.payload.t_reply1 + t_reply2);


			printf("Ranging result node %d: %f\n", received_frame.payload.source, t_prop * TIMEUNITS_TO_US * 1000 / 3.33564);
			distances[received_frame.payload.source] =  t_prop * TIMEUNITS_TO_US * 1000 / 3.33564;
			// dw->PrintSystemStatusInformation();
			// printf("Voltage: %f\n", dw->GetVoltage());

			/* Just relevant for calibration mode */
			if(calibration_mode)
			{
				printf("Calibration exchange number %d\n", ranging_count);
				sum_ranges += t_prop;
				ranging_count++;
			}

			acknowledgement[received_frame.payload.source] = true;
			break;

		default:
			Debug(("Default\n"));
			break;
	}
}


void CTwoWayRanging::CallbackTX()
{
	switch (ranging_frame.type)
	{
		case PING:
			sender_timestamps[ranging_frame.destination][0] = dw->GetTXTimestamp();    //Save the first timestamp on the sending node/beacon (T_sp)
			Debug(("[TX] Sent PING with TX timestamp(t0): %lld\n", sender_timestamps[ranging_frame.destination][0]));
			break;
		case ANCHOR_RESPONSE:
			receiver_timestamps[ranging_frame.destination][1] = dw->GetTXTimestamp();  //Save the second timestamp on the receiving node/anchor (T_sr)
			Debug(("[TX] Sent ANCHOR_RESPONSE with TX timestamp(t1): %lld\n", receiver_timestamps[ranging_frame.destination][1]));
			break;
		case BEACON_RESPONSE:
			sender_timestamps[ranging_frame.destination][2] = dw->GetTXTimestamp();    //Save the third timestamp on the sending node/beacon (T_sr)

			//this is not to be done here
			//CorrectSenderTimestamps(ranging_frame.destination);                      //Correct the timestamps for the case of a counter overflow
			Debug(("[TX] Sent BEACON_RESPONSE with TX timestamp(t2): %lld\n", sender_timestamps[ranging_frame.destination][2]));
			break;
		default:
			break;
	}

}



void CTwoWayRanging::SendPingFrame(uint8_t destination)
{

	/* Ranging frame packing */
	ranging_frame.type = PING;
	ranging_frame.destination = destination;
	ranging_frame.source = dw->config->address;

	/* MAC frame packing */
	mac_envelop.frame_control = GenerateFrameControlField(MAC_FRAME_TYPE_DATA, false, MAC_SHORT_ADDRESS, true, MAC_SHORT_ADDRESS);
	mac_envelop.sequence_number = 0; /* TODO implement sequential numbers % 256 */
	mac_envelop.dest_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.dest_address = destination;
	mac_envelop.source_pan = 0xFFFF;
	mac_envelop.source_address = dw->config->address;
	mac_envelop.payload = ranging_frame;

	dw->SendFrame((uint8_t*)&mac_envelop, sizeof(mac_envelop));
}


void CTwoWayRanging::SendTransferFrame(uint8_t destination, uint64_t _t_reply1, uint64_t _t_round2)
{
	/* Transfer frame packing */
	transfer_frame.source = dw->config->address;
	transfer_frame.destination = destination;
	transfer_frame.type = TRANSFER_FRAME;
	transfer_frame.t_reply1 = (int64_t) _t_reply1;
	transfer_frame.t_round2 = (int64_t) _t_round2;

	/* MAC frame packing */
	mac_envelop.frame_control = GenerateFrameControlField(MAC_FRAME_TYPE_DATA, false, MAC_SHORT_ADDRESS, true, MAC_SHORT_ADDRESS);
	mac_envelop.sequence_number = 0; /* TODO implement sequential numbers % 256 */
	mac_envelop.dest_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.dest_address = destination;
	mac_envelop.source_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.source_address = dw->config->address;
	mac_envelop.payload = transfer_frame;

	Debug(("Sending TRANSFER_FRAME with \n t_reply : %lld\n t_round2 : %lld\n", transfer_frame.t_reply1, transfer_frame.t_round2));
	dw->SendFrame((uint8_t*)&mac_envelop, sizeof(mac_envelop));
}

void CTwoWayRanging::SendAnswer(uint8_t destination, uint8_t type)
{
	/* Ranging frame packing */
	ranging_frame.source = dw->config->address;
	ranging_frame.destination = destination;
	ranging_frame.type = type;

	/* MAC frame packing */
	mac_envelop.frame_control = GenerateFrameControlField(MAC_FRAME_TYPE_DATA, false, MAC_SHORT_ADDRESS, true, MAC_SHORT_ADDRESS);
	mac_envelop.sequence_number = 0; /* TODO implement sequential numbers % 256 */
	mac_envelop.dest_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.dest_address = destination;
	mac_envelop.source_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.source_address = dw->config->address;
	mac_envelop.payload = ranging_frame;

	dw->SendFrame((uint8_t*)&mac_envelop, sizeof(mac_envelop));
}

void CTwoWayRanging::SendDelayedAnswer(uint8_t destination, uint8_t type, uint64_t rx_timestamp)
{
	uint64_t tx_time;

	/* Ranging frame packing */
	ranging_frame.source = dw->config->address;
	ranging_frame.destination = destination;
	ranging_frame.type = type;

	/* MAC frame packing */
	mac_envelop.frame_control = GenerateFrameControlField(MAC_FRAME_TYPE_DATA, false, MAC_SHORT_ADDRESS, true, MAC_SHORT_ADDRESS);
	mac_envelop.sequence_number = 0; /* TODO implement sequential numbers % 256 */
	mac_envelop.dest_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.dest_address = destination;
	mac_envelop.source_pan = SYSTEM_WIDE_PAN_ID;
	mac_envelop.source_address = dw->config->address;
	mac_envelop.payload = ranging_frame;

	//send frame at delayed time
	//time of sending to program into DW1000 = (rx_timestamp + ANSWER_DELAY_TIMEUNITS) & MASK_40BIT
	//& MASK_40BIT - this will take care of wrapping
	tx_time = (uint64_t)(rx_timestamp + ANSWER_DELAY_TIMEUNITS) & MASK_40BIT;

	dw->SendDelayedFrame((uint8_t*)&mac_envelop, sizeof(mac_envelop), tx_time);
}


void CTwoWayRanging::CorrectReceiverTimestamps(int source)
{
	if(receiver_timestamps[source][0] > receiver_timestamps[source][1])
	{
		receiver_timestamps[source][1] += DECA_TIMER_MAX_VALUE;
		receiver_timestamps[source][2] += DECA_TIMER_MAX_VALUE;
	}

	if(receiver_timestamps[source][1] > receiver_timestamps[source][2])
		receiver_timestamps[source][2] += DECA_TIMER_MAX_VALUE;
}


void CTwoWayRanging::CorrectSenderTimestamps(int source)
{
	if (sender_timestamps[source][0] > sender_timestamps[source][1])
	{
		sender_timestamps[source][1] += DECA_TIMER_MAX_VALUE;
		sender_timestamps[source][2] += DECA_TIMER_MAX_VALUE;
		overflow = true;
	}
	else if (sender_timestamps[source][1] > sender_timestamps[source][2])
	{
		sender_timestamps[source][2] += DECA_TIMER_MAX_VALUE;
		overflow = true;
	}
	else
	{
		overflow = false;
	}
}
