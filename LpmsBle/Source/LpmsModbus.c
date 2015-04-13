/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
***********************************************************************/

#include "LpmsModbus.h"

LpmsPacket rxPacketBuffer[MAX_RX_PACKET_BUFFER];
uint8_t connectedInterface = CANOPEN_CONNECTED;
uint8_t transferFormat = TRANSFER_FORMAT_LPBUS;
uint8_t rxPacketBufferPtr = 0;
uint8_t processedPacketPtr = 0;

uint16_t computeCheckSum(LpmsPacket packet)
{
  	uint16_t checksum = packet.address + packet.function + packet.length;
	if (packet.length != 0) {
		for(uint16_t i = 0; i < packet.length; i++) {
			checksum += packet.data[i];	
		}
	}
	
	return checksum;
}

void addPacketToBuffer(LpmsPacket packet)
{
  	rxPacketBuffer[rxPacketBufferPtr] = packet;
	rxPacketBufferPtr++;
	if (rxPacketBufferPtr >= MAX_RX_PACKET_BUFFER) {
		rxPacketBufferPtr = 0;
	}
}