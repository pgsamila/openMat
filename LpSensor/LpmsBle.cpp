/***********************************************************************
** Copyright (C) LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "LpmsBle.h"

#include "BleCmdDef.h"
#include "BleUart.h"

#define MAX_DEVICES 64
#define UART_TIMEOUT 10

int found_devices_count = 0;
bd_addr found_devices[MAX_DEVICES];

std::queue<unsigned char> *bleQueue;
static volatile bool isStopDiscovery = false;

std::mutex bleMutex;
std::mutex rxMutex;

enum actions {
	action_none,
	action_scan,
	action_connect,
	action_info,
};
enum actions action = action_none;

typedef enum {
	state_disconnected,
	state_connecting,
	state_connected,
	state_finding_services,
	state_finding_attributes,
	state_listening_measurements,
	state_finish,
	state_last
} states;

states state = state_disconnected;

const char *state_names[state_last] = {
    "disconnected",
    "connecting",
    "connected",
    "finding_services",
    "finding_attributes",
    "listening_measurements",
    "finish"
};

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE 0xffff

#define LPMS_MEASUREMENT_CONFIG_UUID 0x2902
#define LPMS_SERVICE_UUID 0bd51666-e7cb-469b-8e4d-2742f1ba77cc
#define LPMS_DATA_UUID e7add780-b042-4876-aae1-112855353cc1

uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 lpms_handle_start = 0;
uint16 lpms_handle_end = 0;
uint16 lpms_handle_measurement = 0;
uint16 lpms_handle_configuration = 0;
bd_addr connect_addr;
uint8 k_connection_handle = 0;
uint8 indication_enable_state = 0;

void change_state(states new_state)
{
	state = new_state;
}

int cmp_bdaddr(bd_addr first, bd_addr second)
{
	int i;
	for (i = 0; i < sizeof(bd_addr); i++) {
		if (first.addr[i] != second.addr[i]) return 1;
	}
	return 0;
}

void print_bdaddr(bd_addr bdaddr)
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", bdaddr.addr[5], bdaddr.addr[4], bdaddr.addr[3], bdaddr.addr[2], bdaddr.addr[1], bdaddr.addr[0]);
}

void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
	int i;

	printf("[LPMS-BLE] Incoming packet: ");

	for (i = 0; i < sizeof(*hdr); i++) {
		printf("%02x ", ((unsigned char *)hdr)[i]);
	}
	for (i = 0; i < hdr->lolen; i++) {
		printf("%02x ", data[i]);
	}
	printf("\n");
}

void output(uint8 len1, uint8* data1, uint16 len2, uint8* data2)
{
	if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
		printf("[LPMS-BLE] Writing to serial port failed\n");
	}
}

void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
	uint8 configuration[] = { 0x02, 0x00 };
	ble_cmd_attclient_attribute_write(connection_handle, lpms_handle_configuration, 2, &configuration);
	indication_enable_state = 1;

	printf("[LPMS-BLE] Enabling indications\n");
}

void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
	if (found_devices_count >= MAX_DEVICES) change_state(state_finish);

	int i;
	char *name = NULL;

	for (i = 0; i < found_devices_count; i++) {
		if (!cmp_bdaddr(msg->sender, found_devices[i])) return;
	}
	found_devices_count++;
	memcpy(found_devices[i].addr, msg->sender.addr, sizeof(bd_addr));

	for (i = 0; i < msg->data.len; ) {
		int8 len = msg->data.data[i++];

		if (!len) continue;
		if (i + len > msg->data.len) break;

		uint8 type = msg->data.data[i++];

		switch (type) {
		case 0x09:
			name = (char *) malloc(len);
			memcpy(name, msg->data.data + i, len - 1);
			name[len - 1] = '\0';
		}

		i += len - 1;
	}

	print_bdaddr(msg->sender);
	printf(" RSSI:%u", msg->rssi);

	printf(" Name:");
	if (name) printf("%s", name);
	else printf("Unknown");
	printf("\n");

	free(name);
}

void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
	printf("[LPMS-BLE] Build: %u, protocol_version: %u, hardware: ", msg->build, msg->protocol_version);

	switch (msg->hw) {
	case 0x01: printf("BLE112"); break;
	case 0x02: printf("BLED112"); break;
	default: printf("Unknown");
	}
	printf("\n");

	if (action == action_info) change_state(state_finish);
}

void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
	printf("[LPMS-BLE] ble_evt_connection_status callback\n");

	if (msg->flags & connection_connected) {
		change_state(state_connected);
		printf("[LPMS-BLE] Connected!\n");

		k_connection_handle = msg->connection;

		if (lpms_handle_configuration) {
			change_state(state_listening_measurements);
			enable_indications(msg->connection, lpms_handle_configuration);			
		} else {
			change_state(state_finding_services);
			ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);
		}
	}
}

void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
	int i;

	printf("[LPMS-BLE] ble_evt_attclient_group_found callback\n");

    if (msg->uuid.len < 16) return;
	
	printf("[LPMS-BLE] UUID: ");
	for (i=0; i<msg->uuid.len; ++i) {
		printf("%x ", msg->uuid.data[i]);
	}
	printf("\n");
	
	if (msg->uuid.data[15] == 0x0b &&
		msg->uuid.data[14] == 0xd5 &&
		msg->uuid.data[13] == 0x16 &&
		msg->uuid.data[12] == 0x66 &&
		msg->uuid.data[11] == 0xe7 &&
		msg->uuid.data[10] == 0xcb &&
		msg->uuid.data[9] == 0x46 &&
		msg->uuid.data[8] == 0x9b &&
		msg->uuid.data[7] == 0x8e &&
		msg->uuid.data[6] == 0x4d &&
		msg->uuid.data[5] == 0x27 &&
		msg->uuid.data[4] == 0x42 &&
		msg->uuid.data[3] == 0xf1 &&
		msg->uuid.data[2] == 0xba &&
		msg->uuid.data[1] == 0x77 &&
		msg->uuid.data[0] == 0xcc) {	
		if (state == state_finding_services && lpms_handle_start == 0) {
			lpms_handle_start = msg->start;
			lpms_handle_end = msg->end;
		}
	}
}

bool readyToSend = true;

void ble_rsp_attclient_attribute_write(const struct ble_msg_attclient_attribute_write_rsp_t * msg) {
	if (indication_enable_state == 1) {
		indication_enable_state = 2;
		printf("[LPMS-BLE] Indications enabled. Ready to write/receive.\n");
	}
	
	readyToSend = true;
		
	printf("[LPMS-BLE] Write acknowledged\n");
}

void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
	printf("[LPMS-BLE] ble_evt_attclient_procedure_completed callback\n");

	if (state == state_finding_services) {
		if (lpms_handle_start == 0) {
			printf("[LPMS-BLE] No LPMS service found\n");
			change_state(state_finish);
		} else {
			change_state(state_finding_attributes);
			ble_cmd_attclient_find_information(msg->connection, lpms_handle_start, lpms_handle_end);
		}
	} else if (state == state_finding_attributes) {
		if (lpms_handle_configuration == 0) {
			printf("[LPMS-BLE] No client characteristic configuration found for LPMS service\n");
			change_state(state_finish);
		} else {
			change_state(state_listening_measurements);
			enable_indications(msg->connection, lpms_handle_configuration);
		}
	}
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
	int i;
	
	printf("[LPMS-BLE] ble_evt_attclient_find_information_found callback\n");

	printf("[LPMS-BLE] UUID: ");
	for (i=0; i<msg->uuid.len; ++i) {
		printf("%x ", msg->uuid.data[i]);
	}
	printf("\n");
	
	if (msg->uuid.data[15] == 0xe7 &&
		msg->uuid.data[14] == 0xad &&
		msg->uuid.data[13] == 0xd7 &&
		msg->uuid.data[12] == 0x80 &&
		msg->uuid.data[11] == 0xb0 &&
		msg->uuid.data[10] == 0x42 &&
		msg->uuid.data[9] == 0x48 &&
		msg->uuid.data[8] == 0x76 &&
		msg->uuid.data[7] == 0xaa &&
		msg->uuid.data[6] == 0xe1 &&
		msg->uuid.data[5] == 0x11 &&
		msg->uuid.data[4] == 0x28 &&
		msg->uuid.data[3] == 0x55 &&
		msg->uuid.data[2] == 0x35 &&
		msg->uuid.data[1] == 0x3c &&
		msg->uuid.data[0] == 0xc1) {
		lpms_handle_measurement = msg->chrhandle;
		
		printf("[LPMS-BLE] Measurement handle received\n");	
	} else if (msg->uuid.data[1] == 0x29 && 
		msg->uuid.data[0] == 0x02) {
		lpms_handle_configuration = msg->chrhandle;
		
		printf("[LPMS-BLE] Configuration handle received\n");
	}
}

void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
	rxMutex.lock();
	printf("[LPMS-BLE] ble_evt_attclient_attribute_value callback\n");

	printf("[LPMS-BLE] Data received: ");
	for (unsigned int i=0; i < msg->value.len; i++) {
		printf("%x ", msg->value.data[i]);
		bleQueue->push((unsigned char) msg->value.data[i]);
	}
	printf("\n");
	rxMutex.unlock();
}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
	change_state(state_disconnected);
	printf("[LPMS-BLE] Connection terminated, trying to reconnect\n");
	change_state(state_connecting);
	ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 40, 60, 100,0);
}

LpmsBle::LpmsBle(CalibrationData *configData) :
	LpmsIoInterface(configData)
{
	bleQueue = &this->dataQueue;
}

LpmsBle::~LpmsBle(void)
{
	close();
}

int LpmsBle::read_message(int timeout_ms)
{
	unsigned char data[256];
	struct ble_header hdr;
	int r;

	r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
	
	if (!r) {
		return -1;
	} else if (r < 0) {
		printf("[LPMS-BLE] Reading header failed. Error code:%d\n", r);
		return 1;
	}

	if (hdr.lolen) {
		r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
		if (r <= 0) {
			printf("[LPMS-BLE] Reading data failed. Error code:%d\n", r);
			return 1;
		}
	}

	const struct ble_msg *msg = ble_get_msg_hdr(hdr);

	if (!msg) {
		printf("[LPMS-BLE] Unknown message received\n");
	}

	msg->handler(data);

	return 0;
}

long long LpmsBle::getConnectWait(void) 
{ 
	return 6000000; 
}

void LpmsBle::listDevices(LpmsDeviceList *deviceList)
{
    bglib_output = output;
	MicroMeasure mm;
	int pn;
	std::ostringstream oss;
	std::string cs;
	int l=0;
	char a[256];
	int i;
	
	pn = uart_list_devices();
	if (pn > 0) {	
		oss << pn;
		cs = std::string("COM") + oss.str();			
	} else {
		return;
	}
	
	uart_open(cs.c_str());
	
    ble_cmd_system_reset(0);
    uart_close();
    
	do {
        Sleep(500);
    } while (uart_open(cs.c_str()));
	
	ble_cmd_gap_discover(gap_discover_observation);

	mm.reset();
	while (mm.measure() < 5000000 || isStopDiscovery == true) {
		read_message(UART_TIMEOUT);
	}
	
	for (i=0; i<found_devices_count; ++i) {	
		sprintf(a, "%02x:%02x:%02x:%02x:%02x:%02x", found_devices[i].addr[5], found_devices[i].addr[4], found_devices[i].addr[3], found_devices[i].addr[2], found_devices[i].addr[1], found_devices[i].addr[0]);	
		deviceList->push_back(DeviceListItem(a, DEVICE_LPMS_BLE));
	}
	
	uart_close();
}

void LpmsBle::stopDiscovery(void)
{
	isStopDiscovery = true;	
}

bool LpmsBle::connect(std::string deviceId)
{
	int pn;
	std::ostringstream oss;

	if (isOpen == true) return true;
	
	printf("[LPMS-BLE] Trying to open BG dongle..\n");

    bglib_output = output;	
	
	pn = uart_list_devices();
	if (pn > 0) {	
		oss << pn;
		comStr = std::string("COM") + oss.str();			
	} else {
		return false;
	}
	
	uart_open(comStr.c_str());
	
    ble_cmd_system_reset(0);
    uart_close();
    
	do {
        Sleep(500);
    } while (uart_open(comStr.c_str()));
	
	connect_addr.addr[0] = 0xae;	
	connect_addr.addr[1] = 0x91;	
	connect_addr.addr[2] = 0x68;	
	connect_addr.addr[3] = 0x80;	
	connect_addr.addr[4] = 0x07;	
	connect_addr.addr[5] = 0x00;	
	
	ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 20, 30, 100, 0);
	
	mm.reset();
	oneTx.clear();

	rxState = PACKET_END;
	currentState = IDLE_STATE;
	waitForAck = false;
	waitForData = false;
	ackReceived = false;
	ackTimeout = 0;	
	lpmsStatus = 0;
	configReg = 0;
	dataReceived = false;
	dataTimeout = 0;
	pCount = 0;
	isOpen = false;	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;
	indication_enable_state	= 0;
	
	started = true;
	
	std::thread t(&LpmsBle::run, this);	
	t.detach();
	
	return true;
}

void LpmsBle::close(void)
{
	started = false;
	isOpen = false;
	
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	uart_close();
}

bool LpmsBle::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1014) return false;

	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	txData[2] = (address >> 8) & 0xff;
	txData[3] = function & 0xff;
	txData[4] = (function >> 8) & 0xff;
	txData[5] = length & 0xff;
	txData[6] = (length >> 8) & 0xff;
	
	for (unsigned int i=0; i < length; ++i) {
		txData[7+i] = data[i];
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	for (unsigned int i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[7 + length] = txLrcCheck & 0xff;
	txData[8 + length] = (txLrcCheck >> 8) & 0xff;
	txData[9 + length] = 0x0d;
	txData[10 + length] = 0x0a;
	
	printf("[LPMS-BLE] Writing LPBUS command\n");
	
	bleMutex.lock();
	for (unsigned int i=0; i<11+length; ++i) txQueue.push(txData[i]);
	bleMutex.unlock();
	
	return false;
}

bool LpmsBle::parseModbusByte(void)
{
	unsigned char b;

	rxMutex.lock();
	while (dataQueue.size() > 0) {	
		b = dataQueue.front();
		dataQueue.pop();

		switch (rxState) {
		case PACKET_END:
			if (b == 0x3a) {
				rxState = PACKET_ADDRESS0;
				oneTx.clear();
			}
			break;
			
		case PACKET_ADDRESS0:
			currentAddress = b;
			rxState = PACKET_ADDRESS1;
			break;

		case PACKET_ADDRESS1:
			currentAddress = currentAddress + ((unsigned) b * 256);
			rxState = PACKET_FUNCTION0;
			break;

		case PACKET_FUNCTION0:
			currentFunction = b;
			rxState = PACKET_FUNCTION1;				
			break;

		case PACKET_FUNCTION1:
			currentFunction = currentFunction + ((unsigned) b * 256);
			rxState = PACKET_LENGTH0;			
		break;

		case PACKET_LENGTH0:
			currentLength = b;
			rxState = PACKET_LENGTH1;
		break;
				
		case PACKET_LENGTH1:
			currentLength = currentLength + ((unsigned) b * 256);
			rxState = PACKET_RAW_DATA;
			rawDataIndex = currentLength;
		break;
				
		case PACKET_RAW_DATA:
			if (rawDataIndex == 0) {
				lrcCheck = currentAddress + currentFunction + currentLength;
				for (unsigned i=0; i<oneTx.size(); i++) lrcCheck += oneTx[i];
				lrcReceived = b;
				rxState = PACKET_LRC_CHECK1;
			} else {
				oneTx.push_back(b);
				--rawDataIndex;
			}
			break;
			
		case PACKET_LRC_CHECK1:
			lrcReceived = lrcReceived + ((unsigned) b * 256);			
			if (lrcReceived == lrcCheck) {
				parseFunction();
			} else {
				std::cout << "[LpmsBle] Checksum fail in data packet" << std::endl;
			}
			
			rxState = PACKET_END;
			break;
		
		default:
			rxState = PACKET_END;		
			return false;
			break;
		}
	}
	rxMutex.unlock();
		
	return true;
}

void LpmsBle::run(void)
{
	int l=0;
	unsigned char txData[32];

	while (started == true) {
		read_message(UART_TIMEOUT);
		
		if (indication_enable_state == 2 && isOpen == false) {
			printf("[LPMS-BLE] Connection OPEN\n");
			isOpen = true;
		}
		
		bleMutex.lock();
		if (isOpen == true) {
			if (txQueue.empty() == false && readyToSend == true) {
				if (txQueue.size() < 20) {
					l = txQueue.size();
				} else {
					l = 20;
				}
				
				for (int i=0; i<l; ++i) {
					txData[i] = txQueue.front();
					txQueue.pop();
				}
				
				printf("[LPMS-BLE] Processing queue. %d bytes.\n", l);
				
				ble_cmd_attclient_attribute_write(k_connection_handle, lpms_handle_measurement, l, txData);
				readyToSend = false;
			}
		}
		bleMutex.unlock();
	}
}

bool LpmsBle::pollData(void) 
{	
	return true;
}

bool LpmsBle::deviceStarted(void)
{
	return isOpen;
}