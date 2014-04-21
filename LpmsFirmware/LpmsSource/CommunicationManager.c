/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CommunicationManager.h"

#define MAX_BUFFER 2048
#define MAX_BUFFER_BLE 128

typedef void (*pFunction)(void);

volatile int txIndex = 0;
uint8_t txBuffer[MAX_BUFFER];
uint8_t txBuffer2[MAX_BUFFER];
uint8_t connected = 0;
uint32_t overSending = 0;
uint8_t isFirmwareUpdating = 0;
uint32_t flash_destination;
uint32_t rxFirmwarePacketCounter = 0;
uint32_t rxFirmwarePacketSize = 0;
uint32_t JumpAddress;
pFunction Jump_To_Application;
uint8_t isConfigForFirmwareUpdateSent = 0;
volatile int firstTimeTx = 1;
volatile int rs232FirstTimeTx = 1;
volatile int ttlUsartFirstTimeTx = 1;

extern uint8_t rxPacketBufferPtr;
extern uint8_t processedPacketPtr;
extern uint8_t connectedInterface;
extern LpmsPacket rxPacketBuffer[MAX_RX_PACKET_BUFFER];
extern LpmsReg gReg;
extern int modeHasChanged;
extern int modeChangeCounter;

void initCommunicationManager(void)
{
	uint8_t config[4];
	getReg(config, 4, LPMS_CONFIG);

#ifdef USE_CANBUS_INTERFACE	
#ifdef USE_RS232_INTERFACE	
	rs232PortInit(USART_BAUDRATE_921600);
	connectedInterface = RS232_CONNECTED;

	serialPortInit(USART_BAUDRATE_921600);
#elif defined USE_TTL_UART_INTERFACE
#ifdef LPMS_BLE	
	ttlUsartPortInit(USART_BAUDRATE_9600);
#else
	ttlUsartPortInit(USART_BAUDRATE_921600);
#endif
	
	connectedInterface = TTL_UART_CONNECTED;
	serialPortInit(USART_BAUDRATE_921600);
#else
	uint8_t canBaudrate;

	CANInitBaudrate(CANBUS_BAUDRATE_1M_ENABLED);

	canBaudrate = (gReg.data[LPMS_CONFIG] & LPMS_CAN_BAUDRATE_MASK) >> 3;
	CANInitBaudrate(canBaudrate);
	
	serialPortInit(USART_BAUDRATE_921600);
#endif
#endif

#ifdef USE_BLUETOOTH_INTERFACE	
	bluetoothInitBaudrate(BT_BAUDRATE_921600_ENABLED);
#endif

	setStreamMode();
}

uint8_t checkCommunicationStatus(void)
{
	return 1;
}

uint8_t sendData(uint16_t address, uint16_t function, uint16_t length, uint8_t *data)
{
	int i;

	uint8_t txData[MAX_PACKET_DATA_LENGTH];
	uint16_t txLrcCheck;

#ifdef LPMS_BLE
	if (connectedInterface == TTL_UART_CONNECTED) {
		txData[0] = 0x3a;
		txData[1] = function & 0xff;
		txData[2] = length & 0xff;
		
		if (length != 0) {
			for (uint16_t i = 0; i < length; i++) {
				txData[3 + i] = data[i];
			}
		}
		
		txLrcCheck = function;
		txLrcCheck += length;
		
		if (length != 0) {
			for (uint16_t i = 0; i < length; i++) {
				txLrcCheck += data[i];
			}
		}
		
		txData[3 + length] = txLrcCheck & 0xff;
		txData[4 + length] = (txLrcCheck >> 8) & 0xff;
		
		if (txIndex+5+length < MAX_BUFFER) {
			for (i=0; i<(5+length); ++i) {
				txBuffer[txIndex] = txData[i];
				++txIndex;
			}
		} else {
			asm("NOP");
		}
	} else {
		txData[0] = 0x3a;
		txData[1] = address & 0xff;
		txData[2] = (address >> 8) & 0xff;
		txData[3] = function & 0xff;
		txData[4] = (function >> 8) & 0xff;
		txData[5] = length & 0xff;
		txData[6] = (length >> 8) & 0xff;
		
		if (length != 0) {
			for (uint16_t i = 0; i < length; i++) {
				txData[7 + i] = data[i];
			}
		}
		
		txLrcCheck = address;
		txLrcCheck += function;
		txLrcCheck += length;
		
		if (length != 0) {
			for (uint16_t i = 0; i < length; i++) {
				txLrcCheck += data[i];
			}
		}
		
		txData[7 + length] = txLrcCheck & 0xff;
		txData[8 + length] = (txLrcCheck >> 8) & 0xff;
		txData[9 + length] = 0x0d;
		txData[10 + length] = 0x0a;

		if (txIndex+11+length < MAX_BUFFER) {
			for (i=0; i<(11+length); ++i) {
				txBuffer[txIndex] = txData[i];
				++txIndex;
			}
		} else {
			asm("NOP");
		}
	}
#else	
	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	txData[2] = (address >> 8) & 0xff;
	txData[3] = function & 0xff;
	txData[4] = (function >> 8) & 0xff;
	txData[5] = length & 0xff;
	txData[6] = (length >> 8) & 0xff;
	
	if (length != 0) {
		for (uint16_t i = 0; i < length; i++) {
			txData[7 + i] = data[i];
		}
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	if (length != 0) {
		for (uint16_t i = 0; i < length; i++) {
			txLrcCheck += data[i];
		}
	}
	
	txData[7 + length] = txLrcCheck & 0xff;
	txData[8 + length] = (txLrcCheck >> 8) & 0xff;
	txData[9 + length] = 0x0d;
	txData[10 + length] = 0x0a;

// #ifdef LPMS_BLE
// 	if (txIndex+11+length < MAX_BUFFER_BLE) {
// #else
	if (txIndex+11+length < MAX_BUFFER) {
// #endif
		for (i=0; i<(11+length); ++i) {
			txBuffer[txIndex] = txData[i];
			++txIndex;
		}
	} else {
		asm("NOP");
	}
#endif

	return 1;
}

void waitForSendCompleted(void)
{
#ifdef USE_BLUETOOTH_INTERFACE
	while (bluetoothIsReadyForSend() == 0);
#else
	serialPortStartTransfer(txBuffer2, txIndex);
#endif
}

void sendQueue(void)
{
	int i, nTx;
	
#ifdef USE_BLUETOOTH_INTERFACE
	if (bluetoothIsReadyForSend() == 0 && firstTimeTx == 0) return;
#else
	if (connectedInterface == CANOPEN_CONNECTED) return;
	if (connectedInterface == USB_CONNECTED && serialPortIsTransferCompleted() == 0 && firstTimeTx == 0) return;
	if (connectedInterface == RS232_CONNECTED && rs232PortIsTransferCompleted() == 0 && rs232FirstTimeTx == 0) return;
	if (connectedInterface == TTL_UART_CONNECTED && ttlUsartPortIsTransferCompleted() == 0 && ttlUsartFirstTimeTx == 0) return;
#endif

	if (txIndex == 0) return;                          

#ifdef LPMS_BLE
	if (connectedInterface == TTL_UART_CONNECTED) {
		if (txIndex > 20) {
			nTx = 20;
	
			for (i=0; i<nTx; ++i) txBuffer2[i] = txBuffer[i];
	
			int j = 0;
			for (i=nTx; i<txIndex; ++i) {
				txBuffer[j] = txBuffer[i];
				++j;
			}
	
			txIndex -= 20;
		} else {
			nTx = txIndex;
	
			for (i=0; i<nTx; ++i) txBuffer2[i] = txBuffer[i];
	
			int nF = nTx % 20;
			if (nF > 0) {
				for (int i=nTx; i < (nTx + 20 - nF); ++i) txBuffer2[i] = 0x0;
				nTx += 20 - nF;
			}
	
			txIndex = 0;
		}
	} else {
		nTx = txIndex;
		
		for (i=0; i<txIndex; ++i) txBuffer2[i] = txBuffer[i];
	
		txIndex = 0;
	}
#else
	nTx = txIndex;
	
	for (i=0; i<txIndex; ++i) txBuffer2[i] = txBuffer[i];

	txIndex = 0;
#endif

#ifdef USE_BLUETOOTH_INTERFACE
	bluetoothStartDataTransfer(txBuffer2, nTx);
#endif

#ifdef USE_CANBUS_INTERFACE
	if (	connectedInterface == CANBUS_CONNECTED || 
		connectedInterface == CANOPEN_CONNECTED) {
		CANStartDataTransfer(txBuffer2, nTx);
	} else if (connectedInterface == USB_CONNECTED) { 
	  	serialPortStartTransfer(txBuffer2, nTx);
	} else if (connectedInterface == RS232_CONNECTED) {
		rs232PortStartTransfer(txBuffer2, nTx);
	} else if (connectedInterface == TTL_UART_CONNECTED) {
		ttlUsartPortStartTransfer(txBuffer2, nTx);
	}
#endif

#ifdef USE_BLUETOOTH_INTERFACE
	firstTimeTx = 0;
#else
	if (connectedInterface == USB_CONNECTED) firstTimeTx = 0;
	if (connectedInterface == RS232_CONNECTED) rs232FirstTimeTx = 0;
	if (connectedInterface == TTL_UART_CONNECTED) ttlUsartFirstTimeTx = 0;
#endif
}

void updateDataTransmission(void)
{          
  	uint8_t dataBuffer[512];
	uint16_t dataLength = 0;	

	getSensorData(dataBuffer, &dataLength);

#ifdef USE_BLUETOOTH_INTERFACE	
	sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);
#else
	if (connectedInterface == CANBUS_CONNECTED) {
		sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);
	} else if (connectedInterface == USB_CONNECTED) {
		sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);
	} else if (connectedInterface == CANOPEN_CONNECTED) {
		sendCANOpenOrientationData();
	} else if (connectedInterface == RS232_CONNECTED) {
		sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);
	} else if (connectedInterface == TTL_UART_CONNECTED) {
		sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);
	}
#endif
}

void sendAck(void)
{
	uint8_t data = 0;
	sendData(getImuID(), REPLY_ACK, 0, &data);
}

void sendNack(void)
{
	uint8_t data = 0;
	sendData(getImuID(), REPLY_NACK, 0, &data);
}

void sendFirmwareUpdateAck(void)
{
	uint8_t data[11];
	uint16_t checksum = 0;
	
	data[0] = 0x3a;
	data[1] = (uint8_t)(getImuID() & 0xff);
	data[2] = (uint8_t)(getImuID() >> 8);
	data[3] = (uint8_t)(REPLY_ACK & 0xff);
	data[4] = (uint8_t)(REPLY_ACK >> 8);
	data[5] = 0;
	data[6] = 0;

	for (int i = 1; i < 7; i++) {
		checksum += (uint16_t)data[i];
	}

	data[7] = (uint8_t)(checksum & 0xff);
	data[8] = (uint8_t)(checksum >> 8);
	data[9] = 0x0d;
	data[10] = 0x0a;
	
#ifdef USE_CANBUS_INTERFACE	
	if (connectedInterface == CANBUS_CONNECTED) {
		CANStartDataTransfer(data, 11);
	} else if (connectedInterface == USB_CONNECTED) { 
	  	serialPortStartTransfer(data, 11);
	} else if (connectedInterface == RS232_CONNECTED) {
		rs232PortStartTransfer(txBuffer2, txIndex);
	} else if (connectedInterface == TTL_UART_CONNECTED) {
		ttlUsartPortStartTransfer(txBuffer2, txIndex);
	}
#endif

#ifdef USE_BLUETOOTH_INTERFACE
	bluetoothStartDataTransfer(data, 11);
#endif
}

void sendFirmwareUpdateNack(void)
{
	uint8_t data[11];
	uint16_t checksum = 0;
	
	data[0] = 0x3a;
	data[1] = (uint8_t)(getImuID() & 0xff);
	data[2] = (uint8_t)(getImuID() >> 8);
	data[3] = (uint8_t)(REPLY_NACK & 0xff);
	data[4] = (uint8_t)(REPLY_NACK >> 8);
	data[5] = 0;
	data[6] = 0;

	for (int i = 1; i < 7; i++) {
		checksum += (uint16_t)data[i];
	}

	data[7] = (uint8_t)(checksum & 0xff);
	data[8] = (uint8_t)(checksum >> 8);
	data[9] = 0x0d;
	data[10] = 0x0a;
	

#ifdef USE_CANBUS_INTERFACE	
	if (connectedInterface == CANBUS_CONNECTED) {
		CANStartDataTransfer(data, 11);
	} else if (connectedInterface == USB_CONNECTED) { 
	  	serialPortStartTransfer(data, 11);
	} else if (connectedInterface == RS232_CONNECTED) {
		rs232PortStartTransfer(txBuffer2, txIndex);
	} else if (connectedInterface == TTL_UART_CONNECTED) {
		ttlUsartPortStartTransfer(txBuffer2, txIndex);
	}
#endif
	
#ifdef USE_BLUETOOTH_INTERFACE
	bluetoothStartDataTransfer(data, 11);
#endif
}

void ackForFirmwareUpdate(void)
{
#ifdef USE_CANBUS_INTERFACE
	sendFirmwareUpdateAck();
#endif

#ifdef USE_BLUETOOTH_INTERFACE
	sendAck();
#endif
}

void nackForFirmwareUpdate(void)
{
#ifdef USE_CANBUS_INTERFACE
	sendFirmwareUpdateNack();
#endif

#ifdef USE_BLUETOOTH_INTERFACE
	sendNack();
#endif
}

void parsePacket(void)
{
  	uint8_t ui32[4];
  	uint8_t dataBuffer[256];
	uint16_t dataLength;
	
	if (processedPacketPtr != rxPacketBufferPtr) {
		uint32_t buffer;
		LpmsPacket packet;
		packet = rxPacketBuffer[processedPacketPtr];
		buffer = (uint32_t)packet.data;
		
		if (getCurrentMode() == LPMS_COMMAND_MODE) {
			switch (packet.function) {
			case UPDATE_FIRMWARE:
			  	if ((packet.length == 4) && (isFirmwareUpdating == 0)) {
					isFirmwareUpdating = 1;
					rxFirmwarePacketCounter = 0;
					rxFirmwarePacketSize = 0;

					for (uint8_t i = 0; i < packet.length; i++) {
						rxFirmwarePacketSize = rxFirmwarePacketSize | (((uint32_t)packet.data[i]) << (8 * i));
					}
					
					flash_destination = USER_APPLICATION_BACKUP_ADDRESS;
					
					FLASH_Unlock();
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
					if (FLASH_EraseSector(FLASH_Sector_5, VoltageRange_3) != FLASH_COMPLETE) {
					  	sendNack();
					} else {
					  	sendAck();
					}

				} else if ((packet.length != 4) && (isFirmwareUpdating == 1)) {
#ifdef LPMS_BLE
					if (copyRamToFlash_128bytes(&flash_destination, (uint32_t*)buffer)) {
#else
					if (copyRamToFlash_256bytes(&flash_destination, (uint32_t*)buffer)) {
#endif
						rxFirmwarePacketCounter++;
						sendAck();
    
						if (rxFirmwarePacketCounter >= rxFirmwarePacketSize) {
							sendQueue();
							msDelay(1000);		
							
							isFirmwareUpdating = 0;
							rxFirmwarePacketCounter = 0;

							if (((*(__IO uint32_t*)IAP_FLASH_START_ADDRESS) & 0x2FFE0000 ) == 0x20000000) {
								eraseCompleteRegisterSet();

								JumpAddress = *(__IO uint32_t*) (IAP_FLASH_START_ADDRESS + 4);
								Jump_To_Application = (pFunction) JumpAddress;

								__set_MSP(*(__IO uint32_t*) IAP_FLASH_START_ADDRESS);
								msDelay(1000);
								Jump_To_Application();
							} else {
								sendNack();
							}
						}
					} else {
						sendNack();				
					}
				} else {
					sendNack();
				}
			break;
			
			case GOTO_COMMAND_MODE:
				setCommandMode();
				sendAck();				
			break;				
				
			case GOTO_STREAM_MODE:
				setStreamMode();
				sendAck();				
			break;
				
			case GOTO_SLEEP_MODE:
				setSleepMode();
				sendAck();	
			break;
				
			case RESTORE_FACTORY_VALUE:
				if (resetToFactory() == 0) {
					sendNack();
				} else {
					sendAck();
					initSensorManager();
				}
			break;
				
			case SET_ORIENTATION_OFFSET:
				if (setOrientationOffset(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;

			case RESET_ORIENTATION_OFFSET:
				if (resetOrientationOffset(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
				
			case SELF_TEST:
				if (setSelfTest(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}		
			break;
				
			case SET_IMU_ID:
				setReg(packet.length, packet.data, LPMS_IMU_ID);
#ifdef USE_CANBUS_INTERFACE
				resetCanId();
#endif
				sendAck();
			break;
				
			case SET_STREAM_FREQ:
#ifdef LPMS_BLE
				sendAck();
#else
				if (setStreamFreq(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
#endif
			break;
				
			case START_GYR_CALIBRA:
				startGyrCalibration();
				sendAck();			
			break;
				
			case SET_GYR_RANGE:
				if (setGyrRange(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
				
			case SET_ACC_RANGE:
				if (setAccRange(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
			
			case SET_MAG_RANGE:
				if (setMagRange(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
				
			case SET_ACC_BIAS:
				if (setAccOffset(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
				
			case RESET_REFERENCE:
				startRefCalibration();
				sendAck();
			break;
				
			case ENABLE_GYR_THRES:
				if (setEnableGyrThresh(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;
			
			case SET_FILTER_MODE:
				if (setFilterMode(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;					
				
			case SET_FILTER_PRESET:
				if (setFilterPreset(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;		  
			
			case GET_CONFIG:
				getReg(ui32, 4, LPMS_CONFIG);
				sendData(getImuID(), GET_CONFIG, 4, ui32);
			break;
			
			case GET_IMU_ID:
				getReg(ui32, 4, LPMS_IMU_ID);
				sendData(getImuID(), GET_IMU_ID, 4, ui32);
			break;
			
			case GET_STATUS:
				getStatus(ui32);
				sendData(getImuID(), GET_STATUS, 4, ui32);
			break;
				
			case GET_GYR_RANGE:
				getReg(ui32, 4, LPMS_GYR_RANGE);
				sendData(getImuID(), GET_GYR_RANGE, 4, ui32);
			break;
			
			case GET_ACC_RANGE:
				getReg(ui32, 4, LPMS_ACC_RANGE);
				sendData(getImuID(), GET_ACC_RANGE, 4, ui32);
			break;
			
			case GET_MAG_RANGE:
				getReg(ui32, 4, LPMS_MAG_RANGE);
				sendData(getImuID(), GET_MAG_RANGE, 4, ui32);
			break;
			
			case GET_ACC_BIAS:
				getAccOffset(dataBuffer, &dataLength);
				sendData(getImuID(), GET_ACC_BIAS, dataLength, dataBuffer);
			break;		
				
			case GET_SENSOR_DATA:
				updateSensorData();
				processSensorData();

				getSensorData(dataBuffer, &dataLength);
				sendData(getImuID(), GET_SENSOR_DATA, dataLength, dataBuffer);				
			break;
				
			case GET_FILTER_MODE:
				getReg(ui32, 4, LPMS_FILTER_MODE);
				sendData(getImuID(), GET_FILTER_MODE, 4, ui32);
			break;
			
			case GET_FILTER_PRESET:
				getReg(ui32, 4, LPMS_FILTER_PRESET);
				sendData(getImuID(), GET_FILTER_PRESET, 4, ui32);
			break;
				
			case UPDATE_IAP:
				if ((packet.length == 4) && (isFirmwareUpdating == 0)) {
					isFirmwareUpdating = 1;
					rxFirmwarePacketCounter = 0;
					rxFirmwarePacketSize = 0;
					for (uint8_t i = 0; i < packet.length; i++) {
						rxFirmwarePacketSize = rxFirmwarePacketSize | (((uint32_t)packet.data[i]) << (8 * i));
					}
					flash_destination = IAP_FLASH_START_ADDRESS;  
 
					FLASH_Unlock();
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
					if (FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3) != FLASH_COMPLETE) {
					  	sendNack();
					} else {
					  	sendAck();
					}
				} else if ((packet.length != 4) && (isFirmwareUpdating == 1)) {
#ifdef LPMS_BLE
					if (copyRamToFlash_128bytes(&flash_destination, (uint32_t*)buffer)) {
#else
					if (copyRamToFlash_256bytes(&flash_destination, (uint32_t*)buffer)) {
#endif
						rxFirmwarePacketCounter++;
						sendAck();
						if (rxFirmwarePacketCounter >= rxFirmwarePacketSize) {
							isFirmwareUpdating = 0;
							rxFirmwarePacketCounter = 0;
						}
					} else {
						sendNack();
						
					}
				}
			break;
				
			case WRITE_REGISTERS:
				if (writeRegisters() == 1) {
					sendAck();
				} else {
					sendNack();
				}
			break;

			case ENABLE_GYR_AUTOCAL:
				if (setEnableGyrAutoCal(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;

			case SET_CAN_BAUDRATE:
				if (setCanBaudrate(packet.data)) {
					sendAck();
				} else {
					sendNack();
				}
			break;

			case GET_HARD_IRON_OFFSET:
				getHardIronOffsetData(dataBuffer, &dataLength);		
				sendData(getImuID(), GET_HARD_IRON_OFFSET, dataLength, dataBuffer);
			break;

			case GET_SOFT_IRON_MATRIX:
				getSoftIronMatrixData(dataBuffer, &dataLength);		
				sendData(getImuID(), GET_SOFT_IRON_MATRIX, dataLength, dataBuffer);
			break;

			case GET_FIELD_ESTIMATE:
				getFieldEstimateData(dataBuffer, &dataLength);
				sendData(getImuID(), GET_FIELD_ESTIMATE, dataLength, dataBuffer);
			break;

			case SET_HARD_IRON_OFFSET:
				setHardIronOffsetData(packet.data);
				sendAck();
			break;	

			case SET_SOFT_IRON_MATRIX:
				setSoftIronMatrixData(packet.data);
				sendAck();
			break;

			case SET_FIELD_ESTIMATE:
				setFieldEstimateData(packet.data);
				sendAck();
			break;

			case SET_ACC_ALIGN_MATRIX:
				setAccAlignMatrix(packet.data);
				sendAck();
			break;

			case GET_ACC_ALIGN_MATRIX:
				getAccAlignMatrix(dataBuffer, &dataLength);
				sendData(getImuID(), GET_ACC_ALIGN_MATRIX, dataLength, dataBuffer);
			break;

			case SET_GYR_ALIGN_MATRIX:
				setGyrAlignMatrix(packet.data);
				sendAck();
			break;

			case SET_GYR_ALIGN_BIAS:
				setGyrAlignBias(packet.data);
				sendAck();
			break;

			case GET_GYR_ALIGN_MATRIX:
				getGyrAlignMatrix(dataBuffer, &dataLength);
				sendData(getImuID(), GET_GYR_ALIGN_MATRIX, dataLength, dataBuffer);
			break;

			case GET_GYR_ALIGN_BIAS:
				getGyrAlignBias(dataBuffer, &dataLength);		
				sendData(getImuID(), GET_GYR_ALIGN_BIAS, dataLength, dataBuffer);
			break;

			case SET_TRANSMIT_DATA:
#ifdef LPMS_BLE
				sendAck();
#else
				setTransmitData(packet.data);
				sendAck();
#endif
			break;	

			case GET_FIRMWARE_VERSION:
				getFirmwareVersion(dataBuffer, &dataLength);			
				sendData(getImuID(), GET_FIRMWARE_VERSION, dataLength, dataBuffer);
			break;

			case GET_RAW_DATA_LP:
				getRawDataLp(dataBuffer, &dataLength);
				sendData(getImuID(), GET_RAW_DATA_LP, dataLength, dataBuffer);
			break;

			case SET_RAW_DATA_LP:
				setRawDataLp(packet.data);
				sendAck();
			break;

			case SET_CAN_MAPPING:
				setCanMapping(packet.data);
				sendAck();
			break;

			case GET_CAN_MAPPING:
				getCanMapping(dataBuffer, &dataLength);
				sendData(getImuID(), GET_CAN_MAPPING, dataLength, dataBuffer);
			break;

			case SET_CAN_HEARTBEAT:
				setCanHeartbeat(packet.data);
				sendAck();
			break;

			case GET_CAN_HEARTBEAT:
				getCanHeartbeat(dataBuffer, &dataLength);
				sendData(getImuID(), GET_CAN_HEARTBEAT, dataLength, dataBuffer);
			break;

			case SET_TIMESTAMP:
				setTimestamp(packet.data);
				sendAck();
			break;

			case SET_LIN_ACC_COMP_MODE:
				setLinAccCompMode(packet.data);
				sendAck();
			break;

			case GET_LIN_ACC_COMP_MODE:
				getLinAccCompMode(dataBuffer, &dataLength);
				sendData(getImuID(), GET_LIN_ACC_COMP_MODE, dataLength, dataBuffer);
			break;
			
			case SET_CENTRI_COMP_MODE:
				setCentriCompMode(packet.data);
				sendAck();
			break;
			
			case GET_CENTRI_COMP_MODE:
				getCentriCompMode(dataBuffer, &dataLength);
				sendData(getImuID(), GET_CENTRI_COMP_MODE, dataLength, dataBuffer);
			break;

			case GET_CAN_CONFIGURATION:
				getCanConfiguration(dataBuffer, &dataLength);
				sendData(getImuID(), GET_CAN_CONFIGURATION, dataLength, dataBuffer);
			break;

			case SET_CAN_CHANNEL_MODE:
				setCanChannelMode(packet.data);
				sendAck();
			break;

			case SET_CAN_POINT_MODE:
				setCanPointMode(packet.data);
				sendAck();
			break;

			case SET_CAN_START_ID:
				setCanStartId(packet.data);
				sendAck();
			break;

			case SET_LPBUS_DATA_MODE:
#ifdef LPMS_BLE
				sendAck();
#else
				setLpBusDataMode(packet.data);
				sendAck();
#endif
			break;

			case SET_MAG_ALIGNMENT_MATRIX:
				setMagAlignMatrix(packet.data);
				sendAck();
			break;

			case SET_MAG_ALIGNMENT_BIAS:
				setMagAlignBias(packet.data);
				sendAck();
			break;

			case SET_MAG_REFRENCE:
				setMagReference(packet.data);
				sendAck();
			break;

			case GET_MAG_ALIGNMENT_MATRIX:
				getMagAlignMatrix(dataBuffer, &dataLength);
				sendData(getImuID(), GET_MAG_ALIGNMENT_MATRIX, dataLength, dataBuffer);
			break;			

			case GET_MAG_ALIGNMENT_BIAS:
				getMagAlignBias(dataBuffer, &dataLength);
				sendData(getImuID(), GET_MAG_ALIGNMENT_BIAS, dataLength, dataBuffer);
			break;

			case GET_MAG_REFERENCE:
				getMagReference(dataBuffer, &dataLength);
				sendData(getImuID(), GET_MAG_REFERENCE, dataLength, dataBuffer);
			break;

			default:
			break;
			}
				
			processedPacketPtr++;
			if (processedPacketPtr >= MAX_RX_PACKET_BUFFER) {
				processedPacketPtr = 0;
			}
		} else if (getCurrentMode() == LPMS_STREAM_MODE) {
			switch (packet.function) {				
			case GOTO_SLEEP_MODE:
				setSleepMode();
				sendAck();
			break;
				
			case GOTO_COMMAND_MODE:
				setCommandMode();
				sendAck();
			break;
				
			case GET_STATUS:
				getStatus(ui32);
				sendData(getImuID(), GET_STATUS, 4, ui32);				
			break;

                        case SET_TIMESTAMP:
                                setTimestamp(packet.data);
                        break;
				
			default:
			break;
			}
				
			processedPacketPtr++;
			if (processedPacketPtr >= MAX_RX_PACKET_BUFFER) processedPacketPtr = 0;
		} else if (getCurrentMode() == LPMS_SLEEP_MODE) {
			switch (packet.function) {				
			case GOTO_STREAM_MODE:
				setStreamMode();
				sendAck();
			break;
				
			case GOTO_COMMAND_MODE:
				setCommandMode();
				sendAck();
			break;
				
			case GET_STATUS:
				getStatus(ui32);
				sendData(getImuID(), GET_STATUS, 4, ui32);		
			break;
				
			default:
			break;
			}
				
			processedPacketPtr++;
			if (processedPacketPtr >= MAX_RX_PACKET_BUFFER) processedPacketPtr = 0;
		} else {
			processedPacketPtr++;
			if (processedPacketPtr >= MAX_RX_PACKET_BUFFER) processedPacketPtr = 0;
		}
	}
}