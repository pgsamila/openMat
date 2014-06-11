#include "MfiCom.h"

#include "LpmsRn42.h"

void mfiI2cInit(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	I2C_DeInit(I2C1);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xa0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100;
	
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_Cmd(I2C1, ENABLE);
}

int mfiI2cWrite(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{	
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, 0xd0, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, reg_addr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	while (length) {
		I2C_SendData(I2C1, *data);

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		
		++data;
		--length;
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
		
	return 0;
}

#define MFI_WRITE_ADD 0x20
#define MFI_READ_ADD 0x21

#define MFI_I2C_TIMEOUT 99999

int mfiI2cRead(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{  
  	uint8_t ok = 0;
	uint32_t to = 0;
	
	while (ok == 0) {
		I2C_GenerateSTART(I2C1, ENABLE);
		to = 0;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && to < MFI_I2C_TIMEOUT) ++to;
		if (to >= MFI_I2C_TIMEOUT) continue;
	
		I2C_Send7bitAddress(I2C1, MFI_WRITE_ADD, I2C_Direction_Transmitter);
		to = 0;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && to < MFI_I2C_TIMEOUT) ++to;
		if (to >= MFI_I2C_TIMEOUT) continue;
		
		ok = 1;	
	}

	
	I2C_SendData(I2C1, reg_addr);	
	to = 0;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && to < MFI_I2C_TIMEOUT); // ++to;
	// if (to >= MFI_I2C_TIMEOUT) continue;
	
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	ok = 0;
	while (ok == 0) {		
		I2C_GenerateSTART(I2C1, ENABLE);	
		to = 0;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && to < MFI_I2C_TIMEOUT) ++to;
		if (to >= MFI_I2C_TIMEOUT) continue;
		
		I2C_Send7bitAddress(I2C1, MFI_READ_ADD, I2C_Direction_Receiver);
		to = 0;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && to < MFI_I2C_TIMEOUT) ++to;
		if (to >= MFI_I2C_TIMEOUT) continue;
		
		ok = 1;	
	}
		
	to = 0;
	while (length && to < MFI_I2C_TIMEOUT) {
		if (length == 1) {
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			*data = I2C_ReceiveData(I2C1);
			++data;
			--length;
		} else {
			// ++to;
		}
	}
	// if (to >= MFI_I2C_TIMEOUT) continue;
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
		
	return 0;
}

uint8_t mfiReadDeviceVersion(void)
{
	uint8_t data[1];
  
	mfiI2cRead(0x21, 0x00, 1, data);
	
	return data[0];
}

#define GENERATE_SIGNATURE 1
#define VERIFY_SIGNATURE 3
#define GENERATE_CHALLENGE 2
#define VALIDATE_CERTIFICATE 4

void mfiStartProcess(uint8_t t)
{
	uint8_t data[1];
	
	data[0] = t;
	mfiI2cWrite(0x20, 0x10, 1, data);
}

uint8_t mfiGetProcessResult(void)
{
	uint8_t data[1];
	
	mfiI2cRead(0x21, 0x10, 1, data);
	
	return data[0];
}

#define MFI_WRITE 0x20
#define MFI_READ 0x21

#define MFI_DEVICE_VERSION 0x00
#define MFI_FIRMWARE_VERSION 0x01
#define MFI_PROTOCOL_MAJVER 0x02
#define MFI_PROTOCOL_MINVER 0x03
#define MFI_DEVICE_ID 0x04
#define MFI_ERROR_CODE 0x05
#define MFI_AUT_CONTROL 0x10
#define MFI_SIGNATURE_LEN 0x11
#define MFI_SIGNATURE_START 0x12
#define MFI_CHALLENGE_LEN 0x20
#define MFI_CHALLENGE_START 0x21
#define MFI_ACC_CERTIFICATE_LEN 0x30
#define MFI_ACC_CERTIFICATE_START 0x31

void mfiAuthenticate(uint16_t cl, uint8_t* cd)
{
	uint8_t data[128];
	uint16_t l = 0;
	uint16_t p = 0;
	uint16_t r = 0;

	mfiI2cRead(MFI_READ_ADD, MFI_DEVICE_VERSION, 1, data);
	mfiI2cRead(MFI_READ_ADD, MFI_FIRMWARE_VERSION, 1, data);
	mfiI2cRead(MFI_READ_ADD, MFI_ACC_CERTIFICATE_LEN, 2, data);
	
	mfiI2cRead(MFI_READ_ADD, MFI_DEVICE_ID, 4, data);
	
	l = (data[0] << 8) + data[1];
	p = l  / 128;
	r = l % 128;
	for (int i=0; i<p; ++i) {
		mfiI2cRead(MFI_READ_ADD, MFI_ACC_CERTIFICATE_START+i, 128, data);
		// Send to iOS
	}
	mfiI2cRead(MFI_READ_ADD, MFI_ACC_CERTIFICATE_START+p, r, data);
	// send to iOS

	data[0] = cl;
	data[1] = cl >> 8;
	mfiI2cWrite(MFI_WRITE, MFI_CHALLENGE_LEN, 2, data);
	mfiI2cWrite(MFI_WRITE, MFI_CHALLENGE_START, 2, cd);

	data[0] = VERIFY_SIGNATURE;
	mfiI2cWrite(MFI_WRITE, MFI_AUT_CONTROL, 1, data);

	r = 0;
	while (r==0) {
		mfiI2cRead(MFI_READ, MFI_AUT_CONTROL, 1, data);
		if (data[0] == 1) r = 1;
		// add timeout or state machine to not block main task
	}

	mfiI2cRead(MFI_READ_ADD, MFI_SIGNATURE_LEN, 2, data);
	l = data[0] + data[1] << 8;

	mfiI2cRead(MFI_READ_ADD, MFI_SIGNATURE_START, l, data);
}

extern int txIndex;
extern uint8_t txBuffer[2048];
uint16_t lingoTransId = 0;

void sendLingoPacket(uint16_t lingoId, 
	uint16_t commandId, 
	uint16_t transId, 
	uint16_t payloadLength, 
	uint8_t* payload)
{
	uint8_t data[256];
	uint8_t output[256];
	int8_t cs;
	uint8_t dc;
	uint8_t oc;
	
	int i;
	
	dc = 0;
	data[dc] = lingoId; ++dc;
	data[dc] = commandId; ++dc;
	
	if (commandId > 127) {
                data[dc] = commandId >> 8; ++dc;
        }

	data[dc] = lingoTransId; ++dc;
	data[dc] = lingoTransId >> 8; ++dc;
        ++lingoTransId;

	for (i=0; i<payloadLength; ++i) {
		data[dc] = payload[i]; ++dc;
	}

	oc = 0;
	output[oc] = 0xff; ++oc;
	output[oc] = 0x55; ++oc;

	if (dc > 252) {
		output[oc] = 0x0; ++oc;
		output[oc] = dc; ++oc;
		output[oc] = dc >> 8; ++oc;
	} else {
		output[oc] = dc; ++oc;
	}

	for (i=0; i<dc; ++i) {
		output[oc] = data[i]; ++oc;
	}
	
	cs = 0;
	for (i=2; i<oc; ++i) {
		cs += output[i];
	}

	output[oc] = ~cs + 0x1; ++oc;

	if (txIndex+oc < 2048) {
		for (i=0; i<oc; ++i) {
			txBuffer[txIndex] = output[i];
			++txIndex;
		}
	} else {
		asm("NOP");
	}
}

#define LINGO_START 0
#define LINGO_START_2 1 
#define LINGO_LENGTH 2
#define LINGO_LENGTH_2 3
#define LINGO_LENGTH_3 4
#define LINGO_ID 5
#define LINGO_COMMAND 6
#define LINGO_COMMAND_2 7
#define LINGO_TRANS 8
#define LINGO_TRANS_2 9
#define LINGO_PAYLOAD 10
#define LINGO_CHECKSUM 11

#define MFI_STATE_IDLE 0
#define MFI_START_IDPS 1
#define REQUEST_MAX_PAYLOAD_SIZE 2
#define END_IDPS 3
#define CHECK_READY_FOR_AUTH 4
#define START_AUTHENTIFICATION 5

#define LINGO_REQUEST_IDENTITY 0x0
#define LINGO_IPOD_ACK 0x2
#define LINGO_REQUEST_TRANSPORT_MAXIMUM_PAYLOAD_SIZE 0x11
#define LINGO_RETURN_TRANSPORT_MAXIMUM_PAYLOAD_SIZE 0x12
#define LINGO_REQUEST_IDENTITY 0x0
#define LINGO_START_IDPS 0x38
#define LINGO_END_IDPS 0x3b
#define LINGO_IDPS_STATUS 0x3c

uint8_t lingoState = LINGO_START;
uint16_t lingoLength = 0;
uint16_t lingoCommand = 0;
uint8_t recLingoId = 0;
uint16_t lingoTrans = 0;
uint8_t lingoPayload[256];
uint8_t lingoInput[256];
uint16_t lingoCount = 0;
uint16_t payloadCount = 0;
int8_t lingoChecksum = 0;

extern uint16_t rxDmaBufferPtr;
extern uint8_t bluetoothRxBuffer[BT_MAX_RX_BUFFER_LENGTH];
extern uint8_t prevConnectionStatus;

#define CONNECTION_CONNECTED 0
#define CONNECTION_DISCONNECTED 1

uint8_t mfi_prev_con_status = CONNECTION_DISCONNECTED;
uint8_t mfi_state = MFI_STATE_IDLE;

int waiting_for_data = 0;

uint8_t pollMfiData(void) 
{
	uint8_t b;
        uint8_t payload[16];

// on new connection
        if (checkConnectionStatus() == CONNECTION_CONNECTED) {
                if (mfi_prev_con_status == CONNECTION_DISCONNECTED /* || TIM_GetCounter(TIM5) > 10000 */) {
                        mfi_prev_con_status = CONNECTION_CONNECTED; 
                        mfi_state = MFI_START_IDPS;
                }
        }

// mfi state machine
	switch (mfi_state) {

// start identification
	case MFI_START_IDPS:
                if (waiting_for_data == 0) { 
                        sendLingoPacket(0, LINGO_START_IDPS, 0, 0, payload);
                        mfi_state = REQUEST_MAX_PAYLOAD_SIZE;
                        waiting_for_data = 1;
                }
        break;

// request maximum payload
        case REQUEST_MAX_PAYLOAD_SIZE:
                if (waiting_for_data == 0) {
                        uint16_t ack_result = lingoPayload[0] + lingoPayload[1] << 8;
                        if (ack_result == 0) {
                                sendLingoPacket(0, LINGO_REQUEST_TRANSPORT_MAXIMUM_PAYLOAD_SIZE, 0, 0, payload);
                                mfi_state = END_IDPS;
                                waiting_for_data = 1;
                        }
                }
        break;
        
// end IDPS
        case END_IDPS:
                if (waiting_for_data == 0) {              
                        uint16_t max_payload_size = lingoPayload[0] + lingoPayload[1] << 8;
                        sendLingoPacket(0, LINGO_END_IDPS, 0, 0, payload);
                        mfi_state = CHECK_READY_FOR_AUTH;
                        waiting_for_data = 1;
                }
        break;

// check if IDPS was successful
        case CHECK_READY_FOR_AUTH:
                if (waiting_for_data == 0) {
                        uint16_t idps_status = lingoPayload[0];
                        if (idps_status == 0) {
                                mfi_state = START_AUTHENTIFICATION;
                        } else {
                                mfi_state = MFI_START_IDPS;
                        }
                }
        break;

// start authetification
        case START_AUTHENTIFICATION:
        break;
        }

 	while (rxDmaBufferPtr != (BT_MAX_RX_BUFFER_LENGTH - DMA_GetCurrDataCounter(BT_USART_RX_DMA_STREAM))) {
		b = bluetoothRxBuffer[rxDmaBufferPtr];
		parseLingoByte(b);

		rxDmaBufferPtr++;
		if (rxDmaBufferPtr == BT_MAX_RX_BUFFER_LENGTH) rxDmaBufferPtr = 0;
	}
	
	return 1;
}

void parseLingoCommand(void)
{
        switch (lingoCommand) {
        case LINGO_REQUEST_IDENTITY:
                mfi_state = MFI_START_IDPS;
        break;

        case LINGO_IPOD_ACK:
               waiting_for_data = 0; 
        break;

        case LINGO_RETURN_TRANSPORT_MAXIMUM_PAYLOAD_SIZE:
                waiting_for_data = 0;
        break;

        case LINGO_IDPS_STATUS:
                waiting_for_data = 0;
        break;
        }         
}

void parseLingoByte(uint8_t b) 
{
  	int i;
  
	switch (lingoState) {
	case LINGO_START:
		if (b == 0xff) {
			lingoState = LINGO_START_2;
		} else if (b == 0x55) {
			lingoState = LINGO_LENGTH;
		}
	break;
			
	case LINGO_START_2:
		if (b == 0x55) {
			lingoState = LINGO_LENGTH;
		} else {
			lingoState = LINGO_START;
		}
	break;

	case LINGO_LENGTH:
		lingoCount = 0;
		lingoInput[lingoCount] = b; ++lingoCount;
		
		if (b == 0x00) {
			lingoState = LINGO_LENGTH_2;
		} else {
			lingoLength = b;
                        lingoState = LINGO_ID;
		}
	break;

	case LINGO_LENGTH_2:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoLength = b;
		lingoState = LINGO_LENGTH_3;
	break;

	case LINGO_LENGTH_3:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoLength += b << 8;
		lingoState = LINGO_ID;
	break;

	case LINGO_ID:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		recLingoId = b;
		if (recLingoId != 0) {
			lingoState = LINGO_START;
		} else {
			lingoState = LINGO_COMMAND;
		}
	break;

	case LINGO_COMMAND:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoCommand = b;
		lingoState = LINGO_TRANS; // LINGO_COMMAND_2;
	break;

	case LINGO_COMMAND_2:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoCommand += b << 8;
		lingoState = LINGO_TRANS;
	break;

	case LINGO_TRANS:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoTrans = b;
		lingoState = LINGO_TRANS_2;
	break;
	
	case LINGO_TRANS_2:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		lingoTrans += b << 8;
		payloadCount = 0;
		lingoState = LINGO_PAYLOAD;
	break;

	case LINGO_PAYLOAD:
		lingoInput[lingoCount] = b; ++lingoCount;	  
		if (lingoCount < lingoLength) {
			lingoPayload[payloadCount] = b;
			++payloadCount;
		} else {
			lingoState = LINGO_CHECKSUM;
		}
	break;
	
	case LINGO_CHECKSUM:
	  	lingoChecksum = 0;
		for (i=0; i<lingoCount; ++i) {
			lingoChecksum += lingoInput[i];
		}

                lingoChecksum = ~lingoChecksum + 1;

		if (lingoChecksum == (int8_t) b) {
			parseLingoCommand();
		}
		
		lingoState = LINGO_START;
	break;
	}
}