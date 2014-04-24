/***********************************************************************
** Copyright (C) 2011 LP-Research
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

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "BleUart.h"

#ifdef _WIN32

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include <windows.h>
#include <setupapi.h>

HANDLE serial_handle;

int uart_list_devices()
{
	char name[] = "Bluegiga Bluetooth Low Energy";

	BYTE* pbuf = NULL;
	DWORD reqSize = 0;
	DWORD n=0;
	HDEVINFO hDevInfo;
	
	static const GUID guid = { 0x4d36e978, 0xe325, 0x11ce, { 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } };
	
	char *str;
	char tmp[MAX_PATH+1];
	int i;
	SP_DEVINFO_DATA DeviceInfoData;

	snprintf(tmp, MAX_PATH, "%s (COM%%d)",name);

	DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	hDevInfo = SetupDiGetClassDevs(&guid, 0L, NULL, DIGCF_PRESENT);
	
	if (hDevInfo==INVALID_HANDLE_VALUE) return 0;

	while(1) {
		if (!SetupDiEnumDeviceInfo(hDevInfo, n++, &DeviceInfoData)) {
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 0;
		}
		
		reqSize = 0;
		
		SetupDiGetDeviceRegistryPropertyA(hDevInfo, &DeviceInfoData, SPDRP_FRIENDLYNAME, NULL, NULL, 0, &reqSize);
		
		pbuf = (BYTE*)malloc(reqSize>1? reqSize:1);
		
		if (!SetupDiGetDeviceRegistryPropertyA(hDevInfo, &DeviceInfoData, SPDRP_FRIENDLYNAME, NULL, pbuf, reqSize, NULL)) {
			free(pbuf);
			continue;
		}
		
		str = (char*)pbuf;
		
		if(sscanf(str,tmp,&i) == 1) {
			printf("[BleUart] Found BLE dongle: %s\n", str);
			return i;
		}
		
		free(pbuf);
	}
	
	return 0;
}

int uart_open(const char *port)
{
	char str[20];
	// DCB portDcb;

	snprintf(str, sizeof(str)-1, "\\\\.\\%s", port);
	
	serial_handle = CreateFileA(str, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

	if (serial_handle == INVALID_HANDLE_VALUE) {
		// printf("[BleUart] BLE dongle failed to open\n");
		
		return -1;
	}

	printf("[BleUart] BLE dongle opened successfully\n");

	return 0;
}

void uart_close()
{
	CloseHandle(serial_handle);
}

int uart_tx(int len,unsigned char *data)
{
	DWORD r,written;

	/* printf("[LPMS-BLE] Writing data: ");

	for (int i=0; i<len; ++i) {
		printf("%x ", data[i]);
	}
	printf("\n"); */

	while (len) {
		r = WriteFile (serial_handle, data, len, &written, NULL);

		if (!r) {
			// printf("[BleUart] Write failed\n");
			return -1;
		}

		len -= written;
		data += len;
	}

	return 0;
}

int uart_rx(int len, unsigned char *data, int timeout_ms)
{
	int l = len;
	DWORD r, rread;
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = timeout_ms;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;

	SetCommTimeouts(
		serial_handle,
		&timeouts
	);
	
	while (len) {
		r = ReadFile(serial_handle, data, len, &rread, NULL);

		if (!r) {
			l = GetLastError();
			if (l == ERROR_SUCCESS) return 0;
			
			return -1;
		} else {
			if (rread == 0) return 0;
		}
	
		len -= rread;
		data += len;
	}

	return l;
}

#endif

#ifdef __GNUC__

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

int serial_handle;

int uart_list_devices()
{
	printf("[BleUart] BLE list devices not supported\n");
}

int uart_open(char *port)
{
	struct termios options;
	int i;

	erial_handle = open(port, (O_RDWR | O_NOCTTY /*| O_NDELAY*/));

	if (serial_handle < 0) {
		return -1;
	}

	/*
	* Get the current options for the port...
	*/
	tcgetattr(serial_handle, &options);

	/*
	* Set the baud rates to 115200...
	*/
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	/*
	* Enable the receiver and set parameters ...
	*/
	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
	options.c_cflag |= (CS8 | CLOCAL | CREAD);
	options.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | IEXTEN);
	options.c_iflag &= ~(INPCK | IXON | IXOFF | IXANY | ICRNL);
	options.c_oflag &= ~(OPOST | ONLCR);

	//printf( "size of c_cc = %d\n", sizeof( options.c_cc ) );
	for (i = 0; i < sizeof(options.c_cc); i++) options.c_cc[i] = _POSIX_VDISABLE;

	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 1;

	/*
	* Set the new options for the port...
	*/
	tcsetattr(serial_handle, TCSAFLUSH, &options);

	return 0;
}
void uart_close()
{
	close(serial_handle);
}

int uart_tx(int len,unsigned char *data)
{
	ssize_t written;

	while(len) {
		written=write(serial_handle, data, len);
		if (!written) {
			return -1;
		}
		
		len-=written;
		data+=len;
	}

	return 0;
}

int uart_rx(int len,unsigned char *data,int timeout_ms)
{
	int l = len;
	ssize_t rread;
	struct termios options;

	tcgetattr(serial_handle, &options);
	options.c_cc[VTIME] = timeout_ms/100;
	options.c_cc[VMIN] = 0;
	tcsetattr(serial_handle, TCSANOW, &options);

	while(len) {
		rread = read(serial_handle, data, len);

		if(!rread) {
			return 0;
		} else if(rread < 0) {
			return -1;
		}
		
		len -= rread;
		data += len;
	}

	return l;
}

#endif
