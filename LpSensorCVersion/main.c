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

#include <stdio.h>

#include "LpmsSensorC.h"

#define USE_LPMS_SENSOR

int main()
{
	int i;
	CalibrationData cd;

#ifdef USE_LPMS_SENSOR
	lpmsSensorInit(DEVICE_LPMS_RS232, "COM159");
	
	while (1) {
		lpmsSensorPollData();
		lpmsSensorUpdate();
	}
#else
	lpmsInitIoInterface(&cd);
	lpmsConnect("COM159");
	
	printf("[main] Set command mode.\n");
	lpmsSetCommandMode();
	while (lpmsIsWaitForData() == 1 || lpmsIsWaitForAck() == 1) lpmsSensorPollData();
	
	printf("[main] Get config.\n");
	lpmsGetConfig();
	while (lpmsIsWaitForData() == 1 || lpmsIsWaitForAck() == 1) lpmsSensorPollData();
	
	printf("[main] Set stream mode.\n");
	lpmsSetStreamMode();
	while (lpmsIsWaitForData() == 1 || lpmsIsWaitForAck() == 1) lpmsSensorPollData();
	
	while (1) {
		lpmsSensorPollData();
	}
#endif
	
    return 0;
} 