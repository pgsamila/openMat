/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpmsAssignments.h"

extern LpmsReg gReg;

void setReg(uint16_t length, uint8_t* data, uint32_t address)
{
	uint32_t reg;

	for (uint8_t i = 0; i < (length / 4); i++) {
		reg = 0;
		
		for (uint8_t j = 0; j < 4; j++) {
			reg = reg | (((uint32_t)data[i*4 + j]) << (8 * j));
		}
		
		gReg.data[address] = reg;
		address++;
	}
}

void getReg(uint8_t* data, uint16_t length, uint32_t address)
{
	for (uint8_t i = 0; i < (length/4); i++) {
		for (uint8_t j = 0; j < 4; j++) {
			data[i*4 + j] = (uint8_t)((gReg.data[address + i] >> (j * 8)) & 0x000000ff);
		}
	}
}

uint32_t conFtoI(float f)
{
	float2int f2int;
	f2int.float_val = f;

	return f2int.u32_val;
}

float conItoF(uint32_t v)
{
	float2int f2int;
	f2int.u32_val = v;

	return f2int.float_val;
}

uint32_t getUi32t(uint8_t* data)
{
	uint32_t v = 0;
	
	for (uint8_t i = 0; i < 4; i++) {
		v = v | (((uint32_t)data[i]) << (i * 8));
	}	

	return v;
}

void getMultiUi32t(uint8_t* data, uint8_t n, uint32_t* v)
{	
	for (int j = 0; j < n; j++) {
		v[j] = 0;
		for (int i = 0; i < 4; i++) {
			v[j] = v[j] | (((uint32_t)(data[j*4+i])) << (i * 8));
		}	
	}
}

LpMatrix3x3f getMatrix3x3f(uint8_t* data)
{
	uint32_t t[9];
	int i, j;
	LpMatrix3x3f m;

	getMultiUi32t(data, 9, t);

	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			m.data[i][j] = conItoF(t[i*3+j]);
		}
	}

	return m;
}

float getFloat(uint8_t* data)
{
	uint32_t v = 0;
	
	for (int i = 0; i < 4; i++) {
		v = v | (((uint32_t)(data[i])) << (i * 8));
	}	

	return conItoF(v);
}

LpVector3f getVector3f(uint8_t* data)
{
	uint32_t v;
	LpVector3f o;
	
	for (int j=0; j<3; j++) {
		v = 0;

		for (int i=0; i<4; i++) {
			v = v | (((uint32_t)(data[j*4+i])) << (i * 8));
		}	
		
		o.data[j] = conItoF(v);
	}

	return o;
}

LpVector4f getVector4f(uint8_t* data)
{
	uint32_t v;
	LpVector4f o;
	
	for (int j = 0; j < 4; j++) {
		v = 0;

		for (int i = 0; i < 4; i++) {
			v = v | (((uint32_t)(data[j*4+i])) << (i * 8));
		}	
		
		o.data[j] = conItoF(v);
	}

	return o;
} 

void setFloatMatrix3x3f(uint8_t* data, int o, LpMatrix3x3f M)
{
	uint32_t v;

	for (int k=0; k<3; k++) {		
		for (int j=0; j<3; j++) {	
			v = conFtoI(M.data[k][j]);
			for (int i=0; i<4; i++) {
				data[o+k*3*4+j*4+i] = (v >> (i*8)) & (uint8_t) 0xff;
			}
		}
	}
}

void setMultiUi32t(uint8_t* data, uint32_t *v, int l)
{
	int j;

	for (j=0; j<l; j++) {	
		for (int i=0; i<4; i++) {
			data[j*4+i] = (v[j] >> (i*8)) & (uint8_t) 0xff;
		}
	}
}

void setUi32t(uint8_t* data, uint32_t v)
{
	for (int i=0; i<4; i++) {
		data[i] = (v >> (i * 8)) & (uint8_t) 0xff;
	}
}

void setUi8t(uint8_t* data, uint8_t v)
{
	*data = v;
}

/* uint16_t basetable[512];
uint16_t shifttable[512];
uint16_t isGeneratedTable = 0;

void generateTables() {
	unsigned int i;
	int e;

	for(i=0; i < 256; ++i){
		e = i-127;

		if (e < -24) { // Very small numbers map to zero
			basetable[i | 0x000] = 0x0000;
			basetable[i | 0x100] = 0x8000;
			shifttable[i | 0x000] = 24;
			shifttable[i | 0x100] = 24;
		} else if (e < -14){ // Small numbers map to denorms
			basetable[i | 0x000] = (0x0400>>(-e-14));
			basetable[i | 0x100] = (0x0400>>(-e-14)) | 0x8000;
			shifttable[i | 0x000] = -e-1;
			shifttable[i | 0x100] = -e-1;
		} else if (e <= 15){ // Normal numbers just lose precision
			basetable[i | 0x000] = ((e+15)<<10);
			basetable[i | 0x100] = ((e+15)<<10) | 0x8000;
			shifttable[i | 0x000] = 13;
			shifttable[i | 0x100] = 13;
		} else if (e < 128){ // Large numbers map to Infinity
			basetable[i | 0x000] = 0x7C00;
			basetable[i | 0x100] = 0xFC00;
			shifttable[i | 0x000] = 24;
			shifttable[i | 0x100] = 24;
		} else { // Infinity and NaN's stay Infinity and NaN's
			basetable[i | 0x000] = 0x7C00;
			basetable[i | 0x100] = 0xFC00;
			shifttable[i | 0x000] = 13;
			shifttable[i | 0x100] = 13;
		}
	}

	isGeneratedTable = 1;
} */

void setFloat(uint8_t* data, float v, uint8_t prec)
{
	uint32_t f = conFtoI(v);
	uint16_t h;
	
	// if (isGeneratedTable == 0) generateTables();

	switch (prec) {
	case FLOAT_HALF_PRECISION:
		/* h = basetable[(f >> 23) & 0x1ff] + ((f & 0x007fffff) >> shifttable[( f >> 23) & 0x1ff]);
	
		for (int i=0; i<2; i++) {
			data[i] = (h >> (i * 8)) & (uint8_t) 0xff;
		} */
	break;

	case FLOAT_FULL_PRECISION:
		for (int i=0; i<4; i++) {
			data[i] = (f >> (i * 8)) & (uint8_t) 0xff;
		}
	break;

	case FLOAT_FIXED_POINT_10:
		h = (int16_t)(v * 10.0f);

		for (int i=0; i<2; i++) {
			data[i] = (h >> (i * 8)) & (uint8_t) 0xff;
		}
	break;

	case FLOAT_FIXED_POINT_100:
		h = (int16_t)(v * 100.0f);

		for (int i=0; i<2; i++) {
			data[i] = (h >> (i * 8)) & (uint8_t) 0xff;
		}
	break;

	case FLOAT_FIXED_POINT_1000:
		h = (int16_t)(v * 1000.0f);

		for (int i=0; i<2; i++) {
			data[i] = (h >> (i * 8)) & (uint8_t) 0xff;
		}
	break;
	}
}

void setFloatBigEndian(uint8_t* data, float f)
{
	uint32_t v = conFtoI(f);
	
	for (int i=0; i<4; i++) {
		data[3-i] = (v >> (i * 8)) & (uint8_t) 0xff;
	}
}

void setFloatVector3f(uint8_t* data, int o, LpVector3f f)
{
	uint32_t v;

	for (int j=0; j<3; j++) {	
		v = conFtoI(f.data[j]);
		for (int i=0; i<4; i++) {
			data[o+j*4+i] = (v >> (i*8)) & (uint8_t) 0xff;
		}
	}
}

uint8_t setRegFloat(uint8_t i, float v)
{
	gReg.data[i] = conFtoI(v);
	
	return 1;
}

uint8_t getRegFloat(uint8_t i, float *v)
{
	*v = conItoF(gReg.data[i]);
	
	return 1;
}

uint8_t setRegUInt32(uint8_t i, uint32_t v)
{
	gReg.data[i] = v;

	return 1;
}

uint8_t setRegVector3f(uint8_t i, LpVector3f v)
{
	for (uint8_t j=0; j<3; j++) {
		if (setRegFloat(i+j, v.data[j]) == 0) return 0;
	}
	
	return 1;
}

uint8_t getRegVector3f(uint8_t i, LpVector3f *v)
{
	for (uint8_t j=0; j<3; j++) {
		if (getRegFloat(i+j, &(v->data[j])) == 0) return 0;
	}
	
	return 1;
} 

uint8_t setRegVector4f(uint8_t i, LpVector4f v)
{
	for (uint8_t j=0; j<4; j++) {
		if (setRegFloat(i+j, v.data[j]) == 0) return 0;
	}
	
	return 1;
}

uint8_t setRegMatrix3x3f(uint8_t i, LpMatrix3x3f m)
{
	uint8_t j, k;

	for (j=0; j<3; j++) {
		for (k=0; k<3; k++) {
			if (setRegFloat(i+j*3+k, m.data[j][k]) == 0) return 0;
		}
	}

	return 1;
}

uint8_t getRegMatrix3x3f(uint8_t i, LpMatrix3x3f *m)
{
	uint8_t j, k;

	for (j=0; j<3; j++) {
		for (k=0; k<3; k++) {
			if (getRegFloat(i+j*3+k, &(m->data[j][k])) == 0) return 0;
		}
	}

	return 1;
}

uint8_t writeRegToFlash(uint8_t i)
{
	// address = USER_FLASH_START_ADDRESS + (uint32_t) i * 4;
	
	writeCompleteRegisterSet();
	
	return 1;
}