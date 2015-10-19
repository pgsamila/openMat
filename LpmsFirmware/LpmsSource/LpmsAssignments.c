/***********************************************************************
** (c) LP-RESEARCH Inc.
** info@lp-research.com
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

void setI16t(uint8_t* data, int16_t v)
{
	for (int i=0; i<2; i++) {
		data[i] = (v >> (i * 8)) & (uint8_t) 0xff;
	}
}

char *itobase10(char *buf, int32_t value) {
    sprintf(buf, "%d", value);
    return buf;
}

void setUi32tAscii(uint8_t* data, uint8_t *length, uint32_t v)
{
	char d[64];

	itobase10(d, v);
	*length = strlen(d);
	strcpy((char *)data, d);
}

void setFloatAscii(uint8_t* data, uint8_t *length, float v, uint8_t prec)
{
	uint32_t f = conFtoI(v);
	int32_t h;
	char d[64];
	
	switch (prec) {
	default:
	case FLOAT_HALF_PRECISION:
	case FLOAT_FULL_PRECISION:
	case FLOAT_FIXED_POINT_1:
		h = (int32_t)v;
	break;

	case FLOAT_FIXED_POINT_10:
		h = (int32_t)(v * 10.0f);
	break;

	case FLOAT_FIXED_POINT_100:
		h = (int32_t)(v * 100.0f);
	break;

	case FLOAT_FIXED_POINT_1000:
		h = (int32_t)(v * 1000.0f);
	break;
	}

	itobase10(d, h);
	*length = strlen(d);
	strcpy((char *)data, d);
}

void setFloat(uint8_t* data, float v, uint8_t prec)
{
	uint32_t f = conFtoI(v);
	uint16_t h;
	
	switch (prec) {
	case FLOAT_FULL_PRECISION:
#ifdef BIG_ENDIAN
		for (int i=0; i<4; i++) {
			data[i] = (f >> (i * 8)) & (uint8_t) 0xff;
		}
#else
		for (int i=3; i>0; i--) {
			data[i] = (f >> ((3-i) * 8)) & (uint8_t) 0xff;
		}
#endif
	break;

	case FLOAT_FIXED_POINT_1:
		h = (int16_t)v;

		for (int i=0; i<2; i++) {
			data[i] = (h >> (i * 8)) & (uint8_t) 0xff;
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
	writeCompleteRegisterSet();
	
	return 1;
}