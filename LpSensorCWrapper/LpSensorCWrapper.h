#ifdef __cplusplus
extern "C" {
#endif

#ifndef LABVIEW_INTERFACE
#define LABVIEW_INTERFACE

#include "windows.h"

#ifdef DLL_EXPORT
	#define LPMS_API __declspec(dllexport)
#else
	#define LPMS_API __declspec(dllimport)
#endif

LPMS_API void APIENTRY initializeLpms(void);

LPMS_API void APIENTRY connectToLpmsB(const char* deviceId);
LPMS_API void APIENTRY connectToLpmsCU(const char* deviceId);

LPMS_API float APIENTRY getQuaternionW(void);
LPMS_API float APIENTRY getQuaternionX(void);
LPMS_API float APIENTRY getQuaternionY(void);
LPMS_API float APIENTRY getQuaternionZ(void);

LPMS_API float APIENTRY getEulerX(void);
LPMS_API float APIENTRY getEulerY(void);
LPMS_API float APIENTRY getEulerZ(void);

LPMS_API float APIENTRY getGyrX(void);
LPMS_API float APIENTRY getGyrY(void);
LPMS_API float APIENTRY getGyrZ(void);

LPMS_API float APIENTRY getAccX(void);
LPMS_API float APIENTRY getAccY(void);
LPMS_API float APIENTRY getAccZ(void);

LPMS_API float APIENTRY getMagX(void);
LPMS_API float APIENTRY getMagY(void);
LPMS_API float APIENTRY getMagZ(void);

LPMS_API float APIENTRY getLinAccX(void);
LPMS_API float APIENTRY getLinAccY(void);
LPMS_API float APIENTRY getLinAccZ(void);

LPMS_API void APIENTRY disconnectLpms(void);

#endif

#ifdef __cplusplus
}
#endif