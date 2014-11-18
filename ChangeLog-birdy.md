# Change Log - Birdman
## LpmsFirmware
- Add LpStopWatch.h for timing debug
- Improve i2c sensor raw datacommunication speed
-- LpmsL3gd20.c 
``` cpp
uint8_t getGyrRawData(float* xAxis, float* yAxis, float* zAxis)
{
    ...
    waitGyrI2CStandbyState();
    readGyrRegister(&data_buffer[0], 6, (uint8_t)L3GD20_OUT_X_L|0x80); 
    /* 
    readGyrRegister(&data_buffer[0], (uint8_t)L3GD20_OUT_X_L);  
    readGyrRegister(&data_buffer[1], (uint8_t)L3GD20_OUT_X_H);
    readGyrRegister(&data_buffer[2], (uint8_t)L3GD20_OUT_Y_L);
    readGyrRegister(&data_buffer[3], (uint8_t)L3GD20_OUT_Y_H);
    readGyrRegister(&data_buffer[4], (uint8_t)L3GD20_OUT_Z_L);
    readGyrRegister(&data_buffer[5], (uint8_t)L3GD20_OUT_Z_H);
    */
    ...
}
```
-- LpmsLsm303dlhc.c
``` cpp
uint8_t getAccRawData(float* xAxis, float* yAxis, float* zAxis)
{
    uint8_t data_buffer[6]; 
    int16_t temp;
    
    readAccRegister(&data_buffer[0], 6, (uint8_t)LSM303DLHC_OUT_X_L_A | 0x80);
    /*
    readAccRegister(&data_buffer[0], (uint8_t)LSM303DLHC_OUT_X_L_A);      
    readAccRegister(&data_buffer[1], (uint8_t)LSM303DLHC_OUT_X_H_A);
    readAccRegister(&data_buffer[2], (uint8_t)LSM303DLHC_OUT_Y_L_A);
    readAccRegister(&data_buffer[3], (uint8_t)LSM303DLHC_OUT_Y_H_A);
    readAccRegister(&data_buffer[4], (uint8_t)LSM303DLHC_OUT_Z_L_A);
    readAccRegister(&data_buffer[5], (uint8_t)LSM303DLHC_OUT_Z_H_A);
    */
``` 
- change readGyrRegister to variable length
-- LpmsL3gd20.c
``` cpp
void readGyrRegister(uint8_t* pBuffer, unsigned char length, uint8_t address)
{
    gyrI2cRead(address, length, pBuffer);
}
```
- change readAccRegister to variable length
-- LpmsLsm303dlhc.c
``` cpp
void readAccRegister(uint8_t* pBuffer, unsigned char length, uint8_t address)
{
    accI2cRead(address, length, pBuffer);
}
```
- Added DCM filter (**TODO: add filter selections: Gyro+Acc, Gyro+Acc+Mag**)
-- LpFilterCVersion.c: `void AHRSupdate(...) `
-- SensorManager.c
``` cpp
void processSensorData(void)
{     
    ...
    //lpFilterUpdate(a, b, g, &q, LPMS_MEASUREMENT_PERIOD, 0, &magNoise, &calibrationData, &lpFilterParam);
    AHRSupdate(a,b,g, LPMS_MEASUREMENT_PERIOD, &q) ;
    ...
}
```
- change sensor timer interrupt to 800Hz (Boom!)
-- LpmsTimebase.h
-- LpmsTimebase.c
``` cpp
void setSystemStepTimer(void)
{
    ...
    int clock = 60000000;       // 60M tick/s -half of system clock of 120Mhz
    int cycle = 800;            // 2.5ms cycle
    int ticks = clock/cycle;    // 150000
    int prescaler = 15;
    int period = ticks/prescaler;
    TIM_TimeBaseStructure.TIM_Period = period-1;//2499;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1; //59;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    ...
}
```
- change default communication frequencies to 10, 25, 50, 400, 800Hz
-- **TODO: fix up LPMS_STREAM_FREQ_XXXHZ_ENABLED headers**
-- **TODO: decide communication frequencies**
- 16bit timing bug. Change to use 32bit float to prevent int overflow @800Hz (see changes in LpmsSensor)
-- SensorManager.c
``` cpp
uint8_t getSensorData(uint8_t* data, uint16_t *l)
{
    ...
        if ((gReg.data[LPMS_CONFIG] & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
                //setUi32t(&(data[o]), (uint32_t)(mT * 1000.0f));
                setFloat(&(data[o]), mT, FLOAT_FULL_PRECISION);
                o = o+4;
    ...
```

## LpSensor
- fix inverted raw mag data sign bug
- fix gyro raw data
-- LpmsSensor.cpp
``` cpp
void LpmsSensor::update(void)
{
    ...
    // Corrects magnetometer measurement
    if ((bt->getConfigReg() & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
        vectSub3x1(&bRaw, &configData.hardIronOffset, &b);      
        matVectMult3(&configData.softIronMatrix, &b, &b);
    } else {
        vectZero3x1(&b);
    }
    ...
    // Corrects gyro measurement
    if ((bt->getConfigReg() & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {      
        matVectMult3(&configData.gyrMisalignMatrix, &gRaw, &g);
        vectAdd3x1(&configData.gyrAlignmentBias, &g, &g);
    } else {
        vectZero3x1(&g);
    }
    //g = gRaw;
    ...
}
```
- set save data precision to 6
-- `void LpmsSensor::checkSaveData(void)`
- fix dataQueue update in setCurrentData function
-- LpmsSensor.cpp
``` cpp
void LpmsSensor::setCurrentData(ImuData d)
{
    ...
    if (dataQueue.size() < 64) { 
        dataQueue.push(d);
    } else {        
        dataQueue.pop();
        dataQueue.push(d);
    }
    ... 
}
```
- change hasImuData to return dataQueue size
-- LpmsSensor.cpp LpmsSensor.h LpmsSensorI.h
``` cpp
int LpmsSensor::hasImuData(void)
{
    return dataQueue.size();
}
```
- 16bit timing bug. Change to use 32bit float to prevent int overflow @800Hz (see changes in LpmsFirmware)
-- **TODO: decide to use uint or float as sensor timestamp**
-- LpmsIoInterface.cpp
``` cpp
bool LpmsIoInterface::parseSensorData(void)
{
    ...
    o = 0;
    if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
        //fromBuffer(oneTx, &l);
        //currentTimestamp = (float) l / 1000.0f;
        fromBuffer(oneTx, o, &currentTimestamp);
        o = o + 4;
    ...
}
```
