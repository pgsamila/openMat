/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CalibrationData.h"

#define DEF_GYR_BIAS_X -0.420771
#define DEF_GYR_BIAS_Y -1.41633	
#define DEF_GYR_BIAS_Z -0.141261		

#define DEF_MAG_BIAS_X -0.420771
#define DEF_MAG_BIAS_Y -1.41633	
#define DEF_MAG_BIAS_Z -0.141261		

#define DEF_ACC_REF_X 0
#define DEF_ACC_REF_Y 0
#define DEF_ACC_REF_Z -1

#define DEF_MAG_REF_X -0.114444
#define DEF_MAG_REF_Y 0.474849
#define DEF_MAG_REF_Z -0.285217

#define DEF_MAG_THRES_X 0.551806
#define DEF_MAG_THRES_Y 0.596217
#define DEF_MAG_THRES_Z 0.501186

#define DEF_GYR_THRES_X 0.910771
#define DEF_GYR_THRES_Y 1.34633
#define DEF_GYR_THRES_Z 1.75126

#define DEF_SAMPLING_RATE 200.0f

CalibrationData::CalibrationData(void)
{ 
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {
				for (int l=0; l<3; l++) {
					fieldMap[i][j][k].data[l] = 0.0f;
				}
			}
		}
	}
	
	selectedData = 0xffff;
	firmwareVersion = std::string("n/a");
}

bool CalibrationData::setDefaultParameters(std::string name, std::string deviceId, int deviceType)
{
	setParameter(PRM_NAME, name);	
	setParameter(PRM_OPENMAT_ID, 0);
	setParameter(PRM_DEVICE_ID, deviceId);
	setParameter(PRM_DEVICE_TYPE, deviceType);
	setParameter(PRM_GYR_THRESHOLD_ENABLED, 0);
	setParameter(PRM_PARAMETER_SET, 0);
	setParameter(PRM_FILTER_MODE, 0);
	setParameter(PRM_GYR_RANGE, 0);
	setParameter(PRM_MAG_RANGE, 0);
	setParameter(PRM_ACC_RANGE, 0);
	setParameter(PRM_SAMPLING_RATE, DEF_SAMPLING_RATE);
	setParameter(PRM_LOCAL_Q, 0);
	setParameter(PRM_ACC_COVARIANCE, 1.0f);
	setParameter(PRM_MAG_COVARIANCE, 1.0f);
	setParameter(PRM_ACC_GAIN, 1.0f);
	setParameter(PRM_MAG_GAIN, 1.0f);
	setParameter(PRM_MAG_AUTOCALIBRATION, 0);
	setParameter(PRM_SELF_TEST, SELECT_SELF_TEST_OFF);
	setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_DISABLED);
	setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_DISABLED);
	
	return true;
}

bool CalibrationData::setParameter(int parameterIndex, std::string parameter)
{
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_NAME:
		name = parameter;
	break;
	
	case PRM_DEVICE_ID:
		deviceId = parameter;
	break;
	}
	
	calibrationMutex.unlock();	
	
	return true;
}

bool CalibrationData::setParameter(int parameterIndex, int parameter)
{
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_OPENMAT_ID:	
		openMatId = parameter;
	break;

	case PRM_DEVICE_TYPE:
		deviceType = parameter;
	break;

	case PRM_GYR_THRESHOLD_ENABLED:
		gyrThresEnable = parameter;
	break;	
		
	case PRM_PARAMETER_SET:
		parameterSet = parameter;
	break;

	case PRM_FILTER_MODE:
		filterMode = parameter;
	break;
	
	case PRM_GYR_RANGE:
		gyrRange = parameter;
	break;
	
	case PRM_MAG_RANGE:
		magRange = parameter;	
	break;
	
	case PRM_ACC_RANGE:
		accRange = parameter;		
	break;
	
	case PRM_LOCAL_Q:	
		quaternionCalcLocal = parameter;
	break;
	
	case PRM_MAG_AUTOCALIBRATION:
		magAutocalibration = parameter;
	break;
	
	case PRM_CAN_STREAM_FORMAT:
		canStreamFormat = parameter;
	break;	
	
	case PRM_CAN_BAUDRATE:
		canBaudrate = parameter;
	break;
	
	case PRM_SAMPLING_RATE:
		samplingRate = parameter;
	break;	
	
	case PRM_SELF_TEST:
		selfTestOn = parameter;
	break;	
	
	case PRM_GYR_AUTOCALIBRATION:		
		gyrAutocalibration = parameter;
	break;
	
	case PRM_SELECT_DATA:
		selectedData = parameter;
	break;
	
	case PRM_LOW_PASS:
		lowPassFilter = parameter;
	break;
	
	/* case PRM_CAN_MAPPING:
		canMapping = parameter;
	break; */
	
	case PRM_CAN_HEARTBEAT:
		canHeartbeat = parameter;
	break;
	
	case PRM_HEAVEMOTION_ENABLED:
		heavemotionEnabled = parameter;
	break;
	
	case PRM_GAIT_TRACKING_ENABLED:
		gaitTrackingEnabled = parameter;
	break;

	case PRM_LIN_ACC_COMP_MODE:
		linAccCompMode = parameter;
	break;
	
	case PRM_CENTRI_COMP_MODE:
		centriCompMode = parameter;
	break;
	
	case PRM_CAN_CHANNEL_MODE:
		canChannelMode = parameter;
	break;
	
	case PRM_CAN_POINT_MODE:
		canPointMode = parameter;
	break;
	
	case PRM_CAN_START_ID:
		canStartId = parameter;
	break;
	}

	calibrationMutex.unlock();
	
	return true;
}

bool CalibrationData::setParameter(int parameterIndex, float parameter)
{
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_ACC_COVARIANCE:
		accCovariance = parameter;
	break;

	case PRM_MAG_COVARIANCE:
		magCovariance = parameter;
	break;

	case PRM_ACC_GAIN:
		accCompGain = parameter;
	break;

	case PRM_MAG_GAIN:
		magCompGain = parameter;
	break;
	}
	
	calibrationMutex.unlock();	
	
	return true;
}

bool CalibrationData::setParameter(int parameterIndex, int *parameter)
{	
	switch (parameterIndex) {
	case PRM_CAN_MAPPING:
		for (int i=0; i<8; ++i) {
			canMapping[i] = parameter[i];
		}
	break;
	}
	
	return true;
}

bool CalibrationData::getParameter(int parameterIndex, std::string *parameter)
{
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_NAME:
		*parameter = name;
	break;
	
	case PRM_DEVICE_ID:
		*parameter = deviceId;
	break;
	
	case PRM_FIRMWARE_VERSION:
		*parameter = firmwareVersion;
	break;
	}
	
	calibrationMutex.unlock();
	
	return true;
}

bool CalibrationData::getParameter(int parameterIndex, int *parameter)
{	
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_OPENMAT_ID:	
		*parameter = openMatId;
	break;

	case PRM_DEVICE_TYPE:	
		*parameter = deviceType;
	break;	

	case PRM_GYR_THRESHOLD_ENABLED:
		*parameter = gyrThresEnable;
	break;	
		
	case PRM_PARAMETER_SET:
		*parameter = parameterSet;
	break;

	case PRM_FILTER_MODE:
		*parameter = filterMode;
	break;
	
	case PRM_GYR_RANGE:
		*parameter = gyrRange;
	break;
	
	case PRM_MAG_RANGE:
		*parameter = magRange;
	break;
	
	case PRM_ACC_RANGE:
		*parameter = accRange;
	break;
	
	case PRM_LOCAL_Q:	
		*parameter = quaternionCalcLocal;
	break;
	
	case PRM_MAG_AUTOCALIBRATION:
		*parameter = magAutocalibration;
	break;

	case PRM_CAN_STREAM_FORMAT:
		*parameter = canStreamFormat;
	break;	
	
	case PRM_CAN_BAUDRATE:
		*parameter = canBaudrate;
	break;	

	case PRM_SAMPLING_RATE:
		*parameter = samplingRate;
	break;

	case PRM_SELF_TEST:
		*parameter = selfTestOn;
	break;	
	
	case PRM_GYR_AUTOCALIBRATION:		
		*parameter = gyrAutocalibration;
	break;
	
	case PRM_SELECT_DATA:
		*parameter = selectedData;
	break;
	
	case PRM_LOW_PASS:
		*parameter = lowPassFilter;
	break;
	
	/* case PRM_CAN_MAPPING:
		*parameter = canMapping;
	break; */
	
	case PRM_CAN_HEARTBEAT:
		*parameter = canHeartbeat;
	break;
	
	case PRM_HEAVEMOTION_ENABLED:
		*parameter = heavemotionEnabled;
	break;
	
	case PRM_GAIT_TRACKING_ENABLED:
		*parameter = gaitTrackingEnabled;
	break;
	
	case PRM_CAN_MAPPING:
		for (int i=0; i<8; ++i) {
			parameter[i] = canMapping[i];
		}
	break;
	
	case PRM_LIN_ACC_COMP_MODE:
		*parameter = linAccCompMode;
	break;
	
	case PRM_CENTRI_COMP_MODE:
		*parameter = centriCompMode;
	break;
	
	case PRM_CAN_CHANNEL_MODE:
		*parameter = canChannelMode;
	break;
	
	case PRM_CAN_POINT_MODE:
		*parameter = canPointMode;
	break;
	
	case PRM_CAN_START_ID:
		*parameter = canStartId;
	break;
	}

	calibrationMutex.unlock();	
	
	return true;
}

bool CalibrationData::getParameter(int parameterIndex, float *parameter)
{
	calibrationMutex.lock();

	switch (parameterIndex) {
	case PRM_ACC_COVARIANCE:
		*parameter = accCovariance;
	break;

	case PRM_MAG_COVARIANCE:
		*parameter = magCovariance;
	break;

	case PRM_ACC_GAIN:
		*parameter = accCompGain;
	break;

	case PRM_MAG_GAIN:
		*parameter = magCompGain;
	break;	
	}
	
	calibrationMutex.unlock();	
	
	return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpVector3f *v) 
{
	boost::char_separator<char> sep(", ");
	int i;
	float x;

	std::string line = node.child_value(tag.c_str());	
	
	if (line == "") return false;
	
	boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
	i = 0;
	BOOST_FOREACH(std::string t, tokens) {
		try {
			x = boost::lexical_cast<float>(t);
		} catch (boost::bad_lexical_cast &) {
			x = 1.0f;
		}
		
		if (i<3) v->data[i] = x;
		++i;
	}

	return true;
}	

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, float *v) 
{
	boost::char_separator<char> sep(", ");
	float x;

	std::string line = node.child_value(tag.c_str());	
	
	if (line == "") return false;
	
	try {
			x = boost::lexical_cast<float>(line);
		} catch (boost::bad_lexical_cast &) {
			x = 1.0f;
		}	
	
	*v = x;
	
	return true;
}	

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpVector4f *v) 
{
	boost::char_separator<char> sep(", ");
	int i;
	float x;

	std::string line = node.child_value(tag.c_str());

	if (line == "") return false;
	
	boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
	i = 0;
	BOOST_FOREACH(std::string t, tokens) {
		try {
			x = boost::lexical_cast<float>(t);
		} catch (boost::bad_lexical_cast &) {
			x = 1.0f;
		}

		if (i<4) v->data[i] = x;		
		++i;
	}

	return true;
}
	
bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpMatrix3x3f *m) 
{
	boost::char_separator<char> sep(", ");	
	int i;	
	float x;

	std::string line = node.child_value(tag.c_str());

	if (line == "") return false;
	
	boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
	i = 0;
	BOOST_FOREACH(std::string t, tokens) {
		try {
			x = boost::lexical_cast<float>(t);
			if (i<9) m->data[i/3][i%3] = x;
		} catch (boost::bad_lexical_cast &) {
			if (i/3 == i%3) {
				m->data[i/3][i%3] = 1.0f;
			} else {
				m->data[i/3][i%3] = 0.0f;
			}
		}
		++i;
	}
	
	return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, int *i) 
{
	std::string line = node.child_value(tag.c_str());

	if (line == "") return false;
	
	try {
		*i = boost::lexical_cast<int>(line);
	} catch (boost::bad_lexical_cast &) { 
		*i = 0;
	}
	
	return true;
}				

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, bool *b) 
{
	std::string line = node.child_value(tag.c_str());				

	if (line == "") return false;
	
	if (line == "true") {
		*b = true;
	} else {
		*b = false;
	}
	
	return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, std::string *s) 
{
	*s = node.child_value(tag.c_str());				

	if (*s == "") return false;
		
	return true;
}

bool CalibrationData::load(std::string fn)
{
	pugi::xml_document document;
	pugi::xml_parse_result r = document.load_file(fn.c_str());

	if (!r) {
		std::cout << "[LpmsSensorManager] Could not open configuration file" << std::endl;
		return false;
	}

	pugi::xml_node configuration = document.child("LpmsControlConfiguration");

	if (!configuration) {
		std::cout << "[LpmsSensorManager] XML file doesn't contain SensorConfiguration tag" << std::endl;
		return false;
	}

	readXML("FieldEstimate", configuration, &fieldRadius);
	readXML("HardIronOffset", configuration, &hardIronOffset);
	readXML("SoftIronMatrix", configuration, &softIronMatrix);
	readXML("MisalignmentMatrix", configuration, &misalignMatrix);
	readXML("AccelerometerBias", configuration, &accBias);
	readXML("GyroMisalignmentMatrix", configuration, &gyrMisalignMatrix);
	readXML("GyroMisalignmentBias", configuration, &gyrAlignmentBias);		
	
	return true;
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, float v)	
{
	std::ostringstream s;
	s << v;
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpVector3f v)	
{
	std::ostringstream s;
	s << v.data[0] << ", " << v.data[1] << ", " << v.data[2];
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpVector4f v)	
{
	std::ostringstream s;
	s << v.data[0] << ", " << v.data[1] << ", " << v.data[2] << ", " << v.data[3];
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpMatrix3x3f m)	
{
	std::ostringstream s;
	s << m.data[0][0] << ", " << m.data[0][1] << ", " << m.data[0][2] << ", " <<
		m.data[1][0] << ", " << m.data[1][1] << ", " << m.data[1][2] << ", " <<
		m.data[2][0] << ", " << m.data[2][1] << ", " << m.data[2][2];
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, bool b)
{
	std::ostringstream s;
	
	if (b == true) {
		s << "true";
	} else {
		s << "false";
	}
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());	
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, std::string s)
{
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.c_str());	
}	

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, int i)
{
	std::ostringstream s;
	s << i;
	
	node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}
	
bool CalibrationData::save(std::string fn)
{
    pugi::xml_document doc;
	pugi::xml_node configuration = doc.append_child("LpmsControlConfiguration");

	writeXML("FieldEstimate", configuration, fieldRadius);
	writeXML("HardIronOffset", configuration, hardIronOffset);
	writeXML("SoftIronMatrix", configuration, softIronMatrix);
	writeXML("MisalignmentMatrix", configuration, misalignMatrix);
	writeXML("AccelerometerBias", configuration, accBias);	
	writeXML("GyroMisalignmentMatrix", configuration, gyrMisalignMatrix);
	writeXML("GyroMisalignmentBias", configuration, gyrAlignmentBias);	
	
	std::cout << "[LpmsSensorManager] Writing configuration file " << fn << std::endl;
	
	if (!doc.save_file(fn.c_str())) {
		std::cout << "[LpmsSensorManager] Writing configuration file " << fn << std::endl;
	}
	
	return true;
}

void printMatrix(LpMatrix3x3f m) 
{
	for (int i=0; i<3; i++) {
		printf("[CalibrationData] ");
		for (int j=0; j<3; j++) {
			printf("%f ", m.data[i][j]);
		}
		printf("\n");
	}
}

void printVector(LpVector3f v) 
{
	printf("[CalibrationData] ");
	for (int i=0; i<3; i++) {
		printf("%f ", v.data[i]);
	}
	printf("\n");
}

void CalibrationData::print(void)
{
	printf("[CalibrationData] DeviceID: %s\n", deviceId.c_str());
	printf("[CalibrationData] OpenMAT ID: %d\n", openMatId);
	printf("[CalibrationData] DeviceType: %d\n", deviceType);
	printf("[CalibrationData] Parameter set: %d\n", parameterSet);	
	printf("[CalibrationData] Filter mode: %d\n", filterMode);
	printf("[CalibrationData] Gyroscope threshold: %d\n", gyrThresEnable);
	printf("[CalibrationData] Sampling rate: %d\n", samplingRate);	
	printf("[CalibrationData] Gyro range: %d\n", gyrRange);
	printf("[CalibrationData] Mag. range: %d\n", magRange);
	printf("[CalibrationData] Acc. range: %d\n", accRange);
	printf("[CalibrationData] CAN Stream format: %d\n", canStreamFormat);
	printf("[CalibrationData] CAN Baudrate: %d\n", canBaudrate);
	printf("[CalibrationData] Field estimate: %f\n", fieldRadius);
	printf("[CalibrationData] Gyr. auto-calibration on / off: %d\n", gyrAutocalibration);
	printf("[CalibrationData] Hard iron offset:\n");
	printVector(hardIronOffset);
	printf("[CalibrationData] Soft iron matrix:\n");
	printMatrix(softIronMatrix);
	printf("[CalibrationData] Misalignment matrix:\n");
	printMatrix(misalignMatrix);
	printf("[CalibrationData] Accelerometer bias:\n");
	printVector(accBias);
	printf("[CalibrationData] Gyroscope misalignment matrix:\n");
	printMatrix(gyrMisalignMatrix);
	printf("[CalibrationData] Gyroscope alignment bias:\n");
	printVector(gyrAlignmentBias);
	printf("[CalibrationData] Selected data: %d\n", selectedData);
	printf("[CalibrationData] Firmware version %s\n", firmwareVersion.c_str());
	printf("[CalibrationData] Gyroscope temp. cal. prm. A:\n");
	printVector(gyrCalA);
	printf("[CalibrationData] Gyroscope temp. cal. prm. B:\n");
	printVector(gyrCalB);
	printf("[CalibrationData] Gyroscope temp. cal. base V:\n");
	printVector(gyrCalBaseV);
	printf("[CalibrationData] Gyroscope temp. cal. base T: %f\n", gyrCalBaseT);
}