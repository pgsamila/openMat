/***********************************************************************
** Copyright (C) 2013 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "CalibrationData.h"

CalibrationData::CalibrationData(void)
{ 
	LpMatrix3x3f m;
	createIdentity3x3(&m);
	LpVector3f v;
	vectZero3x1(&v);
	
	for (int i=0; i<ABSMAXPITCH; i++) {
		for (int j=0; j<ABSMAXROLL; j++) {
			for (int k=0; k<ABSMAXYAW; k++) {
				for (int l=0; l<3; l++) {
					fieldMap[i][j][k].data[l] = 0.0f;
				}
			}
		}
	}
	
	firmwareVersion = std::string("n/a");
	name = std::string("");
	deviceId = std::string("");
	openMatId = 1;
	deviceType = 0;
	parameterSet = 0;	
	filterMode = 0;
	gyrThresEnable = 0;
	accCompGain = 0.0f;
	accCovariance = 0.0f;
	magCompGain = 0.0f;
	magCovariance = 0.0f;	
	quaternionCalcLocal = 0;
	samplingRate = 0;	
	gyrRange = 0;
	magRange = 0;
	accRange = 0;
	magAutocalibration = 0;
	canStreamFormat = 0;
	canBaudrate = 0;
	selfTestOn = 0;	
	fieldRadius = 0;
	magThreshold = 0;
	magOutOfRange = 0;
	gyrAutocalibration = 0;	
	hardIronOffset = v;
	softIronMatrix = m;
	misalignMatrix = m;
	accBias = v;
	gyrMisalignMatrix = m;
	gyrAlignmentBias = v;
	firmwareVersion = std::string("");
	lowPassFilter = 0;
	canHeartbeat = 0;
	heavemotionEnabled = 0;
	gaitTrackingEnabled = 0;
	linAccCompMode = 0;
	centriCompMode = 0;
	canPointMode = 0;
	canChannelMode = 0;
	canStartId = 0;	
	
	selectedData = 0xffff;
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
	printf("[CalibrationData] Magnetometer misalignment matrix:\n");
	printMatrix(magMAlignmentMatrix);
	printf("[CalibrationData] Magnetometer alignment bias:\n");
	printVector(magMAlignmentBias);
	printf("[CalibrationData] Magnetometer reference:\n");
	printVector(magReference);
	printf("[CalibrationData] Selected data: 0x%d\n", selectedData);
	printf("[CalibrationData] Firmware version %s\n", firmwareVersion.c_str());
}