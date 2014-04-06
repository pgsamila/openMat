#ifndef C3D_PARSER
#define C3D_PARSER

#include <iostream>
#include <fstream>

#include <boost/cstdint.hpp>

#include "windows.h"

class C3dParser {
public:
	std::ifstream input_file_stream;	
	int no_markers_;
	int no_frames_;

public:
	~C3dParser(void);
	void OpenFile(const char* fn);
	void ReadHeader(void);
	void ReadParameterSections(void);
	bool ReadData(double *data_x, double *data_y, double *data_z);
	int GetNoFrames(void);
	int GetNoMarkers(void);
};

#endif