/****************************************************************************
**
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** OpenMAT is free software: you can redistribute it and/or modify it under 
** the terms of the GNU General Public License as published by the Free 
** Software Foundation, either version 3 of the License, or (at your option) 
** any later version.
** 
** OpenMAT is distributed in the hope that it will be useful, but WITHOUT 
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
** FITNESS FOR A PARTICULAR PURPOSE. See the GNU \ General Public License 
** for more details.
** 
** You should have received a copy of the GNU General Public License along 
** with the OpenMAT library. If not, see <http://www.gnu.org/licenses/>.
**
****************************************************************************/

#include "ObjFileParser.h"

#define OBJ_FILE_COMMENT 0
#define OBJ_FILE_VERTEX_NORMAL 1
#define OBJ_FILE_VERTEX 2
#define OBJ_FILE_NEWLINE 3
#define OBJ_FILE_FACE 4
#define OBJ_FILE_UNKNOWN 5
#define OBJ_FILE_VERTEX_PART 6
#define OBJ_FILE_NORMAL_PART 7

bool ObjFileParser::parse(string filename) 
{
	bool f;
	
	std::vector<Eigen::Vector3f> vertexNormalList;
	std::vector<Eigen::Vector3f> vertexList;
	
	faceList.clear();
	
	fs.open(filename.c_str());
	
	if (fs.is_open() == true) {
		f = true;
		cout << "[ObjFileParser] File " << filename.c_str() <<" opened." << endl;	
	} else {
		cout << "[ObjFileParser] Could not open " << filename.c_str() << endl;	
		return false;
	}
	
	while (fs.is_open() == true && fs.eof() == false) {
		char s[512];	
		int tokenCount;
		Eigen::Vector3f vertex;
		Eigen::Vector3f vertexNormal;
		ObjFace face;
		
		fs.getline(s, 512);
		string l(s);
				
		boost::char_separator<char> sep(" ");
		boost::tokenizer< boost::char_separator<char> > tokens(l, sep);

		int state = OBJ_FILE_NEWLINE;
		BOOST_FOREACH(string t, tokens) {
			switch (state) {
			case OBJ_FILE_NEWLINE:
				if (t == "#") {
					state = OBJ_FILE_COMMENT;
				} else if (t == "vn") {
					state = OBJ_FILE_VERTEX_NORMAL;
					tokenCount = 3;
				} else if (t == "v") {
					state = OBJ_FILE_VERTEX;
					tokenCount = 3;
				} else if (t == "f") {
					state = OBJ_FILE_FACE;
					tokenCount = 3;
				} else {
					state = OBJ_FILE_UNKNOWN;
				}
				break;
			
			case OBJ_FILE_COMMENT:
			case OBJ_FILE_UNKNOWN:
				break;

			case OBJ_FILE_VERTEX_NORMAL:
				if (tokenCount == 0) break;
				
				--tokenCount;
				vertexNormal(tokenCount) = boost::lexical_cast<float>(t);
				break;
				
			case OBJ_FILE_VERTEX:
				if (tokenCount == 0) break;
				
				--tokenCount;
				vertex(tokenCount) = boost::lexical_cast<float>(t);
				break;				

			case OBJ_FILE_FACE:
				if (tokenCount == 0) break;
				
				--tokenCount;
				
				boost::char_separator<char> sep2("//");
				boost::tokenizer< boost::char_separator<char> > tokens2(t, sep2);				
				
				int faceState = OBJ_FILE_VERTEX_PART;
				BOOST_FOREACH(string u, tokens2) {
					int i;
				
					switch (faceState) {
					case OBJ_FILE_VERTEX_PART:
						i = boost::lexical_cast<int>(u);
						try {
							face.vertexList.push_back(vertexList.at(i-1)); 
						} catch (std::out_of_range e) {
							cout << e.what() << endl;
						}
						faceState = OBJ_FILE_NORMAL_PART;
						break;
						
					case OBJ_FILE_NORMAL_PART:
						i = boost::lexical_cast<int>(u);
						try {
							face.faceNormal = vertexNormalList.at(i-1); 
						} catch (std::out_of_range e) {
							cout << e.what() << endl;
						}
						faceState = OBJ_FILE_UNKNOWN;
						break;
					}
				}
				break;
			}
		}
			
		switch (state) {
		case OBJ_FILE_VERTEX_NORMAL:
			if (tokenCount > 0) break;
			vertexNormalList.push_back(vertexNormal);
			break;
							
		case OBJ_FILE_VERTEX:
			if (tokenCount > 0) break;
			vertexList.push_back(vertex);
			break;
					
		case OBJ_FILE_FACE:
			if (tokenCount > 0) break;
			faceList.push_back(face);
			break;
		}
	}

	maxVertex << -9999, -9999, -9999;
	minVertex << 9999, 9999, 9999;
	
	for (unsigned int i=0; i < vertexList.size(); i++) {
		for (unsigned int j=0; j < 3; j++) {
			if (vertexList[i](j) > maxVertex(j)) maxVertex(j) = vertexList[i](j);
			if (vertexList[i](j) < minVertex(j)) minVertex(j) = vertexList[i](j);
		}
	}
	
	centerVertex = (maxVertex - minVertex) * 0.5;
	
	cout << "[ObjFileParser] Processed vertices: " << vertexList.size() << endl;	
	cout << "[ObjFileParser] Processed vertex normals: " << vertexNormalList.size() << endl;		
	cout << "[ObjFileParser] Processed faces: " << faceList.size() << endl;	

	cout << "[ObjFileParser] Center: " << centerVertex(0) << " " << centerVertex(1) << " " << centerVertex(2) << endl;
	
	fs.close();
	
	return f;
}

std::vector<ObjFace> ObjFileParser::getFaceList(void)
{
	return faceList;
}
