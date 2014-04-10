#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H
/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
*/
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <Windows.h>
#include <algorithm>
#include <ctime>
#include  "dirent.h"

inline std::vector<std::string> listFiles(std::string dir)
{
	std::vector<std::string> files;
	std::string ext(".xml");
	DIR *dirc;
	struct dirent *ent;
	if ((dirc = opendir (dir.c_str())) != NULL) 
	{
		/* print all the files and directories within directory */
		while ((ent = readdir (dirc)) != NULL) 
		{
			if (ent->d_type == DT_REG)
			{
				std::string file(ent->d_name);
				if (file.find(ext) != std::string::npos)
				{
					files.push_back(file);
				}
			}
		}
		closedir (dirc);
	} 
	else 
	{
		/* could not open directory */
		perror (""); 
	}
	return files;
}


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
inline const std::string currentDateTime()
{
	SYSTEMTIME lt;
	GetLocalTime(&lt);
	std::stringstream ss;
	ss << lt.wYear << "-" << lt.wMonth << "-" << lt.wDay << " " << lt.wHour << ":" << lt.wMinute << ":" << lt.wSecond;
	return ss.str();
}

inline bool DirectoryExists(const char* path)
{
	DWORD dw = ::GetFileAttributes(path);
	return (dw != INVALID_FILE_ATTRIBUTES &&
	(dw & FILE_ATTRIBUTE_DIRECTORY) != 0);
}


inline bool CreateDirectories ( std::string s, std::string &path )
{ 
	std::vector<std::string> dirs;
	std::string delimiter = "/";
	std::replace( s.begin(), s.end(), '\\', '/');

	if (s.substr(1,2) != ":/")
	{
		std::cout << "Error path" << std::endl;

		return false;
	} 
	else 
	{
		path = s.substr(0,3);
		s.erase(0, path.length());
	}

	// remove trailing slash
	while (!s.empty() && *s.rbegin() == '/')
		s = s.substr (0,s.length()-1);

	if ( s.empty() )
	{
		return true;
	}

	size_t pos = 0;
	std::string token;
	while ( (pos = s.find(delimiter)) != std::string::npos )
	{
		token = s.substr(0, pos);
		dirs.push_back(token);
		s.erase(0, pos + delimiter.length());
	}  
	dirs.push_back( s );
	  
	for ( std::vector<std::string>::iterator it = dirs.begin(); it != dirs.end(); ++it )
	{
		path += *it +"/"; 
		if ( !DirectoryExists(path.c_str()) ){
			if ( !CreateDirectory (path.c_str(), NULL) )
			{			
				std::cout << "create fail: " << path << std::endl; 
				return false;
			}
		}
	}
	return true;
}

#endif