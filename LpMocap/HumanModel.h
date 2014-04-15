#ifndef HUMANMODEL_H
#define HUMANMODEL_H

#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include "pugixml.hpp"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include "C3dParser.h"

class HumanModel 
{
public:
	// body parts
	enum BodyPart{BP_HIP, 
		BP_LEFTUPLEG, BP_LEFTLEG, BP_LEFTFOOT, BP_LEFTTOEBASE, BP_L_TOE_END,
		BP_RIGHTUPLEG, BP_RIGHTLEG, BP_RIGHTFOOT, BP_RIGHTTOEBASE, BP_R_TOE_END,
		BP_SPINE, BP_SPINE1, BP_NECK, BP_HEAD, BP_HEAD_END,
		BP_LEFTSHOULDER, BP_LEFTARM, BP_LEFTFOREARM, BP_LEFTHAND, BP_LEFTHANDTHUMB, BP_L_THUMB_END, BP_L_WRIST_END,
		BP_RIGHTSHOULDER, BP_RIGHTARM, BP_RIGHTFOREARM, BP_RIGHTHAND, BP_RIGHTHANDTHUMB, BP_R_THUMB_END, BP_R_WRIST_END};
	
	// openMatId - body part map
	// idxxx are set to openMatId - 1
	// TODO: perhaps make this dynamically loaded from config file.
	static const int numSensors			= 13;
	static const int idHead				= 0;
	static const int idTorso			= 1;
	static const int idUpperarmLeft		= 2;
	static const int idForearmLeft		= 3;
	static const int idUpperarmRight	= 4;
	static const int idForearmRight		= 5;
	static const int idHip				= 6;
	static const int idThighLeft		= 7;
	static const int idCalfLeft			= 8;
	static const int idThighRight		= 9;
	static const int idCalfRight		= 10;
	static const int idFootLeft			= 11;
	static const int idFootRight		= 12;

public:
	static const int mChannelCount		= 30;

public:
	HumanModel();
	void resetSkeleton();
	void reloadSkeleton() { setupSkeleton(); }
	bool loadHumanModel(const std::string& xmlFile);

	void resetRotationAll();
	void resetOffsetAll();

	void decodeSensorRotation(int pId, float pQ[4]);
	
	bool getEnableHorizontalMovement()			  { return enableHorizontalMovement; }
	void setEnableHorizontalMovement(bool enable) { enableHorizontalMovement = enable; }

	void setRotation(int channel, const Eigen::Quaternion<double> &q);
	void setOffset(int channel, float q[4]);

	void updateBodyData(void);
	void rotateBodyPart(BodyPart bodyPart, const Eigen::Quaternion<double> &rotQ );

	bool StartSaveBinaryMotionData(const char* fn);
	void UpdateSaveBinaryMotionData(void);
	void StopSaveBinaryMotionData(void);
	bool ReadBinaryMotionDataFile(const char* fn);
	bool UpdateModelFromData(void);
	bool ExportToCsvFile(const char* fn);
	double GetPlaybackTime(void);
	double GetRecordTime(void);
	bool IsPlaybackOn(void);
	bool IsRecordOn(void);
	void StartPlayBinaryMotionData(const char* fn, bool loop_playback);
	void StopPlayBinaryMotionData(void);
	void UpdateTimestamp(float t);
	void setOffsetAll(void);
	void ReadC3dFile(const char* fn);

	Eigen::Vector3f getProjectionAngle(int channel_id);
	 
	const char*	GetChannelName	(int pChannel) { return mChannel[pChannel].mName;			}
	int		GetChannelParent	(int pChannel) { return mChannel[pChannel].mParentChannel;	}
	double	GetDataTX			(int pChannel) { return mChannel[pChannel].mT[0];			}
	double	GetDataTY			(int pChannel) { return mChannel[pChannel].mT[1];			}
	double	GetDataTZ			(int pChannel) { return mChannel[pChannel].mT[2];			}
	double	GetDataRX			(int pChannel) { return mChannel[pChannel].mR[0];			}
	double	GetDataRY			(int pChannel) { return mChannel[pChannel].mR[1];			}
	double	GetDataRZ			(int pChannel) { return mChannel[pChannel].mR[2];			}

private:
	bool is_saving_data;
	bool is_playing_data;
	std::ofstream save_data_handle;
	double latest_timestamp;
	int playback_pointer;
	double record_time_offset;
	double play_time_offset;
	bool first_play_step;
	bool first_save_step;
	bool loop_is_on_;

	double lHead;
	double lNeck;
	double lShoulder;
	double lUpperArm;
	double lLowerArm;
	double lHand;
	double lThumb;
	double lUpperSpine;
	double lLowerSpine;
	double lCalf;
	double lThigh;
	double lHip;
	double lFoot;
	double lToe;
	
	bool enableHorizontalMovement;
	
	struct SkeletonNodeInfo
	{
		const char*	mName;
		int			mParentChannel;
		Eigen::Vector3d mDefaultT;
		Eigen::Vector3d mDefaultR;
		Eigen::Vector3d mT;		// PosX,Y,Z
		Eigen::Vector3d mOldT;	// PosX,Y,Z for horizontal movement use
		Eigen::Vector3d mR;		// RotX,Y,Z
		Eigen::Quaternion<double> mDefaultRot;
		Eigen::Quaternion<double> mCurrentRot;
	};

	struct RotationData
	{
		Eigen::Quaternion<double> offset;	// w, x, y, z
		Eigen::Quaternion<double> rot;		// w, x, y, z
	};

	SkeletonNodeInfo	mChannel[mChannelCount];	//!< Channel data & info.
	RotationData		mRotationData[numSensors];	//!< Channel data & info.

	struct MotionData {
		double timestamp;
		SkeletonNodeInfo mChannel[mChannelCount];
	};
	
	std::vector<MotionData> motion_data_list;
	
	double d2r(double d) { return d*0.017453292; }
	double r2d(double r) { return r*57.29577951; }

	void setupSkeleton(); 
};

#endif