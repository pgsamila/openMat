#include "HumanModel.h"

#include <windows.h>
#include <iostream>
#include <iomanip>

#define _USE_MATH_DEFINES

#include <math.h> 
#include <stdlib.h>

#include "quateuler.h"

using namespace std;
using Eigen::Quaternion;

HumanModel::HumanModel():
	 enableHorizontalMovement(false)
{
	lHead		= 18;
	lNeck		= 12;
	lShoulder	= 38;
	lUpperArm	= 28;
	lLowerArm	= 28;
	lHand		= 19;
	lThumb		= 10;
	lUpperSpine	= 38;
	lLowerSpine	= 21;
	lCalf		= 42;
	lThigh		= 49;
	lHip		= 28;
	lFoot		= 20;
	lToe		= 8;
	
	setupSkeleton();
	resetOffsetAll();
	resetRotationAll();
	
	is_saving_data = false;
	playback_pointer = 0;
	record_time_offset = 0;
	play_time_offset = 0;
	first_play_step = false;
	first_save_step = false;
	latest_timestamp = 0;
	is_playing_data = false;
	is_saving_data = false;
	loop_is_on_ = false;
}

void HumanModel::resetSkeleton()
{
	lHead		= 18;
	lNeck		= 12;
	lShoulder	= 38;
	lUpperArm	= 28;
	lLowerArm	= 28;
	lHand		= 19;
	lThumb		= 10;
	lUpperSpine	= 38;
	lLowerSpine	= 21;
	lCalf		= 42;
	lThigh		= 49;
	lHip		= 28;
	lFoot		= 20;
	lToe		= 8;
	setupSkeleton();
}


bool HumanModel::loadHumanModel(const std::string& xmlFile)
{
	cout << "[LpMocap] Parsing human model xml file." << endl; 
	pugi::xml_document doc;
	if (!doc.load_file(xmlFile.c_str())) {
		cout << "[LpMocap] Error loading human model file: " << xmlFile << endl;
		
		// createDefaultSensorXml();	
		// doc.load_file(LPMB_SETTINGS_OUTPUTFILE.c_str());
		
		return false;
	};

	pugi::xml_node bodyParts = doc.child("HumanModel");
    
#ifdef CPP11_FULL_SUPPORT
	for (pugi::xml_node bodyPart: bodyParts.children("BodyPart"))
#else
	BOOST_FOREACH(pugi::xml_node bodyPart, bodyParts.children("BodyPart"))
#endif
	{  
		string bodyPartName(bodyPart.child("Name").child_value()); 
		if (bodyPartName == "Head")
			lHead = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Neck")
			lNeck = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Shoulder")
			lShoulder = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "UpperArm")
			lUpperArm = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "LowerArm")
			lLowerArm = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Hand")
			lHand = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Thumb")
			lThumb = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "UpperSpine")
			lUpperSpine = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "LowerSpine")
			lLowerSpine = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Hip")
			lHip = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Thigh")
			lThigh = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Calf")
			lCalf = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Foot")
			lFoot = atof(bodyPart.child("Length").child_value());
		else if (bodyPartName == "Toe")
			lToe = atof(bodyPart.child("Length").child_value());
	}
    
	cout << "[LpMocap] Done Parsing xml file." << endl; 

	setupSkeleton();

	return true;
}


void HumanModel::setupSkeleton(){ 
	//0:Hips
	mChannel[0].mName = "Hips";
	mChannel[0].mParentChannel = -1;
	mChannel[0].mT[0] = 0;
	mChannel[0].mT[1] = lThigh + lCalf;
	mChannel[0].mT[2] = 0.0;
	mChannel[0].mR[0] = 0.0;
	mChannel[0].mR[1] = 0.0;
	mChannel[0].mR[2] = 0.0;

	//1:LeftUpLeg
	mChannel[1].mName = "LeftUpLeg";
	mChannel[1].mParentChannel = 0;
	mChannel[1].mT[0] = lHip/2;
	mChannel[1].mT[1] = lThigh + lCalf;
	mChannel[1].mT[2] = 0.0;
	mChannel[1].mR[0] = 0.0;
	mChannel[1].mR[1] = 0.0;
	mChannel[1].mR[2] = 0.0;

	//2:LeftLeg
	mChannel[2].mName = "LeftLeg";
	mChannel[2].mParentChannel = 1;
	mChannel[2].mT[0] = lHip/2;
	mChannel[2].mT[1] = lCalf;
	mChannel[2].mT[2] = 0.0;
	mChannel[2].mR[0] = 0.0;
	mChannel[2].mR[1] = 0.0;
	mChannel[2].mR[2] = 0.0;

	//3:LeftFoot
	mChannel[3].mName = "LeftFoot";
	mChannel[3].mParentChannel = 2;
	mChannel[3].mT[0] = lHip/2;
	mChannel[3].mT[1] = 0.0;
	mChannel[3].mT[2] = 0.0;
	mChannel[3].mR[0] = 0.0;
	mChannel[3].mR[1] = 0.0;
	mChannel[3].mR[2] = 0.0;

	//4:LeftToeBase
	mChannel[4].mName = "LeftToeBase";
	mChannel[4].mParentChannel = 3;
	mChannel[4].mT[0] = lHip/2;
	mChannel[4].mT[1] = 0;
	mChannel[4].mT[2] = lFoot;
	mChannel[4].mR[0] = 0.0;
	mChannel[4].mR[1] = 0.0;
	mChannel[4].mR[2] = 0.0;

	//5:L_Toe_End
	mChannel[5].mName = "L_Toe_End";
	mChannel[5].mParentChannel = 4;
	mChannel[5].mT[0] = lHip/2;
	mChannel[5].mT[1] = 0;
	mChannel[5].mT[2] = lFoot+lToe;
	mChannel[5].mR[0] = 0.0;
	mChannel[5].mR[1] = 0.0;
	mChannel[5].mR[2] = 0.0;

	//6:RightUpLeg
	mChannel[6].mName = "RightUpLeg";
	mChannel[6].mParentChannel = 0;
	mChannel[6].mT[0] = -lHip/2;
	mChannel[6].mT[1] = lThigh + lCalf;
	mChannel[6].mT[2] = 0.0;
	mChannel[6].mR[0] = 0.0;
	mChannel[6].mR[1] = 0.0;
	mChannel[6].mR[2] = 0.0;

	//7:RightLeg
	mChannel[7].mName = "RightLeg";
	mChannel[7].mParentChannel = 6;
	mChannel[7].mT[0] = -lHip/2;
	mChannel[7].mT[1] = lCalf;
	mChannel[7].mT[2] = 0.0;
	mChannel[7].mR[0] = 0.0;
	mChannel[7].mR[1] = 0.0;
	mChannel[7].mR[2] = 0.0;

	//8:RightFoot
	mChannel[8].mName = "RightFoot";
	mChannel[8].mParentChannel = 7;
	mChannel[8].mT[0] = -lHip/2;
	mChannel[8].mT[1] = 0;
	mChannel[8].mT[2] = 0.0;
	mChannel[8].mR[0] = 0.0;
	mChannel[8].mR[1] = 0.0;
	mChannel[8].mR[2] = 0.0;

	//9:RightToeBase
	mChannel[9].mName = "RightToeBase";
	mChannel[9].mParentChannel = 8;
	mChannel[9].mT[0] = -lHip/2;
	mChannel[9].mT[1] = 0;
	mChannel[9].mT[2] = lFoot;
	mChannel[9].mR[0] = 0.0;
	mChannel[9].mR[1] = 0.0;
	mChannel[9].mR[2] = 0.0;

	//10:R_Toe_End
	mChannel[10].mName = "R_Toe_End";
	mChannel[10].mParentChannel = 9;
	mChannel[10].mT[0] = -lHip/2;
	mChannel[10].mT[1] = 0;
	mChannel[10].mT[2] = lFoot+lToe;
	mChannel[10].mR[0] = 0.0;
	mChannel[10].mR[1] = 0.0;
	mChannel[10].mR[2] = 0.0;

	//11:Spine
	mChannel[11].mName = "Spine";
	mChannel[11].mParentChannel = 0;
	mChannel[11].mT[0] = 0;
	mChannel[11].mT[1] = lThigh + lCalf;
	mChannel[11].mT[2] = 0.0;
	mChannel[11].mR[0] = 0.0;
	mChannel[11].mR[1] = 0.0;
	mChannel[11].mR[2] = 0.0; 
	//12:Spine1
	mChannel[12].mName = "Spine1";
	mChannel[12].mParentChannel = 11;
	mChannel[12].mT[0] = 0;
	mChannel[12].mT[1] = lLowerSpine + lThigh + lCalf;
	mChannel[12].mT[2] = 0.0;
	mChannel[12].mR[0] = 0.0;
	mChannel[12].mR[1] = 0.0;
	mChannel[12].mR[2] = 0.0;

	//13:Neck
	mChannel[13].mName = "Neck";
	mChannel[13].mParentChannel = 12;
	mChannel[13].mT[0] = 0;
	mChannel[13].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[13].mT[2] = 0.0;
	mChannel[13].mR[0] = 0.0;
	mChannel[13].mR[1] = 0.0;
	mChannel[13].mR[2] = 0.0;

	//14:Head
	mChannel[14].mName = "Head";
	mChannel[14].mParentChannel = 13;
	mChannel[14].mT[0] = 0;
	mChannel[14].mT[1] = lNeck + lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[14].mT[2] = 0.0;
	mChannel[14].mR[0] = 0.0;
	mChannel[14].mR[1] = 0.0;
	mChannel[14].mR[2] = 0.0;

	//15:Head_End
	mChannel[15].mName = "Head_End";
	mChannel[15].mParentChannel = 14;
	mChannel[15].mT[0] = 0;
	mChannel[15].mT[1] = lHead + lNeck + lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[15].mT[2] = 0.0;
	mChannel[15].mR[0] = 0.0;
	mChannel[15].mR[1] = 0.0;
	mChannel[15].mR[2] = 0.0;

	//16:LeftShoulder
	mChannel[16].mName = "LeftShoulder";
	mChannel[16].mParentChannel = 12;
	mChannel[16].mT[0] = 0;
	mChannel[16].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[16].mT[2] = 0.0;
	mChannel[16].mR[0] = 0.0;
	mChannel[16].mR[1] = 0.0;
	mChannel[16].mR[2] = 0.0;

	//17:LeftArm
	mChannel[17].mName = "LeftArm";
	mChannel[17].mParentChannel = 16;
	mChannel[17].mT[0] = lShoulder/2;
	mChannel[17].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[17].mT[2] = 0.0;
	mChannel[17].mR[0] = 0.0;
	mChannel[17].mR[1] = 0.0;
	mChannel[17].mR[2] = 0.0;

	//18:LeftForeArm
	mChannel[18].mName = "LeftForeArm";
	mChannel[18].mParentChannel = 17;
	mChannel[18].mT[0] = lShoulder/2 + lUpperArm;
	mChannel[18].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[18].mT[2] = 0.0;
	mChannel[18].mR[0] = 0.0;
	mChannel[18].mR[1] = 0.0;
	mChannel[18].mR[2] = 0.0;

	//19:LeftHand
	mChannel[19].mName = "LeftHand";
	mChannel[19].mParentChannel = 18;
	mChannel[19].mT[0] = lShoulder/2 + lUpperArm + lLowerArm;
	mChannel[19].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[19].mT[2] = 0.0;
	mChannel[19].mR[0] = 0.0;
	mChannel[19].mR[1] = 0.0;
	mChannel[19].mR[2] = 0.0;

	//20:LeftHandThumb
	mChannel[20].mName = "LeftHandThumb";
	mChannel[20].mParentChannel = 19;
	mChannel[20].mT[0] = lShoulder/2 + lUpperArm + lLowerArm;
	mChannel[20].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[20].mT[2] = 0.0;
	mChannel[20].mR[0] = 0.0;
	mChannel[20].mR[1] = 0.0;
	mChannel[20].mR[2] = 0.0;

	//21:L_Thumb_End
	mChannel[21].mName = "L_Thumb_End";
	mChannel[21].mParentChannel = 20;
	mChannel[21].mT[0] = lShoulder/2 + lUpperArm + lLowerArm;
	mChannel[21].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[21].mT[2] = lThumb;
	mChannel[21].mR[0] = 0.0;
	mChannel[21].mR[1] = 0.0;
	mChannel[21].mR[2] = 0.0;

	//22:L_Wrist_End
	mChannel[22].mName = "L_Wrist_End";
	mChannel[22].mParentChannel = 19;
	mChannel[22].mT[0] = lShoulder/2 + lUpperArm + lLowerArm + lHand;
	mChannel[22].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[22].mT[2] = 0.0;
	mChannel[22].mR[0] = 0.0;
	mChannel[22].mR[1] = 0.0;
	mChannel[22].mR[2] = 0.0;;

	//23:RightShoulder
	mChannel[23].mName = "RightShoulder";
	mChannel[23].mParentChannel = 12;
	mChannel[23].mT[0] = 0;
	mChannel[23].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[23].mT[2] = 0.0;
	mChannel[23].mR[0] = 0.0;
	mChannel[23].mR[1] = 0.0;
	mChannel[23].mR[2] = 0.0;

	//24:RightArm
	mChannel[24].mName = "RightArm";
	mChannel[24].mParentChannel = 23;
	mChannel[24].mT[0] = -lShoulder/2;
	mChannel[24].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[24].mT[2] = 0.0;
	mChannel[24].mR[0] = 0.0;
	mChannel[24].mR[1] = 0.0;
	mChannel[24].mR[2] = 0.0;

	//25:RightForeArm
	mChannel[25].mName = "RightForeArm";
	mChannel[25].mParentChannel = 24;
	mChannel[25].mT[0] = -(lShoulder/2 + lUpperArm) ;
	mChannel[25].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[25].mT[2] = 0.0;
	mChannel[25].mR[0] = 0.0;
	mChannel[25].mR[1] = 0.0;
	mChannel[25].mR[2] = 0.0;

	//26:RightHand
	mChannel[26].mName = "RightHand";
	mChannel[26].mParentChannel = 25;
	mChannel[26].mT[0] = -(lShoulder/2 + lUpperArm + lLowerArm);
	mChannel[26].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[26].mT[2] = 0.0;
	mChannel[26].mR[0] = 0.0;
	mChannel[26].mR[1] = 0.0;
	mChannel[26].mR[2] = 0.0;

	//27:RightHandThumb
	mChannel[27].mName = "RightHandThumb";
	mChannel[27].mParentChannel = 26;
	mChannel[27].mT[0] = -(lShoulder/2 + lUpperArm + lLowerArm);
	mChannel[27].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[27].mT[2] = 0.0;
	mChannel[27].mR[0] = 0.0;
	mChannel[27].mR[1] = 0.0;
	mChannel[27].mR[2] = 0.0;

	//28:R_Thumb_End
	mChannel[28].mName = "R_Thumb_End";
	mChannel[28].mParentChannel = 27;
	mChannel[28].mT[0] = -(lShoulder/2 + lUpperArm + lLowerArm);
	mChannel[28].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[28].mT[2] = lThumb;
	mChannel[28].mR[0] = 0.0;
	mChannel[28].mR[1] = 0.0;
	mChannel[28].mR[2] = 0.0;

	//29:R_Wrist_End
	mChannel[29].mName = "R_Wrist_End";
	mChannel[29].mParentChannel = 26;
	mChannel[29].mT[0] =-(lShoulder/2 + lUpperArm + lLowerArm + lHand);
	mChannel[29].mT[1] = lUpperSpine + lLowerSpine + lThigh + lCalf;
	mChannel[29].mT[2] = 0.0;
	mChannel[29].mR[0] = 0.0;
	mChannel[29].mR[1] = 0.0;
	mChannel[29].mR[2] = 0.0;//90.0002104591;

	// Setup default skeleton data
	for (int i = 0; i < mChannelCount; i++) { 
		mChannel[i].mDefaultT = mChannel[i].mT;
		mChannel[i].mOldT	  = mChannel[i].mT;
		mChannel[i].mDefaultR = mChannel[i].mR; 
		Quaternion<double> qx( cos(d2r(mChannel[i].mDefaultR[0])/2 ), sin( d2r(mChannel[i].mDefaultR[0])/2 ), 0 ,0 );
		Quaternion<double> qy( cos(d2r(mChannel[i].mDefaultR[1])/2 ), 0, sin( d2r(mChannel[i].mDefaultR[1])/2 ) ,0 );
		Quaternion<double> qz( cos(d2r(mChannel[i].mDefaultR[2])/2 ), 0 ,0 , sin( d2r(mChannel[i].mDefaultR[2])/2 ) );
		mChannel[i].mDefaultRot = qz*qy*qx;
	}
}

void HumanModel::resetRotationAll()
{
	for (int i=0; i<numSensors; ++i) {
		mRotationData[i].rot.setIdentity();
	}
}

void HumanModel::decodeSensorRotation(int pId, float pQ[4])
{
	if (pId > 0 && pId <= mChannelCount) {  
		// inverse due to the fact that quaternion from LMPS is inversed
		Quaternion<double> q ( (double)pQ[0], (double)-pQ[1], (double)-pQ[2], (double)-pQ[3] );
		Quaternion<double> qx90(cos(M_PI/4), sin(M_PI/4), 0,0);	// rotation difference between lpms-b & skeleton axes
		Quaternion<double> qRes;
		
		qRes =  q * mRotationData[pId-1].offset.inverse() ;
		qRes = qx90.inverse() * qRes * qx90; 
		
		setRotation(pId-1, qRes);  
	}
}
 
void HumanModel::setRotation(int channel, const Quaternion<double> &q)
{
	mRotationData[channel].rot = q;
}

void HumanModel::resetOffsetAll()
{
	for (int i=0; i<numSensors; ++i)
		mRotationData[i].offset.setIdentity();
}

void HumanModel::setOffset(int channel, float q[4])
{
	// inverse due to the fact that quaternion from LMPS is inversed
	mRotationData[channel].offset.w() = (double) q[0];
	mRotationData[channel].offset.x() = (double)-q[1]; 
	mRotationData[channel].offset.y() = (double)-q[2]; 
	mRotationData[channel].offset.z() = (double)-q[3];
}

void HumanModel::updateBodyData(void)
{
	if (IsPlaybackOn() == true) return;

	if (enableHorizontalMovement) {
		for (int i=0; i < mChannelCount; ++i) { 
			mChannel[i].mOldT = mChannel[i].mT; 
		}
	}

	// LowerBody
	//
	// Hip
	//  
	for (int i = BP_HIP; i <= BP_HIP; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idHip].rot); 

	// 
	// Left Leg
	//
	// Thigh 	
	for(int i = BP_LEFTUPLEG; i <= BP_LEFTUPLEG; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idThighLeft].rot);
	// Calf & Foot
	for(int i = BP_LEFTLEG; i <= BP_LEFTLEG; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idCalfLeft].rot);
		
	for(int i = BP_LEFTFOOT; i <= BP_L_TOE_END; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idFootLeft].rot);
	 
	// 
	// Right Leg
	//
	// Thigh 
	for(int i = BP_RIGHTUPLEG; i <= BP_RIGHTUPLEG; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idThighRight].rot);
	// Calf & Foot 
	for(int i = BP_RIGHTLEG; i <= BP_RIGHTLEG; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idCalfRight].rot);
	for(int i = BP_RIGHTFOOT; i <= BP_R_TOE_END; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idFootRight].rot);

	// Translate lower body to ground
	if (enableHorizontalMovement)
	{
		double xOffset=0, yOffset=0, zOffset=0;
		if ( mChannel[BP_LEFTFOOT].mT[1] >  mChannel[BP_RIGHTFOOT].mT[1] )
		{
			// Right leg lower
			xOffset = mChannel[BP_RIGHTFOOT].mT[0] - mChannel[BP_RIGHTFOOT].mOldT[0];
			zOffset = mChannel[BP_RIGHTFOOT].mT[2] - mChannel[BP_RIGHTFOOT].mOldT[2];
		} 
		else if ( mChannel[BP_LEFTFOOT].mT[1] <  mChannel[BP_RIGHTFOOT].mT[1] )
		{
			// Left leg lower
			xOffset = mChannel[BP_LEFTFOOT].mT[0] - mChannel[BP_LEFTFOOT].mOldT[0];
			zOffset = mChannel[BP_LEFTFOOT].mT[2] - mChannel[BP_LEFTFOOT].mOldT[2];
		}
	
		yOffset = min(mChannel[BP_LEFTFOOT].mT[1], mChannel[BP_RIGHTFOOT].mT[1]); 

		for(int i = BP_HIP; i <= BP_R_TOE_END; i++){
			mChannel[i].mT[0] -= xOffset;
			mChannel[i].mT[1] -= yOffset;
			mChannel[i].mT[2] -= zOffset;
		}
	} 
	else 
	{
		//double lOffset = min(mChannel[BP_LEFTFOOT].mT[1], mChannel[BP_RIGHTFOOT].mT[1]);
		double lOffset=0;
		double lLeftOffset=mChannel[BP_LEFTFOOT].mT[1]; 
        double lRightOffset=mChannel[BP_RIGHTFOOT].mT[1]; 
        bool isLToeDown=false, isRToeDown=false;
        if (mChannel[BP_LEFTTOEBASE].mT[1] <= lLeftOffset){
            lLeftOffset = mChannel[BP_LEFTTOEBASE].mT[1];
            isLToeDown = true;
        }
        if (mChannel[BP_RIGHTTOEBASE].mT[1] <= lRightOffset){
            lRightOffset = mChannel[BP_RIGHTTOEBASE].mT[1];
            isRToeDown = true;
        }

        if (lLeftOffset < lRightOffset) {
            lOffset = lLeftOffset;
            if (isLToeDown) {
                double angle = atan2(mChannel[BP_L_TOE_END].mT[2]-mChannel[BP_LEFTTOEBASE].mT[2], mChannel[BP_L_TOE_END].mT[0]-mChannel[BP_LEFTTOEBASE].mT[0]);
                mChannel[BP_L_TOE_END].mT[0] = mChannel[BP_LEFTTOEBASE].mT[0]+lToe*cos(angle);
                mChannel[BP_L_TOE_END].mT[1]=0;
                mChannel[BP_L_TOE_END].mT[2] = mChannel[BP_LEFTTOEBASE].mT[2]+lToe*sin(angle);

            }
        } else {
            lOffset = lRightOffset;
            if (isRToeDown){
                double angle = atan2(mChannel[BP_R_TOE_END].mT[2]-mChannel[BP_RIGHTTOEBASE].mT[2], mChannel[BP_R_TOE_END].mT[0]-mChannel[BP_RIGHTTOEBASE].mT[0]);
                mChannel[BP_R_TOE_END].mT[0] = mChannel[BP_RIGHTTOEBASE].mT[0]+lToe*cos(angle);
                mChannel[BP_R_TOE_END].mT[1]=0;
                mChannel[BP_R_TOE_END].mT[2] = mChannel[BP_RIGHTTOEBASE].mT[2]+lToe*sin(angle);
            }

        }
		
		for(int i = BP_HIP; i <= BP_R_TOE_END; i++){
			mChannel[i].mT[1]-=lOffset;
		}
	}

	// UpperBody
	// 
	// Torso
	//
	for(int i = BP_SPINE; i <= BP_LEFTSHOULDER; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idTorso].rot); 
	for(int i = BP_RIGHTSHOULDER; i <= BP_RIGHTSHOULDER; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idTorso].rot);

	//
	// Head
	//
	for(int i = BP_HEAD; i <= BP_HEAD_END; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idHead].rot);

	// 
	// Left arm 
	//
	// upper arm   
	for(int i = BP_LEFTARM; i <= BP_LEFTARM; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idUpperarmLeft].rot);
	
	// fore arm  	
	for(int i = BP_LEFTFOREARM; i <= BP_L_WRIST_END; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idForearmLeft].rot);
	
	// 
	// Right arm
	//
	// upper arm 
	for(int i = BP_RIGHTARM; i <= BP_RIGHTARM; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idUpperarmRight].rot);
	// fore arm 
	for(int i = BP_RIGHTFOREARM; i <= BP_R_WRIST_END; i++)
		rotateBodyPart((BodyPart)i, mRotationData[idForearmRight].rot);
}


void HumanModel::rotateBodyPart(BodyPart bodyPart, const Quaternion<double> &rot )
{
	int parentId = mChannel[bodyPart].mParentChannel; 

	//
	// Translation
	//
	if (parentId != -1){ 
		// Translate current body part vector to origin		
		Eigen::Vector3d orgT = mChannel[bodyPart].mDefaultT - mChannel[parentId].mDefaultT;

		// Rotate based on parent's rotation 
		Eigen::Vector3d res = mChannel[parentId].mCurrentRot._transformVector(orgT);

		// Translate to parent's current position
		mChannel[bodyPart].mT = res + mChannel[parentId].mT;
	}

	// 
	// Rotation
	// 
	/* mChannel[bodyPart].mCurrentRot = rot ;
	double lR[3];
	quaternion2Euler( mChannel[bodyPart].mCurrentRot*mChannel[bodyPart].mDefaultRot, lR );
	mChannel[bodyPart].mR[0] = r2d(lR[2]);
	mChannel[bodyPart].mR[1] = r2d(lR[1]);
	mChannel[bodyPart].mR[2] = r2d(lR[0]); */
	
	getProjectionAngle(bodyPart, mChannel[bodyPart].mCurrentRot*mChannel[bodyPart].mDefaultRot);
}

void HumanModel::getProjectionAngle(int channel_id, const Quaternion<double> &rot)
{
	Eigen::Vector3d xv;
	Eigen::Vector3d yv;
	Eigen::Vector3d zv;	
	Eigen::Vector3d pv;
	Eigen::Vector3d v;
	Eigen::Matrix3d rm;
	
	float r2d = 57.2958f;	
		
	xv << 1, 0, 0;
	yv << 0, 1, 0;
	zv << 0, 0, 1;
	
	rm = rot.toRotationMatrix();
	
	v = rm * yv;
	pv = v - (v.dot(xv) * xv);
	mChannel[channel_id].mR[0] = acos(pv.dot(zv)) * r2d;

	v = rm * zv;
	pv = v - (v.dot(yv) * yv);
	mChannel[channel_id].mR[1] = acos(pv.dot(xv)) * r2d;

	v = rm * yv;
	pv = v - (v.dot(zv) * zv);
	mChannel[channel_id].mR[2] = acos(pv.dot(xv)) * r2d;
}

bool HumanModel::StartSaveBinaryMotionData(const char* fn)
{	
	save_data_handle.open(fn, std::ios::out /* | std::ios::binary */);

	if (save_data_handle.is_open() == true) {
		is_saving_data = true;
		first_save_step = true;

		save_data_handle << "TIMESTAMP, ";
		save_data_handle << "HIP TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTUPLEG TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTLEG TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTFOOT TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTTOEBASE TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "L_TOE_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTUPLEG TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTLEG TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTFOOT TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTTOEBASE TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "R_TOE_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "SPINE TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "SPINE1 TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "NECK TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "HEAD TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "HEAD_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTSHOULDER TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTARM TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTFOREARM TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTHAND TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "LEFTHANDTHUMB TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "L_THUMB_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "L_WRIST_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTSHOULDER TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTARM TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTFOREARM TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTHAND TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "RIGHTHANDTHUMB TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "R_THUMB_END TX, TY, TZ, RX, RY, RZ, ";
		save_data_handle << "R_WRIST_END TX, TY, TZ, RX, RY, RZ, " << std::endl;

		return true;
	}
		
	return false;
}

void HumanModel::UpdateSaveBinaryMotionData(void)
{
	if (is_saving_data == true && save_data_handle.is_open() == true) {
		if (first_save_step == true) {
			record_time_offset = latest_timestamp;
			first_save_step = false;
		}

		save_data_handle << (latest_timestamp - record_time_offset);
		for (int i = 0; i < mChannelCount; i++) {
			save_data_handle << "," << GetDataTX(i) << "," << GetDataTY(i) << "," << GetDataTZ(i) 
				<< "," << GetDataRX(i) << "," << GetDataRY(i) << "," << GetDataRZ(i);
		}

		save_data_handle << std::endl;
	}
}

void HumanModel::StopSaveBinaryMotionData(void)
{
	if (is_saving_data == false) return;
	
	is_saving_data = false;
	
	if (save_data_handle != NULL) save_data_handle.close();
}

void HumanModel::ReadC3dFile(const char* fn) 
{
	C3dParser c3d_parser;
	bool f;
	
	motion_data_list.clear();
	
	c3d_parser.OpenFile(fn);
	
	int no_frames = c3d_parser.GetNoFrames();
	
	for (int i=0; i<no_frames; ++i) {
		double x=0, y=0, z=0;	
		MotionData single_motion_data;	
			
		f = c3d_parser.ReadData(&x, &y, &z);			
		if (f == false) return;
		
		single_motion_data.timestamp = 33 * (double) i;
	
		for (int j=0; j<mChannelCount; ++j) {
			f = c3d_parser.ReadData(&x, &y, &z);
			
			if (f == false) return;

			single_motion_data.mChannel[j].mT[0] = x * -0.1; 
			single_motion_data.mChannel[j].mT[1] = z * 0.1;
			single_motion_data.mChannel[j].mT[2] = y * 0.1;

			single_motion_data.mChannel[j].mR[0] = 0;
			single_motion_data.mChannel[j].mR[1] = 0;
			single_motion_data.mChannel[j].mR[2] = 0;
		}
		
		motion_data_list.push_back(single_motion_data);
	}
}

bool HumanModel::ReadBinaryMotionDataFile(const char* fn) 
{
	std::string line;
	MotionData single_motion_data;
	
	std::ifstream motion_data_file(fn, std::ios::in /* | std::ios::binary */);	

	motion_data_list.clear();

	if (motion_data_file.is_open()) {
		if (motion_data_file.good()) getline(motion_data_file, line);
		
		while (motion_data_file.good()) {
			getline(motion_data_file, line);
			
			boost::char_separator<char> sep(", ");
			boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
			boost::tokenizer<boost::char_separator<char>>::iterator ti;
			
			if (tokens.begin() == tokens.end()) break;
			
			ti = tokens.begin();
			
			single_motion_data.timestamp = boost::lexical_cast<double>(*ti);
			if (ti != tokens.end()) ++ti;

			for (int i = 0; i < mChannelCount; i++) {
				single_motion_data.mChannel[i].mT[0] = boost::lexical_cast<double>(*ti); 
				if (ti != tokens.end()) ++ti;
				single_motion_data.mChannel[i].mT[1] = boost::lexical_cast<double>(*ti);
				if (ti != tokens.end()) ++ti;
				single_motion_data.mChannel[i].mT[2] = boost::lexical_cast<double>(*ti);
				if (ti != tokens.end()) ++ti;
			
				// single_motion_data.mChannel[i].mR[0] = boost::lexical_cast<double>(*ti);
				if (ti != tokens.end()) ++ti;
				// single_motion_data.mChannel[i].mR[1] = boost::lexical_cast<double>(*ti);
				if (ti != tokens.end()) ++ti;
				// single_motion_data.mChannel[i].mR[2] = boost::lexical_cast<double>(*ti);
				if (ti != tokens.end()) ++ti;
			}

			motion_data_list.push_back(single_motion_data);
		}
		
		motion_data_file.close();		
	} else {
		std::cout << "[LpMocap] Could not open file " << fn << endl;
		
		return false;
	}
	
	return true;
}

void HumanModel::StartPlayBinaryMotionData(const char* fn, bool loop_playback)
{
	playback_pointer = 0;
	first_play_step = true;
	is_playing_data = true;
	loop_is_on_ = loop_playback;
}

void HumanModel::StopPlayBinaryMotionData(void)
{
	is_playing_data = false;
}

bool HumanModel::UpdateModelFromData(void) 
{
	MotionData single_motion_data;
	double avg_alpha = 1.0;

	if (is_playing_data == false) return false;

	if (first_play_step == true) {
		play_time_offset = latest_timestamp;
		first_play_step = false;
	}

	if (playback_pointer < (int) motion_data_list.size()) {
		single_motion_data = motion_data_list[playback_pointer];
	} else {
		playback_pointer = 0;
		single_motion_data = motion_data_list[playback_pointer];
		first_play_step = true;
		if (loop_is_on_ == false) is_playing_data = false;
	}
		
	if ((latest_timestamp - play_time_offset) > single_motion_data.timestamp) {
		for (int i = 0; i < mChannelCount; i++) {
			mChannel[i].mT[0] = mChannel[i].mT[0] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mT[0] * avg_alpha;
			mChannel[i].mT[1] = mChannel[i].mT[1] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mT[1] * avg_alpha;
			mChannel[i].mT[2] = mChannel[i].mT[2] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mT[2] * avg_alpha;	
			
			mChannel[i].mR[0] = mChannel[i].mR[0] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mR[0] * avg_alpha;
			mChannel[i].mR[1] = mChannel[i].mR[1] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mR[1] * avg_alpha;
			mChannel[i].mR[2] = mChannel[i].mR[2] * (1.0 - avg_alpha) + single_motion_data.mChannel[i].mR[2] * avg_alpha;
		}
		++playback_pointer;
	}

	updateBodyData();
	
	return true;
}

bool HumanModel::ExportToCsvFile(const char* fn)
{
	ofstream csv_file_handle;
	
	csv_file_handle.open(fn);	
	
	if (csv_file_handle.is_open() == false) return false;	
	
	csv_file_handle << "TIMESTAMP, ";
	csv_file_handle << "HIP TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTUPLEG TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTLEG TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTFOOT TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTTOEBASE TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "L_TOE_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTUPLEG TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTLEG TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTFOOT TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTTOEBASE TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "R_TOE_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "SPINE TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "SPINE1 TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "NECK TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "HEAD TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "HEAD_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTSHOULDER TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTARM TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTFOREARM TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTHAND TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "LEFTHANDTHUMB TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "L_THUMB_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "L_WRIST_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTSHOULDER TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTARM TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTFOREARM TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTHAND TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "RIGHTHANDTHUMB TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "R_THUMB_END TX, TY, TZ, RX, RY, RZ, ";
	csv_file_handle << "R_WRIST_END TX, TY, TZ, RX, RY, RZ" << std::endl;
	
	for (unsigned int i=0; i<motion_data_list.size(); ++i) {
		csv_file_handle << motion_data_list[i].timestamp;
		for (int j=0; j<mChannelCount; j++) {
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mT[0];
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mT[1];
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mT[2];
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mR[0];
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mR[1];
			csv_file_handle << ", " << motion_data_list[i].mChannel[j].mR[2];
		}
	}

	return true;
}

double HumanModel::GetPlaybackTime(void)
{
	if (IsPlaybackOn() == true) {
		return latest_timestamp - play_time_offset;
	} else {
		return 0;
	}
}

double HumanModel::GetRecordTime(void)
{
	if (IsRecordOn() == true) {
		return latest_timestamp - record_time_offset;
	} else {
		return 0;
	}
}

bool HumanModel::IsPlaybackOn(void)
{
	return is_playing_data;
}

bool HumanModel::IsRecordOn(void)
{
	return is_saving_data;
}

void HumanModel::UpdateTimestamp(float t)
{
	latest_timestamp = t;
}