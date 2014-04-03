Hello there, welcome to OpenMAT, the open motion analysis toolkit.

For licensing information, credits etc., please go to our Wiki:
https://bitbucket.org/lpresearch/openmat 

This directory contains the following folders:

CORE APPLICATIONS
- LpSensor: The core library to manage communication with LPMS devices. Most applications in this directory use this library.
- LpmsControl: An application to control and use LPMS devices.
- LpMocap: LpMocap human motion capture software.

EXAMPLE APPLICATIONS
- LpmsSimpleExample: A simple example on how to use the LpSensor library
- LpmsSanAngeles: Virtual reality application demo using an LPMS device for viewpoint control in a 3D environment
- LpmsBNativeAndroidLibrary: Java class for acquiring data from LPMS-B on an Android device

SUPPORT COMPONENTS
- OpenMATCommon: Common classes for the whole toolkit
- OpenMATInstaller: Script to build Windows installer based on NSIS
- LpSensorCWrapper: C language wrapper for LpSensor
- LpSensorCWrapperTest: Simple test application for C language wrapper

SENSOR FIRMWARE
- LpmsFirmware: Open-source version of the LPMS firmware
- LpmsIAP: In-application programmer for LPMS