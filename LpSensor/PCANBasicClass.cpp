#include "PCANBasicClass.h"

PCANBasicClass::PCANBasicClass()
{
	// Init members
	//
	m_bWasLoaded = false;
	m_hDll = NULL;

	// Loads the API
	//
	LoadAPI();
}

PCANBasicClass::~PCANBasicClass()
{
	// Unloads the API
	//
	UnloadAPI();
}

void PCANBasicClass::LoadAPI()
{
	// Initializes pointers
	//
	InitializePointers();

	// Loads the DLL
	//
	if(!LoadDllHandle())
	{
		std::cout << "[CANEngine] Couldn't load PCANBasic DLL." << std::endl;
		/* ::MessageBox(NULL,"Error: \"Unable to find the DLL PCANBasic.dll!\"","Error!",MB_ICONERROR); */
		return;
	}

	// Loads API functions
	//
	m_pInitialize = (fpInitialize)GetFunction("CAN_Initialize");
	m_pUnInitialize = (fpUninitialize)GetFunction("CAN_Uninitialize");
	m_pReset = (fpReset)GetFunction("CAN_Reset");
	m_pGetStatus = (fpGetStatus)GetFunction("CAN_GetStatus");
	m_pRead = (fpRead)GetFunction("CAN_Read");
	m_pWrite = (fpWrite)GetFunction("CAN_Write");
	m_pFilterMessages = (fpFilterMessages)GetFunction("CAN_FilterMessages");
	m_pGetValue = (fpSetValue)GetFunction("CAN_GetValue");
	m_pSetValue = (fpGetValue)GetFunction("CAN_SetValue");
	m_pGetTextError = (fpGetErrorText)GetFunction("CAN_GetErrorText");

	m_bWasLoaded = m_pInitialize && m_pUnInitialize && m_pReset && m_pGetStatus && m_pRead
		&& m_pWrite && m_pFilterMessages && m_pGetValue && m_pSetValue && m_pGetTextError;
	
	// If the API was not loaded (Wrong version), an error message is shown. 
	//
	if(!m_bWasLoaded)
		std::cout << "[CANEngine] Couldn't initialize PCANBasic DLL functions." << std::endl;

		/* ::MessageBox(NULL,"Error: \"DLL functions could not be loaded!\"","Error!",MB_ICONERROR); */ 
}

void PCANBasicClass::UnloadAPI()
{
	// Frees a loaded DLL
	//
	if(m_hDll != NULL)
		FreeLibrary(m_hDll);
	m_hDll = NULL;	

	// Initializes pointers
	//
	InitializePointers();

	m_bWasLoaded = false;
}

void PCANBasicClass::InitializePointers()
{
	// Initializes thepointers for the PCANBasic functions
	//
	m_pInitialize = NULL;
	m_pUnInitialize = NULL;
	m_pReset = NULL;
	m_pGetStatus = NULL;
	m_pRead = NULL;
	m_pWrite = NULL;
	m_pFilterMessages = NULL;
	m_pGetValue = NULL;
	m_pSetValue = NULL;	
	m_pGetTextError = NULL;
}

bool PCANBasicClass::LoadDllHandle()
{   
	// Was already loaded
	//
	if(m_bWasLoaded)
		return true;

	//Loads Dll
	//
	m_hDll = LoadLibrary("PCANBasic");

	// Return true if the DLL was loaded or false otherwise
	//
	return (m_hDll != NULL);
}

FARPROC PCANBasicClass::GetFunction(char* strName)
{
	// There is no DLL loaded
	//
	if(m_hDll  == NULL)
		return NULL;

	// Gets the address of the given function in the loeaded DLL
	//
	return GetProcAddress(m_hDll, strName);
}

TPCANStatus PCANBasicClass::Initialize(
        TPCANHandle Channel, 
        TPCANBaudrate Btr0Btr1, 
        TPCANType HwType,
		DWORD IOPort, 
		WORD Interrupt)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pInitialize(Channel, Btr0Btr1, HwType, IOPort, Interrupt); 
}

TPCANStatus PCANBasicClass::Uninitialize(
        TPCANHandle Channel)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pUnInitialize(Channel);  
}

TPCANStatus PCANBasicClass::Reset(
     TPCANHandle Channel)
{
    if(!m_bWasLoaded)
        return PCAN_ERROR_UNKNOWN;

    return (TPCANStatus)m_pReset(Channel);
}

TPCANStatus PCANBasicClass::GetStatus(
        TPCANHandle Channel)
{
    if(!m_bWasLoaded)
        return PCAN_ERROR_UNKNOWN;

    return (TPCANStatus)m_pGetStatus(Channel);
}

TPCANStatus PCANBasicClass::Read(
        TPCANHandle Channel, 
        TPCANMsg* MessageBuffer, 
        TPCANTimestamp* TimestampBuffer)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pRead(Channel, MessageBuffer, TimestampBuffer);  
}

TPCANStatus PCANBasicClass::Write(
        TPCANHandle Channel, 
        TPCANMsg* MessageBuffer)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pWrite(Channel, MessageBuffer);  
}

TPCANStatus PCANBasicClass::FilterMessages(
        TPCANHandle Channel, 
        DWORD FromID, 
        DWORD ToID, 
        TPCANMode Mode)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pFilterMessages(Channel, FromID, ToID, Mode);  
}

TPCANStatus PCANBasicClass::GetValue(
        TPCANHandle Channel, 
        TPCANParameter Parameter,  
        void* Buffer, 
        DWORD BufferLength)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pGetValue(Channel, Parameter, Buffer, BufferLength); 
}

TPCANStatus PCANBasicClass::SetValue(
        TPCANHandle Channel,
        TPCANParameter Parameter,
        void* Buffer,
		DWORD BufferLength)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pSetValue(Channel, Parameter, Buffer, BufferLength); 
}

TPCANStatus PCANBasicClass::GetErrorText(
        TPCANStatus Error, 
        WORD Language, 
        LPSTR Buffer)
{
	if(!m_bWasLoaded)
		return PCAN_ERROR_UNKNOWN;

	return (TPCANStatus)m_pGetTextError(Error, Language, Buffer); 
}