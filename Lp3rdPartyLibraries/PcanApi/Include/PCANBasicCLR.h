//  PCANBasicCLR.h
//
//  ~~~~~~~~~~~~
//
//  PCAN-Basic API
//
//  ~~~~~~~~~~~~
//
//  ------------------------------------------------------------------
//  Author : Keneth Wagner
//	Last change: 08.11.2011 Wagner
//
//  Language: C++/CLR
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2012  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com 
//
using namespace System;
using namespace System::Text;
using namespace System::Runtime::InteropServices;

////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////
#define TPCANHandle              System::Byte  // Represents a PCAN hardware channel handle

namespace Peak
{
	namespace Can
	{
		namespace Basic
		{	
			#pragma region Enumerations
			/// <summary>
			/// Represents a PCAN status/error code
			/// </summary>
			[Flags]
			public enum class TPCANStatus : UInt32
			{
				/// <summary>
				/// No error
				/// </summary>
				PCAN_ERROR_OK           = 0x00000,
				/// <summary>
				/// Transmit buffer in CAN controller is full
				/// </summary>
				PCAN_ERROR_XMTFULL      = 0x00001,
				/// <summary>
				/// CAN controller was read too late
				/// </summary>
				PCAN_ERROR_OVERRUN      = 0x00002,  
				/// <summary>
				/// Bus error: an error counter reached the 'light' limit
				/// </summary>
				PCAN_ERROR_BUSLIGHT     = 0x00004,  
				/// <summary>
				/// Bus error: an error counter reached the 'heavy' limit
				/// </summary>
				PCAN_ERROR_BUSHEAVY     = 0x00008,  
				/// <summary>
				/// Bus error: the CAN controller is in bus-off state
				/// </summary>
				PCAN_ERROR_BUSOFF       = 0x00010,  
				/// <summary>
				/// Mask for all bus errors
				/// </summary>
				PCAN_ERROR_ANYBUSERR    = (PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSOFF),
				/// <summary>
				/// Receive queue is empty
				/// </summary>
				PCAN_ERROR_QRCVEMPTY    = 0x00020,  
				/// <summary>
				/// Receive queue was read too late
				/// </summary>
				PCAN_ERROR_QOVERRUN     = 0x00040,  
				/// <summary>
				/// Transmit queue is full
				/// </summary>
				PCAN_ERROR_QXMTFULL     = 0x00080,  
				/// <summary>
				/// Test of the CAN controller hardware registers failed (no hardware found)
				/// </summary>
				PCAN_ERROR_REGTEST      = 0x00100,
				/// <summary>
				/// Driver not loaded
				/// </summary>
				PCAN_ERROR_NODRIVER     = 0x00200,
				/// <summary>
				/// Hardware already in use by a Net
				/// </summary>
				PCAN_ERROR_HWINUSE      = 0x00400,
				/// <summary>
				/// A Client is already connected to the Net
				/// </summary>
				PCAN_ERROR_NETINUSE     = 0x00800,
				/// <summary>
				/// Hardware handle is invalid
				/// </summary>
				PCAN_ERROR_ILLHW        = 0x01400,
				/// <summary>
				/// Net handle is invalid
				/// </summary>
				PCAN_ERROR_ILLNET       = 0x01800,
				/// <summary>
				/// Client handle is invalid
				/// </summary>
				PCAN_ERROR_ILLCLIENT    = 0x01C00,
				/// <summary>
				/// Mask for all handle errors
				/// </summary>
				PCAN_ERROR_ILLHANDLE    = (PCAN_ERROR_ILLHW | PCAN_ERROR_ILLNET | PCAN_ERROR_ILLCLIENT),
				/// <summary>
				/// Resource (FIFO, Client, timeout) cannot be created
				/// </summary>
				PCAN_ERROR_RESOURCE     = 0x02000,
				/// <summary>
				/// Invalid parameter
				/// </summary>
				PCAN_ERROR_ILLPARAMTYPE = 0x04000,
				/// <summary>
				/// Invalid parameter value
				/// </summary>
				PCAN_ERROR_ILLPARAMVAL  = 0x08000,
				/// <summary>
				/// Unknow error
				/// </summary>
				PCAN_ERROR_UNKNOWN      = 0x10000,
				/// <summary>
                /// Invalid data, function, or action.
                /// </summary>
                PCAN_ERROR_ILLDATA      = 0x20000,
				/// <summary>
				/// Channel is not initialized
				/// </summary>
				PCAN_ERROR_INITIALIZE   = 0x40000,
			};

			/// <summary>
			/// Represents a PCAN device
			/// </summary>
			public enum class TPCANDevice : Byte
			{
				/// <summary>
				/// Undefined, unknown or not selected PCAN device value
				/// </summary>
				PCAN_NONE = 0,
				/// <summary>
				/// PCAN Non-Plug&Play devices. NOT USED WITHIN PCAN-Basic API
				/// </summary>
				PCAN_PEAKCAN = 1,
				/// <summary>
				/// PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus
				/// </summary>
				PCAN_ISA = 2,
				/// <summary>
				/// PCAN-Dongle
				/// </summary>
				PCAN_DNG = 3,
				/// <summary>
				/// PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express
				/// </summary>
				PCAN_PCI = 4,
				/// <summary>
				/// PCAN-USB and PCAN-USB Pro
				/// </summary>
				PCAN_USB = 5,
				/// <summary>
				/// PCAN-PC Card
				/// </summary>
				PCAN_PCC = 6
			};

			/// <summary>
			/// Represents a PCAN parameter to be read or set
			/// </summary>
			public enum class TPCANParameter : Byte
			{
				/// <summary>
				/// PCAN-USB device number parameter
				/// </summary>
				PCAN_DEVICE_NUMBER       = 1,
				/// <summary>
				/// PCAN-PC Card 5-Volt power parameter
				/// </summary>
				PCAN_5VOLTS_POWER        = 2,
				/// <summary>
				/// PCAN receive event handler parameter
				/// </summary>
				PCAN_RECEIVE_EVENT       = 3,
				/// <summary>
				/// PCAN message filter parameter
				/// </summary>
				PCAN_MESSAGE_FILTER      = 4,
				/// <summary>
				/// PCAN-Basic API version parameter
				/// </summary>
				PCAN_API_VERSION         = 5,
				/// <summary>
				/// PCAN device channel version parameter
				/// </summary>
				PCAN_CHANNEL_VERSION     = 6,
				/// <summary>
				/// PCAN Reset-On-Busoff parameter
				/// </summary>
				PCAN_BUSOFF_AUTORESET    = 7,
				/// <summary>
				/// PCAN Listen-Only parameter
				/// </summary>
				PCAN_LISTEN_ONLY         = 8,
				/// <summary>
				/// Directory path for trace files
				/// </summary>
				PCAN_LOG_LOCATION        = 9,
				/// <summary>
				/// Debug-Trace activation status
				/// </summary>
				PCAN_LOG_STATUS          = 10,
				/// <summary>
				/// Configuration of the debugged information (LOG_FUNCTION_***)
				/// </summary>
				PCAN_LOG_CONFIGURE       = 11,
				/// <summary>
				/// Custom insertion of text into the log file
				/// </summary>
				PCAN_LOG_TEXT            = 12,
				/// <summary>
				/// Availability status of a PCAN-Channel
				/// </summary>
				PCAN_CHANNEL_CONDITION   = 13,
				/// <summary>
				/// PCAN hardware name parameter
				/// </summary>
				PCAN_HARDWARE_NAME       = 14,
				/// <summary>
				/// Message reception status of a PCAN-Channel
				/// </summary>
				PCAN_RECEIVE_STATUS      = 15,
				/// <summary>
				/// CAN-Controller number of a PCAN-Channel
				/// </summary>
				PCAN_CONTROLLER_NUMBER   = 16,
			};

			/// <summary>
			/// Represents the type of a PCAN message
			/// </summary>
			[Flags]
			public enum class TPCANMessageType : Byte
			{
				/// <summary>
				/// The PCAN message is a CAN Standard Frame (11-bit identifier)
				/// </summary>
				PCAN_MESSAGE_STANDARD  = 0x00,
				/// <summary>
				/// The PCAN message is a CAN Remote-Transfer-Request Frame
				/// </summary>
				PCAN_MESSAGE_RTR       = 0x01,
				/// <summary>
				/// The PCAN message is a CAN Extended Frame (29-bit identifier)
				/// </summary>
				PCAN_MESSAGE_EXTENDED  = 0x02,
				/// <summary>
				/// The PCAN message represents a PCAN status message
				/// </summary>
				PCAN_MESSAGE_STATUS    = 0x80,
			};

			/// <summary>
			/// Represents a PCAN filter mode
			/// </summary>
			public enum class TPCANMode : Byte
			{
				/// <summary>
				/// Mode is Standard (11-bit identifier)
				/// </summary>
				PCAN_MODE_STANDARD = TPCANMessageType::PCAN_MESSAGE_STANDARD,
				/// <summary>
				/// Mode is Extended (29-bit identifier)
				/// </summary>
				PCAN_MODE_EXTENDED = TPCANMessageType::PCAN_MESSAGE_EXTENDED,
			};

			/// <summary>
			/// Represents a PCAN Baud rate register value
			/// </summary>
			public enum class TPCANBaudrate : UInt16
			{
				/// <summary>
				/// 1 MBit/s
				/// </summary>
				PCAN_BAUD_1M      = 0x0014,
				/// <summary>
				/// 800 kBit/s
				/// </summary>
				PCAN_BAUD_800K    = 0x0016,
				/// <summary>
				/// 500 kBit/s
				/// </summary>
				PCAN_BAUD_500K    = 0x001C,
				/// <summary>
				/// 250 kBit/s
				/// </summary>
				PCAN_BAUD_250K    = 0x011C,
				/// <summary>
				/// 125 kBit/s
				/// </summary>
				PCAN_BAUD_125K    = 0x031C,
				/// <summary>
				/// 100 kBit/s
				/// </summary>
				PCAN_BAUD_100K    = 0x432F,
				/// <summary>
				/// 95,238 kBit/s
				/// </summary>
				PCAN_BAUD_95K     = 0xC34E,
				/// <summary>
				/// 83,333 kBit/s
				/// </summary>
				PCAN_BAUD_83K     = 0x4B14,
				/// <summary>
				/// 50 kBit/s
				/// </summary>
				PCAN_BAUD_50K     = 0x472F,
				/// <summary>
				/// 47,619 kBit/s
				/// </summary>
				PCAN_BAUD_47K     = 0x1414,
				/// <summary>
				/// 33,333 kBit/s
				/// </summary>
				PCAN_BAUD_33K     = 0x1D14,
				/// <summary>
				/// 20 kBit/s
				/// </summary>
				PCAN_BAUD_20K     = 0x532F,
				/// <summary>
				/// 10 kBit/s
				/// </summary>
				PCAN_BAUD_10K     = 0x672F,
				/// <summary>
				/// 5 kBit/s
				/// </summary>
				PCAN_BAUD_5K      = 0x7F7F,
			};

			/// <summary>
			/// Represents the type of PCAN (non plug&play) hardware to be initialized
			/// </summary>
			public enum class TPCANType : Byte
			{
				/// <summary>
				/// PCAN-ISA 82C200
				/// </summary>
				PCAN_TYPE_ISA           = 0x01,
				/// <summary>
				/// PCAN-ISA SJA1000
				/// </summary>
				PCAN_TYPE_ISA_SJA       = 0x09,
				/// <summary>
				/// PHYTEC ISA 
				/// </summary>
				PCAN_TYPE_ISA_PHYTEC    = 0x04,
				/// <summary>
				/// PCAN-Dongle 82C200
				/// </summary>
				PCAN_TYPE_DNG           = 0x02,
				/// <summary>
				/// PCAN-Dongle EPP 82C200
				/// </summary>
				PCAN_TYPE_DNG_EPP       = 0x03,
				/// <summary>
				/// PCAN-Dongle SJA1000
				/// </summary>
				PCAN_TYPE_DNG_SJA       = 0x05,
				/// <summary>
				/// PCAN-Dongle EPP SJA1000
				/// </summary>
				PCAN_TYPE_DNG_SJA_EPP   = 0x06,
			};
			#pragma endregion

			#pragma region Strutures
			/// <summary>
			/// Represents a PCAN message
			/// </summary>
			public value struct TPCANMsg
			{
				/// <summary>
				/// 11/29-bit message identifier
				/// </summary>
				UInt32 ID;
				/// <summary>
				/// Type of the message
				/// </summary>
				[MarshalAs(UnmanagedType::U1)]
				TPCANMessageType MSGTYPE;  
				/// <summary>
				/// Data Length Code of the message (0..8)
				/// </summary>
				Byte LEN;      
				/// <summary>
				/// Data of the message (DATA[0]..DATA[7])
				/// </summary>
				[MarshalAs(UnmanagedType::ByValArray, SizeConst = 8)]
				array<Byte>^ DATA;   
			};

			/// <summary>
			/// Represents a timestamp of a received PCAN message.
			/// Total Microseconds = micros + 1000 * millis + 0xFFFFFFFF * 1000 * millis_overflow
			/// </summary>
			public value struct TPCANTimestamp
			{
				/// <summary>
				/// Base-value: milliseconds: 0.. 2^32-1
				/// </summary>
				UInt32 millis;             
				/// <summary>
				/// Roll-arounds of millis
				/// </summary>
				UInt16 millis_overflow;  
				/// <summary>
				/// Microseconds: 0..999
				/// </summary>
				UInt16 micros;           
			};
			#pragma endregion

			#pragma region PCANBasic class
			/// <summary>
			/// PCAN-Basic API class implementation
			/// </summary>
			public ref class PCANBasic abstract sealed
			{
				public:
					#pragma region PCAN-BUS Handles Definition
					/// <summary>
					/// Undefined/default value for a PCAN bus
					/// </summary>
					static const TPCANHandle PCAN_NONEBUS = 0x00;

					/// <summary>
					/// PCAN-ISA interface, channel 1
					/// </summary>
					static const TPCANHandle PCAN_ISABUS1 = 0x21;
					/// <summary>
					/// PCAN-ISA interface, channel 2
					/// </summary>
					static const TPCANHandle PCAN_ISABUS2  = 0x22;
					/// <summary>
					/// PCAN-ISA interface, channel 3
					/// </summary>
					static const TPCANHandle PCAN_ISABUS3 = 0x23;
					/// <summary>
					/// PCAN-ISA interface, channel 4
					/// </summary>
					static const TPCANHandle PCAN_ISABUS4 = 0x24;
					/// <summary>
					/// PCAN-ISA interface, channel 5
					/// </summary>
					static const TPCANHandle PCAN_ISABUS5 = 0x25;
					/// <summary>
					/// PCAN-ISA interface, channel 6
					/// </summary>
					static const TPCANHandle PCAN_ISABUS6 = 0x26;
					/// <summary>
					/// PCAN-ISA interface, channel 7
					/// </summary>
					static const TPCANHandle PCAN_ISABUS7 = 0x27;
					/// <summary>
					/// PCAN-ISA interface, channel 8
					/// </summary>
					static const TPCANHandle PCAN_ISABUS8 = 0x28;
			        
					/// <summary>
					/// PPCAN-Dongle/LPT interface, channel 1 
					/// </summary>
					static const TPCANHandle PCAN_DNGBUS1 = 0x31;

					/// <summary>
					/// PCAN-PCI interface, channel 1
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS1 = 0x41;
					/// <summary>
					/// PCAN-PCI interface, channel 2
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS2 = 0x42;
					/// <summary>
					/// PCAN-PCI interface, channel 3
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS3 = 0x43;
					/// <summary>
					/// PCAN-PCI interface, channel 4
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS4 = 0x44;
					/// <summary>
					/// PCAN-PCI interface, channel 5
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS5 = 0x45;
					/// <summary>
					/// PCAN-PCI interface, channel 6
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS6 = 0x46;
					/// <summary>
					/// PCAN-PCI interface, channel 7
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS7 = 0x47;
					/// <summary>
					/// PCAN-PCI interface, channel 8
					/// </summary>
					static const TPCANHandle PCAN_PCIBUS8 = 0x48;

					/// <summary>
					/// PCAN-USB interface, channel 1
					/// </summary>
					static const TPCANHandle PCAN_USBBUS1 = 0x51;
					/// <summary>
					/// PCAN-USB interface, channel 2
					/// </summary>
					static const TPCANHandle PCAN_USBBUS2 = 0x52;
					/// <summary>
					/// PCAN-USB interface, channel 3
					/// </summary>
					static const TPCANHandle PCAN_USBBUS3 = 0x53;
					/// <summary>
					/// PCAN-USB interface, channel 4
					/// </summary>
					static const TPCANHandle PCAN_USBBUS4 = 0x54;
					/// <summary>
					/// PCAN-USB interface, channel 5
					/// </summary>
					static const TPCANHandle PCAN_USBBUS5 = 0x55;
					/// <summary>
					/// PCAN-USB interface, channel 6
					/// </summary>
					static const TPCANHandle PCAN_USBBUS6 = 0x56;
					/// <summary>
					/// PCAN-USB interface, channel 7
					/// </summary>
					static const TPCANHandle PCAN_USBBUS7 = 0x57;
					/// <summary>
					/// PCAN-USB interface, channel 8
					/// </summary>
					static const TPCANHandle PCAN_USBBUS8 = 0x58;

					/// <summary>
					/// PCAN-PC Card interface, channel 1
					/// </summary>
					static const TPCANHandle PCAN_PCCBUS1 = 0x61;
					/// <summary>
					/// PCAN-PC Card interface, channel 2
					/// </summary>
					static const TPCANHandle PCAN_PCCBUS2 = 0x62;
					#pragma endregion

					#pragma region Parameter values definition
					/// <summary>
					/// The PCAN parameter is not set (inactive)
					/// </summary>
					static const int PCAN_PARAMETER_OFF = 0;
					/// <summary>
					/// The PCAN parameter is set (active)
					/// </summary>
					static const int PCAN_PARAMETER_ON = 1;
					/// <summary>
					/// The PCAN filter is closed. No messages will be received
					/// </summary>
					static const int PCAN_FILTER_CLOSE = 0;
					/// <summary>
					/// The PCAN filter is fully opened. All messages will be received
					/// </summary>
					static const int PCAN_FILTER_OPEN = 1;
					/// <summary>
					/// The PCAN filter is custom configured. Only registered 
					/// messages will be received
					/// </summary>
					static const int PCAN_FILTER_CUSTOM = 2;
					/// <summary>
					/// The PCAN-Channel handle is illegal, or its associated hadware is not available
					/// </summary>
					static const int PCAN_CHANNEL_UNAVAILABLE = 0;
					/// <summary>
					/// The PCAN-Channel handle is available to be connected (Plug&Play Hardware: it means futhermore that the hardware is plugged-in)
					/// </summary>
					static const int PCAN_CHANNEL_AVAILABLE = 1;
					/// <summary>
					/// The PCAN-Channel handle is valid, and is already being used
					/// </summary>
					static const int PCAN_CHANNEL_OCCUPIED = 2;

					/// <summary>
					/// Logs system exceptions / errors
					/// </summary>
					static const int LOG_FUNCTION_DEFAULT = 0x00;
					/// <summary>
					/// Logs the entries to the PCAN-Basic API functions 
					/// </summary>
					static const int LOG_FUNCTION_ENTRY = 0x01;
					/// <summary>
					/// Logs the parameters passed to the PCAN-Basic API functions 
					/// </summary>
					static const int LOG_FUNCTION_PARAMETERS = 0x02;
					/// <summary>
					/// Logs the exits from the PCAN-Basic API functions 
					/// </summary>
					static const int LOG_FUNCTION_LEAVE = 0x04;
					/// <summary>
					/// Logs the CAN messages passed to the CAN_Write function
					/// </summary>
					static const int LOG_FUNCTION_WRITE = 0x08;
					/// <summary>
					/// Logs the CAN messages received within the CAN_Read function
					/// </summary>
					static const int LOG_FUNCTION_READ = 0x10;
					/// <summary>
					/// Logs all possible information within the PCAN-Basic API functions
					/// </summary>
					static const int LOG_FUNCTION_ALL = 0xFFFF;
					#pragma endregion

					#pragma region PCANBasic API Implementation
					/// <summary>
					/// Initializes a PCAN Channel 
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
					/// <param name="HwType">NON PLUG&PLAY: The type of hardware and operation mode</param>
					/// <param name="IOPort">NON PLUG&PLAY: The I/O address for the parallel port</param>
					/// <param name="Interrupt">NON PLUG&PLAY: Interrupt number of the parallel por</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Initialize")]
					static TPCANStatus Initialize(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						[MarshalAs(UnmanagedType::U2)]
						TPCANBaudrate Btr0Btr1,
						[MarshalAs(UnmanagedType::U1)]
						TPCANType HwType, 
						UInt32 IOPort, 
						UInt16 Interrupt);

					/// <summary>
					/// Initializes a PCAN Channel
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
					/// <returns>A TPCANStatus error code</returns>
					static TPCANStatus Initialize(
						TPCANHandle Channel,
						TPCANBaudrate Btr0Btr1)
					{
						return Initialize(Channel, Btr0Btr1, (TPCANType)0, 0, 0);
					}

					/// <summary>
					/// Uninitializes one or all PCAN Channels initialized by CAN_Initialize
					/// </summary>
					/// <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
					/// uninitialize all initialized channels</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Uninitialize")]
					static TPCANStatus Uninitialize(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel);

					/// <summary>
					/// Resets the receive and transmit queues of the PCAN Channel
					/// </summary>
					/// <remarks>A reset of the CAN controller is not performed</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Reset")]
					static TPCANStatus Reset(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel);

					/// <summary>
					/// Gets the current status of a PCAN Channel
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_GetStatus")]
					static TPCANStatus GetStatus(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel);

					/// <summary>
					/// Reads a CAN message from the receive queue of a PCAN Channel
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>
					/// <param name="TimestampBuffer">A TPCANTimestamp structure buffer to get
					/// the reception time of the message</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Read")]
					static TPCANStatus Read(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						TPCANMsg %MessageBuffer,
						TPCANTimestamp %TimestampBuffer);

					/// <summary>
					/// Reads a CAN message from the receive queue of a PCAN Channel
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Read")]
					static TPCANStatus Read(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						TPCANMsg %MessageBuffer);

					/// <summary>
					///  Transmits a CAN message 
					/// </summary>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="MessageBuffer">A TPCANMsg buffer with the message to be sent</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_Write")]
					static TPCANStatus Write(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						TPCANMsg %MessageBuffer);

					/// <summary>
					/// Configures the reception filter
					/// </summary>
					/// <remarks>The message filter will be expanded with every call to 
					/// this function. If it is desired to reset the filter, please use
					/// the 'SetValue' function</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="FromID">The lowest CAN ID to be received</param>
					/// <param name="ToID">The highest CAN ID to be received</param>
					/// <param name="Mode">Message type, Standard (11-bit identifier) or
					/// Extended (29-bit identifier)</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_FilterMessages")]
					static TPCANStatus FilterMessages(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						UInt32 FromID,
						UInt32 ToID,
						[MarshalAs(UnmanagedType::U1)]
						TPCANMode Mode);

					/// <summary>
					/// Retrieves a PCAN Channel value
					/// </summary>
					/// <remarks>Parameters can be present or not according with the kind 
					/// of Hardware (PCAN Channel) being used. If a parameter is not available,
					/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Parameter">The TPCANParameter parameter to get</param>
					/// <param name="StringBuffer">Buffer for the parameter value</param>
					/// <param name="BufferLength">Size in bytes of the buffer</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_GetValue")]
					static TPCANStatus GetValue(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						[MarshalAs(UnmanagedType::U1)]
						TPCANParameter Parameter,
						StringBuilder^ StringBuffer,            
						UInt32 BufferLength);

					/// <summary>
					/// Retrieves a PCAN Channel value
					/// </summary>
					/// <remarks>Parameters can be present or not according with the kind 
					/// of Hardware (PCAN Channel) being used. If a parameter is not available,
					/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Parameter">The TPCANParameter parameter to get</param>
					/// <param name="NumericBuffer">Buffer for the parameter value</param>
					/// <param name="BufferLength">Size in bytes of the buffer</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_GetValue")]
					static TPCANStatus GetValue(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						[MarshalAs(UnmanagedType::U1)]
						TPCANParameter Parameter,
						UInt32 %NumericBuffer,
						UInt32 BufferLength);

					/// <summary>
					/// Configures a PCAN Channel value 
					/// </summary>
					/// <remarks>Parameters can be present or not according with the kind 
					/// of Hardware (PCAN Channel) being used. If a parameter is not available,
					/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Parameter">The TPCANParameter parameter to set</param>
					/// <param name="NumericBuffer">Buffer with the value to be set</param>
					/// <param name="BufferLength">Size in bytes of the buffer</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_SetValue")]
					static TPCANStatus SetValue(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						[MarshalAs(UnmanagedType::U1)]
						TPCANParameter Parameter,
						UInt32% NumericBuffer,
						UInt32 BufferLength);

					/// <summary>
					/// Configures a PCAN Channel value
					/// </summary>
					/// <remarks>Parameters can be present or not according with the kind 
					/// of Hardware (PCAN Channel) being used. If a parameter is not available,
					/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
					/// <param name="Channel">The handle of a PCAN Channel</param>
					/// <param name="Parameter"></param>
					/// <param name="StringBuffer">Buffer with the value to be set</param>
					/// <param name="BufferLength">Size in bytes of the buffer</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_SetValue")]
					static TPCANStatus SetValue(
						[MarshalAs(UnmanagedType::U1)]
						TPCANHandle Channel,
						[MarshalAs(UnmanagedType::U1)]
						TPCANParameter Parameter,
						[MarshalAs(UnmanagedType::LPStr,SizeParamIndex=3)]
						String^ StringBuffer,
						UInt32 BufferLength);

					/// <summary>
					/// Returns a descriptive text of a given TPCANStatus error 
					/// code, in any desired language
					/// </summary>
					/// <remarks>The current languages available for translation are: 
					/// Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
					/// Italian (0x10) and French (0x0C)</remarks>
					/// <param name="Error">A TPCANStatus error code</param>
					/// <param name="Language">Indicates a 'Primary language ID'</param>
					/// <param name="StringBuffer">Buffer for the text (must be at least 256 in length)</param>
					/// <returns>A TPCANStatus error code</returns>
					[DllImport("PCANBasic.dll", EntryPoint = "CAN_GetErrorText")]
					static TPCANStatus GetErrorText(
						[MarshalAs(UnmanagedType::U4)]
						TPCANStatus Error,
						UInt16 Language,
						StringBuilder^ StringBuffer);
					#pragma endregion
			};
			#pragma endregion
		}
	}
}