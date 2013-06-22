'  PCANBasic.cs
'
'  ~~~~~~~~~~~~
'
'  PCAN-Basic API
'
'  ~~~~~~~~~~~~
'
'  ------------------------------------------------------------------
'  Author : Keneth Wagner
'  Last change: 08.11.2011 Wagner
'
'  Language: VB .NET
'  ------------------------------------------------------------------
'
'  Copyright (C) 1999-2012  PEAK-System Technik GmbH, Darmstadt
'  more Info at http://www.peak-system.com 
'
Imports System
Imports System.Text
Imports System.Runtime.InteropServices

Imports TPCANHandle = System.Byte

Namespace Peak.Can.Basic
#Region "Enumerations"
    ''' <summary>
    ''' Represents a PCAN status/error code
    ''' </summary>
    <Flags()> _
    Public Enum TPCANStatus As UInt32
        ''' <summary>
        ''' No error
        ''' </summary>
        PCAN_ERROR_OK = &H0
        ''' <summary>
        ''' Transmit buffer in CAN controller is full
        ''' </summary>
        PCAN_ERROR_XMTFULL = &H1
        ''' <summary>
        ''' CAN controller was read too late
        ''' </summary>
        PCAN_ERROR_OVERRUN = &H2
        ''' <summary>
        ''' Bus error: an error counter reached the 'light' limit
        ''' </summary>
        PCAN_ERROR_BUSLIGHT = &H4
        ''' <summary>
        ''' Bus error: an error counter reached the 'heavy' limit
        ''' </summary>
        PCAN_ERROR_BUSHEAVY = &H8
        ''' <summary>
        ''' Bus error: the CAN controller is in bus-off state
        ''' </summary>
        PCAN_ERROR_BUSOFF = &H10
        ''' <summary>
        ''' Mask for all bus errors
        ''' </summary>
        PCAN_ERROR_ANYBUSERR = (PCAN_ERROR_BUSLIGHT Or PCAN_ERROR_BUSHEAVY Or PCAN_ERROR_BUSOFF)
        ''' <summary>
        ''' Receive queue is empty
        ''' </summary>
        PCAN_ERROR_QRCVEMPTY = &H20
        ''' <summary>
        ''' Receive queue was read too late
        ''' </summary>
        PCAN_ERROR_QOVERRUN = &H40
        ''' <summary>
        ''' Transmit queue is full
        ''' </summary>
        PCAN_ERROR_QXMTFULL = &H80
        ''' <summary>
        ''' Test of the CAN controller hardware registers failed (no hardware found)
        ''' </summary>
        PCAN_ERROR_REGTEST = &H100
        ''' <summary>
        ''' Driver not loaded
        ''' </summary>
        PCAN_ERROR_NODRIVER = &H200
        ''' <summary>
        ''' Hardware already in use by a Net
        ''' </summary>
        PCAN_ERROR_HWINUSE = &H400
        ''' <summary>
        ''' A Client is already connected to the Net
        ''' </summary>
        PCAN_ERROR_NETINUSE = &H800
        ''' <summary>
        ''' Hardware handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLHW = &H1400
        ''' <summary>
        ''' Net handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLNET = &H1800
        ''' <summary>
        ''' Client handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLCLIENT = &H1C00
        ''' <summary>
        ''' Mask for all handle errors
        ''' </summary>
        PCAN_ERROR_ILLHANDLE = (PCAN_ERROR_ILLHW Or PCAN_ERROR_ILLNET Or PCAN_ERROR_ILLCLIENT)
        ''' <summary>
        ''' Resource (FIFO, Client, timeout) cannot be created
        ''' </summary>
        PCAN_ERROR_RESOURCE = &H2000
        ''' <summary>
        ''' Invalid parameter
        ''' </summary>
        PCAN_ERROR_ILLPARAMTYPE = &H4000
        ''' <summary>
        ''' Invalid parameter value
        ''' </summary>
        PCAN_ERROR_ILLPARAMVAL = &H8000
        ''' <summary>
        ''' Unknow error
        ''' </summary>
        PCAN_ERROR_UNKNOWN = &H10000
        ''' <summary>
        ''' Invalid data, function, or action.
        ''' </summary>
        PCAN_ERROR_ILLDATA = &H20000
        ''' <summary>
        ''' Channel is not initialized
        ''' </summary>
        PCAN_ERROR_INITIALIZE = &H40000
    End Enum

    ''' <summary>
    ''' Represents a PCAN device
    ''' </summary>
    Public Enum TPCANDevice As Byte
        ''' <summary>
        ''' Undefined, unknown or not selected PCAN device value
        ''' </summary>
        PCAN_NONE = 0
        ''' <summary>
        ''' PCAN Non-Plug And Play devices. NOT USED WITHIN PCAN-Basic API
        ''' </summary>        
        PCAN_PEAKCAN = 1
        ''' <summary>
        ''' PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus
        ''' </summary>
        PCAN_ISA = 2
        ''' <summary>
        ''' PCAN-Dongle
        ''' </summary>
        PCAN_DNG = 3
        ''' <summary>
        ''' PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express
        ''' </summary>
        PCAN_PCI = 4
        ''' <summary>
        ''' PCAN-USB and PCAN-USB Pro
        ''' </summary>
        PCAN_USB = 5
        ''' <summary>
        ''' PCAN-PC Card
        ''' </summary>
        PCAN_PCC = 6
    End Enum

    ''' <summary>
    ''' Represents a PCAN parameter to be read or set
    ''' </summary>
    Public Enum TPCANParameter As Byte
        ''' <summary>
        ''' PCAN-USB device number parameter
        ''' </summary>
        PCAN_DEVICE_NUMBER = 1
        ''' <summary>
        ''' PCAN-PC Card 5-Volt power parameter
        ''' </summary>
        PCAN_5VOLTS_POWER = 2
        ''' <summary>
        ''' PCAN receive event handler parameter
        ''' </summary>
        PCAN_RECEIVE_EVENT = 3
        ''' <summary>
        ''' PCAN message filter parameter
        ''' </summary>
        PCAN_MESSAGE_FILTER = 4
        ''' <summary>
        ''' PCAN-Basic API version parameter
        ''' </summary>
        PCAN_API_VERSION = 5
        ''' <summary>
        ''' PCAN device channel version parameter
        ''' </summary>
        PCAN_CHANNEL_VERSION = 6
        ''' <summary>
        ''' PCAN Reset-On-Busoff parameter
        ''' </summary>
        PCAN_BUSOFF_AUTORESET = 7
        ''' <summary>
        ''' PCAN Listen-Only parameter
        ''' </summary>
        PCAN_LISTEN_ONLY = 8
        ''' <summary>
        ''' Directory path for trace files
        ''' </summary>
        PCAN_LOG_LOCATION = 9
        ''' <summary>
        ''' Debug-Trace activation status
        ''' </summary>
        PCAN_LOG_STATUS = 10
        ''' <summary>
        ''' Configuration of the debugged information (LOG_FUNCTION_***)
        ''' </summary>
        PCAN_LOG_CONFIGURE = 11
        ''' <summary>
        ''' Custom insertion of text into the log file
        ''' </summary>
        PCAN_LOG_TEXT = 12
        ''' <summary>
        ''' Availability status of a PCAN-Channel
        ''' </summary>
        PCAN_CHANNEL_CONDITION = 13
        ''' <summary>
        ''' PCAN hardware name parameter
        ''' </summary>
        PCAN_HARDWARE_NAME = 14
        ''' <summary>
        ''' Message reception status of a PCAN-Channel
        ''' </summary>
        PCAN_RECEIVE_STATUS = 15
        ''' <summary>
        ''' CAN-Controller number of a PCAN-Channel
        ''' </summary>
        PCAN_CONTROLLER_NUMBER = 16
    End Enum

    ''' <summary>
    ''' Represents the type of a PCAN message
    ''' </summary>
    <Flags()> _
    Public Enum TPCANMessageType As Byte
        ''' <summary>
        ''' The PCAN message is a CAN Standard Frame (11-bit identifier)
        ''' </summary>
        PCAN_MESSAGE_STANDARD = &H0
        ''' <summary>
        ''' The PCAN message is a CAN Remote-Transfer-Request Frame
        ''' </summary>
        PCAN_MESSAGE_RTR = &H1
        ''' <summary>
        ''' The PCAN message is a CAN Extended Frame (29-bit identifier)
        ''' </summary>
        PCAN_MESSAGE_EXTENDED = &H2
        ''' <summary>
        ''' The PCAN message represents a PCAN status message
        ''' </summary>
        PCAN_MESSAGE_STATUS = &H80
    End Enum

    ''' <summary>
    ''' Represents a PCAN filter mode
    ''' </summary>
    Public Enum TPCANMode As Byte
        ''' <summary>
        ''' Mode is Standard (11-bit identifier)
        ''' </summary>
        PCAN_MODE_STANDARD = TPCANMessageType.PCAN_MESSAGE_STANDARD
        ''' <summary>
        ''' Mode is Extended (29-bit identifier)
        ''' </summary>
        PCAN_MODE_EXTENDED = TPCANMessageType.PCAN_MESSAGE_EXTENDED
    End Enum

    ''' <summary>
    ''' Represents a PCAN Baud rate register value
    ''' </summary>
    Public Enum TPCANBaudrate As UInt16
        ''' <summary>
        ''' 1 MBit/s
        ''' </summary>
        PCAN_BAUD_1M = &H14
        ''' <summary>
        ''' 800 kBit/s
        ''' </summary>
        PCAN_BAUD_800K = &H16
        ''' <summary>
        ''' 500 kBit/s
        ''' </summary>
        PCAN_BAUD_500K = &H1C
        ''' <summary>
        ''' 250 kBit/s
        ''' </summary>
        PCAN_BAUD_250K = &H11C
        ''' <summary>
        ''' 125 kBit/s
        ''' </summary>
        PCAN_BAUD_125K = &H31C
        ''' <summary>
        ''' 100 kBit/s
        ''' </summary>
        PCAN_BAUD_100K = &H432F
        ''' <summary>
        ''' 95,238 kBit/s
        ''' </summary>
        PCAN_BAUD_95K = &HC34E
        ''' <summary>
        ''' 83,333 kBit/s
        ''' </summary>
        PCAN_BAUD_83K = &H4B14
        ''' <summary>
        ''' 50 kBit/s
        ''' </summary>
        PCAN_BAUD_50K = &H472F
        ''' <summary>
        ''' 47,619 kBit/s
        ''' </summary>
        PCAN_BAUD_47K = &H1414
        ''' <summary>
        ''' 33,333 kBit/s
        ''' </summary>
        PCAN_BAUD_33K = &H1D14
        ''' <summary>
        ''' 20 kBit/s
        ''' </summary>
        PCAN_BAUD_20K = &H532F
        ''' <summary>
        ''' 10 kBit/s
        ''' </summary>
        PCAN_BAUD_10K = &H672F
        ''' <summary>
        ''' 5 kBit/s
        ''' </summary>
        PCAN_BAUD_5K = &H7F7F
    End Enum

    ''' <summary>
    ''' Represents the type of PCAN (non plug and play) hardware to be initialized
    ''' </summary>
    Public Enum TPCANType As Byte
        ''' <summary>
        ''' PCAN-ISA 82C200
        ''' </summary>
        PCAN_TYPE_ISA = &H1
        ''' <summary>
        ''' PCAN-ISA SJA1000
        ''' </summary>
        PCAN_TYPE_ISA_SJA = &H9
        ''' <summary>
        ''' PHYTEC ISA 
        ''' </summary>
        PCAN_TYPE_ISA_PHYTEC = &H4
        ''' <summary>
        ''' PCAN-Dongle 82C200
        ''' </summary>
        PCAN_TYPE_DNG = &H2
        ''' <summary>
        ''' PCAN-Dongle EPP 82C200
        ''' </summary>
        PCAN_TYPE_DNG_EPP = &H3
        ''' <summary>
        ''' PCAN-Dongle SJA1000
        ''' </summary>
        PCAN_TYPE_DNG_SJA = &H5
        ''' <summary>
        ''' PCAN-Dongle EPP SJA1000
        ''' </summary>
        PCAN_TYPE_DNG_SJA_EPP = &H6
    End Enum
#End Region

#Region "Structures"
    ''' <summary>
    ''' Represents a PCAN message
    ''' </summary>
    Public Structure TPCANMsg
        ''' <summary>
        ''' 11/29-bit message identifier
        ''' </summary>
        Public ID As UInt32
        ''' <summary>
        ''' Type of the message
        ''' </summary>
        <MarshalAs(UnmanagedType.U1)> _
        Public MSGTYPE As TPCANMessageType
        ''' <summary>
        ''' Data Length Code of the message (0..8)
        ''' </summary>
        Public LEN As Byte
        ''' <summary>
        ''' Data of the message (DATA[0]..DATA[7])
        ''' </summary>
        <MarshalAs(UnmanagedType.ByValArray, SizeConst:=8)> _
        Public DATA As Byte()
    End Structure

    ''' <summary>
    ''' Represents a timestamp of a received PCAN message.
    ''' Total Microseconds = micros + 1000 * millis + 0xFFFFFFFF * 1000 * millis_overflow
    ''' </summary>
    Public Structure TPCANTimestamp
        ''' <summary>
        ''' Base-value: milliseconds: 0.. 2^32-1
        ''' </summary>
        Public millis As UInt32
        ''' <summary>
        ''' Roll-arounds of millis
        ''' </summary>
        Public millis_overflow As UInt16
        ''' <summary>
        ''' Microseconds: 0..999
        ''' </summary>
        Public micros As UInt16
    End Structure
#End Region

#Region "PCANBasic class"
    ''' <summary>
    ''' PCAN-Basic API class implementation
    ''' </summary>
    Public NotInheritable Class PCANBasic
#Region "PCAN-BUS Handles Definition"
        ''' <summary>
        ''' Undefined/default value for a PCAN bus
        ''' </summary>
        Public Const PCAN_NONEBUS As TPCANHandle = &H0

        ''' <summary>
        ''' PCAN-ISA interface, channel 1
        ''' </summary>
        Public Const PCAN_ISABUS1 As TPCANHandle = &H21
        ''' <summary>
        ''' PCAN-ISA interface, channel 2
        ''' </summary>
        Public Const PCAN_ISABUS2 As TPCANHandle = &H22
        ''' <summary>
        ''' PCAN-ISA interface, channel 3
        ''' </summary>
        Public Const PCAN_ISABUS3 As TPCANHandle = &H23
        ''' <summary>
        ''' PCAN-ISA interface, channel 4
        ''' </summary>
        Public Const PCAN_ISABUS4 As TPCANHandle = &H24
        ''' <summary>
        ''' PCAN-ISA interface, channel 5
        ''' </summary>
        Public Const PCAN_ISABUS5 As TPCANHandle = &H25
        ''' <summary>
        ''' PCAN-ISA interface, channel 6
        ''' </summary>
        Public Const PCAN_ISABUS6 As TPCANHandle = &H26
        ''' <summary>
        ''' PCAN-ISA interface, channel 7
        ''' </summary>
        Public Const PCAN_ISABUS7 As TPCANHandle = &H27
        ''' <summary>
        ''' PCAN-ISA interface, channel 8
        ''' </summary>
        Public Const PCAN_ISABUS8 As TPCANHandle = &H28

        ''' <summary>
        ''' PPCAN-Dongle/LPT interface, channel 1 
        ''' </summary>
        Public Const PCAN_DNGBUS1 As TPCANHandle = &H31

        ''' <summary>
        ''' PCAN-PCI interface, channel 1
        ''' </summary>
        Public Const PCAN_PCIBUS1 As TPCANHandle = &H41
        ''' <summary>
        ''' PCAN-PCI interface, channel 2
        ''' </summary>
        Public Const PCAN_PCIBUS2 As TPCANHandle = &H42
        ''' <summary>
        ''' PCAN-PCI interface, channel 3
        ''' </summary>
        Public Const PCAN_PCIBUS3 As TPCANHandle = &H43
        ''' <summary>
        ''' PCAN-PCI interface, channel 4
        ''' </summary>
        Public Const PCAN_PCIBUS4 As TPCANHandle = &H44
        ''' <summary>
        ''' PCAN-PCI interface, channel 5
        ''' </summary>
        Public Const PCAN_PCIBUS5 As TPCANHandle = &H45
        ''' <summary>
        ''' PCAN-PCI interface, channel 6
        ''' </summary>
        Public Const PCAN_PCIBUS6 As TPCANHandle = &H46
        ''' <summary>
        ''' PCAN-PCI interface, channel 7
        ''' </summary>
        Public Const PCAN_PCIBUS7 As TPCANHandle = &H47
        ''' <summary>
        ''' PCAN-PCI interface, channel 8
        ''' </summary>
        Public Const PCAN_PCIBUS8 As TPCANHandle = &H48

        ''' <summary>
        ''' PCAN-USB interface, channel 1
        ''' </summary>
        Public Const PCAN_USBBUS1 As TPCANHandle = &H51
        ''' <summary>
        ''' PCAN-USB interface, channel 2
        ''' </summary>
        Public Const PCAN_USBBUS2 As TPCANHandle = &H52
        ''' <summary>
        ''' PCAN-USB interface, channel 3
        ''' </summary>
        Public Const PCAN_USBBUS3 As TPCANHandle = &H53
        ''' <summary>
        ''' PCAN-USB interface, channel 4
        ''' </summary>
        Public Const PCAN_USBBUS4 As TPCANHandle = &H54
        ''' <summary>
        ''' PCAN-USB interface, channel 5
        ''' </summary>
        Public Const PCAN_USBBUS5 As TPCANHandle = &H55
        ''' <summary>
        ''' PCAN-USB interface, channel 6
        ''' </summary>
        Public Const PCAN_USBBUS6 As TPCANHandle = &H56
        ''' <summary>
        ''' PCAN-USB interface, channel 7
        ''' </summary>
        Public Const PCAN_USBBUS7 As TPCANHandle = &H57
        ''' <summary>
        ''' PCAN-USB interface, channel 8
        ''' </summary>
        Public Const PCAN_USBBUS8 As TPCANHandle = &H58

        ''' <summary>
        ''' PCAN-PC Card interface, channel 1
        ''' </summary>
        Public Const PCAN_PCCBUS1 As TPCANHandle = &H61
        ''' <summary>
        ''' PCAN-PC Card interface, channel 2
        ''' </summary>
        Public Const PCAN_PCCBUS2 As TPCANHandle = &H62
#End Region

#Region "Parameter values definition"
        ''' <summary>
        ''' The PCAN parameter is not set (inactive)
        ''' </summary>
        Public Const PCAN_PARAMETER_OFF As Integer = 0
        ''' <summary>
        ''' The PCAN parameter is set (active)
        ''' </summary>
        Public Const PCAN_PARAMETER_ON As Integer = 1
        ''' <summary>
        ''' The PCAN filter is closed. No messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_CLOSE As Integer = 0
        ''' <summary>
        ''' The PCAN filter is fully opened. All messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_OPEN As Integer = 1
        ''' <summary>
        ''' The PCAN filter is custom configured. Only registered 
        ''' messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_CUSTOM As Integer = 2
        ''' <summary>
        ''' The PCAN-Channel handle is illegal, or its associated hadware is not available
        ''' </summary>
        Public Const PCAN_CHANNEL_UNAVAILABLE As Integer = 0
        ''' <summary>
        ''' The PCAN-Channel handle is available to be connected (Plug and Play Hardware: it means futhermore that the hardware is plugged-in)
        ''' </summary>
        Public Const PCAN_CHANNEL_AVAILABLE As Integer = 1
        ''' <summary>
        ''' The PCAN-Channel handle is valid, and is already being used
        ''' </summary>
        Public Const PCAN_CHANNEL_OCCUPIED As Integer = 2

        ''' <summary>
        ''' Logs system exceptions / errors
        ''' </summary>
        Public Const LOG_FUNCTION_DEFAULT As Integer = &H0
        ''' <summary>
        ''' Logs the entries to the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_ENTRY As Integer = &H1
        ''' <summary>
        ''' Logs the parameters passed to the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_PARAMETERS As Integer = &H2
        ''' <summary>
        ''' Logs the exits from the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_LEAVE As Integer = &H4
        ''' <summary>
        ''' Logs the CAN messages passed to the CAN_Write function
        ''' </summary>
        Public Const LOG_FUNCTION_WRITE As Integer = &H8
        ''' <summary>
        ''' Logs the CAN messages received within the CAN_Read function
        ''' </summary>
        Public Const LOG_FUNCTION_READ As Integer = &H10
        ''' <summary>
        ''' Logs all possible information within the PCAN-Basic API functions
        ''' </summary>
        Public Const LOG_FUNCTION_ALL As Integer = &HFFFF
#End Region

#Region "PCANBasic API Implementation"
        ''' <summary>
        ''' Initializes a PCAN Channel 
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        ''' <param name="HwType">NON PLUG AND PLAY: The type of hardware and operation mode</param>
        ''' <param name="IOPort">NON PLUG AND PLAY: The I/O address for the parallel port</param>
        ''' <param name="Interrupt">NON PLUG AND PLAY: Interrupt number of the parallel por</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Initialize")> _
        Public Shared Function Initialize( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Btr0Btr1 As TPCANBaudrate, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal HwType As TPCANType, _
            ByVal IOPort As UInt32, _
            ByVal Interrupt As UInt16) As TPCANStatus
        End Function

        ''' <summary>
        ''' Initializes a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function Initialize( _
            ByVal Channel As TPCANHandle, _
            ByVal Btr0Btr1 As TPCANBaudrate) As TPCANStatus
            Return Initialize(Channel, Btr0Btr1, 0, 0, 0)
        End Function

        ''' <summary>
        ''' Uninitializes one or all PCAN Channels initialized by CAN_Initialize
        ''' </summary>
        ''' <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
        ''' uninitialize all initialized channels</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Uninitialize")> _
        Public Shared Function Uninitialize( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Resets the receive and transmit queues of the PCAN Channel
        ''' </summary>
        ''' <remarks>A reset of the CAN controller is not performed</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Reset")> _
        Public Shared Function Reset( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Gets the current status of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetStatus")> _
        Public Shared Function GetStatus( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>
        ''' <param name="TimestampBuffer">A TPCANTimestamp structure buffer to get
        ''' the reception time of the message</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Read")> _
        Public Shared Function Read( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg, _
            ByRef TimestampBuffer As TPCANTimestamp) As TPCANStatus
        End Function

        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Read")> _
        Private Shared Function Read( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg, _
            ByVal BufferPointer As IntPtr) As TPCANStatus
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>        
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function Read( _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg) As TPCANStatus
            Return Read(Channel, MessageBuffer, IntPtr.Zero)
        End Function

        ''' <summary>
        '''  Transmits a CAN message 
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg buffer with the message to be sent</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Write")> _
        Public Shared Function Write( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures the reception filter
        ''' </summary>
        ''' <remarks>The message filter will be expanded with every call to 
        ''' this function. If it is desired to reset the filter, please use
        ''' the 'SetValue' function</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="FromID">The lowest CAN ID to be received</param>
        ''' <param name="ToID">The highest CAN ID to be received</param>
        ''' <param name="Mode">Message type, Standard (11-bit identifier) or
        ''' Extended (29-bit identifier)</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_FilterMessages")> _
        Public Shared Function FilterMessages( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            ByVal FromID As UInt32, _
            ByVal ToID As UInt32, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Mode As TPCANMode) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="StringBuffer">Buffer for the parameter value</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")> _
        Public Shared Function GetValue( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            ByVal StringBuffer As StringBuilder, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="NumericBuffer">Buffer for the parameter value</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")> _
        Public Shared Function GetValue( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            ByRef NumericBuffer As UInt32, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures or sets a PCAN Channel value 
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to set</param>
        ''' <param name="NumericBuffer">Buffer with the value to be set</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_SetValue")> _
        Public Shared Function SetValue( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            ByRef NumericBuffer As UInt32, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures or sets a PCAN Channel value 
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter"></param>
        ''' <param name="StringBuffer">Buffer with the value to be set</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_SetValue")> _
        Public Shared Function SetValue( _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            <MarshalAs(UnmanagedType.LPStr, SizeParamIndex:=3)> _
            ByVal StringBuffer As String, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Returns a descriptive text of a given TPCANStatus error 
        ''' code, in any desired language
        ''' </summary>
        ''' <remarks>The current languages available for translation are: 
        ''' Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
        ''' Italian (0x10) and French (0x0C)</remarks>
        ''' <param name="anError">A TPCANStatus error code</param>
        ''' <param name="Language">Indicates a 'Primary language ID'</param>
        ''' <param name="StringBuffer">Buffer for the text (must be at least 256 in length)</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetErrorText")> _
        Public Shared Function GetErrorText( _
            <MarshalAs(UnmanagedType.U4)> _
            ByVal anError As TPCANStatus, _
            ByVal Language As UInt16, _
            ByVal StringBuffer As StringBuilder) As TPCANStatus
        End Function
#End Region
    End Class
#End Region
End Namespace


