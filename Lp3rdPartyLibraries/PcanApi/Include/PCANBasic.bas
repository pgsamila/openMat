Attribute VB_Name = "PCANBasic"
'  PCANBasic.bas
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
'  Language: Visual Basic 6.0
'  ------------------------------------------------------------------
'
'  Copyright (C) 1999-2012  PEAK-System Technik GmbH, Darmstadt
'  more Info at http://www.peak-system.com 
'

'///////////////////////////////////////////////////////////
'/ Value definitions
'///////////////////////////////////////////////////////////

' Currently defined and supported PCAN channels
'
Public Const PCAN_NONEBUS As Byte = &H0   ' Undefined/default value for a PCAN bus

Public Const PCAN_ISABUS1 As Byte = &H21  ' PCAN-ISA interface, channel 1
Public Const PCAN_ISABUS2 As Byte = &H22  ' PCAN-ISA interface, channel 2
Public Const PCAN_ISABUS3 As Byte = &H23  ' PCAN-ISA interface, channel 3
Public Const PCAN_ISABUS4 As Byte = &H24  ' PCAN-ISA interface, channel 4
Public Const PCAN_ISABUS5 As Byte = &H25  ' PCAN-ISA interface, channel 5
Public Const PCAN_ISABUS6 As Byte = &H26  ' PCAN-ISA interface, channel 6
Public Const PCAN_ISABUS7 As Byte = &H27  ' PCAN-ISA interface, channel 7
Public Const PCAN_ISABUS8 As Byte = &H28  ' PCAN-ISA interface, channel 8

Public Const PCAN_DNGBUS1 As Byte = &H31  ' PCAN-Dongle/LPT interface, channel 1

Public Const PCAN_PCIBUS1 As Byte = &H41  ' PCAN-PCI interface, channel 1
Public Const PCAN_PCIBUS2 As Byte = &H42  ' PCAN-PCI interface, channel 2
Public Const PCAN_PCIBUS3 As Byte = &H43  ' PCAN-PCI interface, channel 3
Public Const PCAN_PCIBUS4 As Byte = &H44  ' PCAN-PCI interface, channel 4
Public Const PCAN_PCIBUS5 As Byte = &H45  ' PCAN-PCI interface, channel 5
Public Const PCAN_PCIBUS6 As Byte = &H46  ' PCAN-PCI interface, channel 6
Public Const PCAN_PCIBUS7 As Byte = &H47  ' PCAN-PCI interface, channel 7
Public Const PCAN_PCIBUS8 As Byte = &H48  ' PCAN-PCI interface, channel 8

Public Const PCAN_USBBUS1 As Byte = &H51  ' PCAN-USB interface, channel 1
Public Const PCAN_USBBUS2 As Byte = &H52  ' PCAN-USB interface, channel 2
Public Const PCAN_USBBUS3 As Byte = &H53  ' PCAN-USB interface, channel 3
Public Const PCAN_USBBUS4 As Byte = &H54  ' PCAN-USB interface, channel 4
Public Const PCAN_USBBUS5 As Byte = &H55  ' PCAN-USB interface, channel 5
Public Const PCAN_USBBUS6 As Byte = &H56  ' PCAN-USB interface, channel 6
Public Const PCAN_USBBUS7 As Byte = &H57  ' PCAN-USB interface, channel 7
Public Const PCAN_USBBUS8 As Byte = &H58  ' PCAN-USB interface, channel 8

Public Const PCAN_PCCBUS1 As Byte = &H61  ' PCAN-PC Card interface, channel 1
Public Const PCAN_PCCBUS2 As Byte = &H62  ' PCAN-PC Card interface, channel 2

' Represent the PCAN error and status codes
'
Public Const PCAN_ERROR_OK As Long = &H0              ' No error
Public Const PCAN_ERROR_XMTFULL As Long = &H1         ' Transmit buffer in CAN controller is full
Public Const PCAN_ERROR_OVERRUN As Long = &H2         ' CAN controller was read too late
Public Const PCAN_ERROR_BUSLIGHT As Long = &H4        ' Bus error: an error counter reached the 'light' limit
Public Const PCAN_ERROR_BUSHEAVY As Long = &H8        ' Bus error: an error counter reached the 'heavy' limit
Public Const PCAN_ERROR_BUSOFF As Long = &H10         ' Bus error: the CAN controller is in bus-off state
Public Const PCAN_ERROR_ANYBUSERR = (PCAN_ERROR_BUSLIGHT Or PCAN_ERROR_BUSHEAVY Or PCAN_ERROR_BUSOFF)    ' Mask for all bus errors
Public Const PCAN_ERROR_QRCVEMPTY As Long = &H20      ' Receive queue is empty
Public Const PCAN_ERROR_QOVERRUN As Long = &H40       ' Receive queue was read too late
Public Const PCAN_ERROR_QXMTFULL As Long = &H80       ' Transmit queue is full
Public Const PCAN_ERROR_REGTEST As Long = &H100       ' Test of the CAN controller hardware registers failed (no hardware found)
Public Const PCAN_ERROR_NODRIVER As Long = &H200      ' Driver not loaded
Public Const PCAN_ERROR_HWINUSE As Long = &H400       ' Hardware already in use by a Net
Public Const PCAN_ERROR_NETINUSE As Long = &H800      ' A Client is already connected to the Net
Public Const PCAN_ERROR_ILLHW As Long = &H1400        ' Hardware handle is invalid
Public Const PCAN_ERROR_ILLNET As Long = &H1800       ' Net handle is invalid
Public Const PCAN_ERROR_ILLCLIENT As Long = &H1C00    ' Client handle is invalid
Public Const PCAN_ERROR_ILLHANDLE = (PCAN_ERROR_ILLHW Or PCAN_ERROR_ILLNET Or PCAN_ERROR_ILLCLIENT)     ' Mask for all handle errors
Public Const PCAN_ERROR_RESOURCE As Long = &H2000     ' Resource (FIFO, Client, timeout) cannot be created
Public Const PCAN_ERROR_ILLPARAMTYPE As Long = &H4000 ' Invalid parameter
Public Const PCAN_ERROR_ILLPARAMVAL As Long = &H8000  ' Invalid parameter value
Public Const PCAN_ERROR_UNKNOWN As Long = &H10000     ' Unknow error
Public Const PCAN_ERROR_ILLDATA As Long = &H20000     ' Invalid data, function, or action
Public Const PCAN_ERROR_INITIALIZE As Long = &H40000  ' Channel is not initialized

' PCAN devices
'
Public Const PCAN_NONE = &H0        ' Undefined, unknown or not selected PCAN device value
Public Const PCAN_PEAKCAN = &H1     ' PCAN Non-Plug&Play devices. NOT USED WITHIN PCAN-Basic API
Public Const PCAN_ISA = &H2         ' PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus
Public Const PCAN_DNG = &H3         ' PCAN-Dongle
Public Const PCAN_PCI = &H4         ' PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express
Public Const PCAN_USB = &H5         ' PCAN-USB and PCAN-USB Pro
Public Const PCAN_PCC = &H6         ' PCAN-PC Card

' PCAN parameters
'
Public Const PCAN_DEVICE_NUMBER = &H1         ' PCAN-USB device number parameter
Public Const PCAN_5VOLTS_POWER = &H2          ' PCAN-PC Card 5-Volt power parameter
Public Const PCAN_RECEIVE_EVENT = &H3         ' PCAN receive event handler parameter
Public Const PCAN_MESSAGE_FILTER = &H4        ' PCAN message filter parameter
Public Const PCAN_API_VERSION = &H5           ' PCAN-Basic API version parameter
Public Const PCAN_CHANNEL_VERSION = &H6       ' PCAN device channel version parameter
Public Const PCAN_BUSOFF_AUTORESET = &H7      ' PCAN Reset-On-Busoff parameter
Public Const PCAN_LISTEN_ONLY = &H8           ' PCAN Listen-Only parameter
Public Const PCAN_LOG_LOCATION = &H9          ' Directory path for trace files
Public Const PCAN_LOG_STATUS = &HA            ' Debug-Trace activation status
Public Const PCAN_LOG_CONFIGURE = &HB         ' Configuration of the debugged information (LOG_FUNCTION_***)
Public Const PCAN_LOG_TEXT = &HC              ' Custom insertion of text into the log file
Public Const PCAN_CHANNEL_CONDITION = &HD     ' Availability status of a PCAN-Channel
Public Const PCAN_HARDWARE_NAME = &HE         ' PCAN hardware name parameter
Public Const PCAN_RECEIVE_STATUS = &HF        ' Message reception status of a PCAN-Channel
Public Const PCAN_CONTROLLER_NUMBER = &H10    ' CAN-Controller number of a PCAN-Channel

' PCAN parameter values
'
Public Const PCAN_PARAMETER_OFF = &H0         ' The PCAN parameter is not set (inactive)
Public Const PCAN_PARAMETER_ON = &H1          ' The PCAN parameter is set (active)
Public Const PCAN_FILTER_CLOSE = &H0          ' The PCAN filter is closed. No messages will be received
Public Const PCAN_FILTER_OPEN = &H1           ' The PCAN filter is fully opened. All messages will be received
Public Const PCAN_FILTER_CUSTOM = &H2         ' The PCAN filter is custom configured. Only registered messages will be received
Public Const PCAN_CHANNEL_UNAVAILABLE = &H0   ' The PCAN-Channel handle is illegal, or its associated hadware is not available
Public Const PCAN_CHANNEL_AVAILABLE = &H1     ' The PCAN-Channel handle is available to be connected (Plug&Play Hardware: it means futhermore that the hardware is plugged-in)
Public Const PCAN_CHANNEL_OCCUPIED = &H2      ' The PCAN-Channel handle is valid, and is already being used

Public Const LOG_FUNCTION_DEFAULT = &H0       ' Logs system exceptions / errors
Public Const LOG_FUNCTION_ENTRY = &H1         ' Logs the entries to the PCAN-Basic API functions
Public Const LOG_FUNCTION_PARAMETERS = &H2    ' Logs the parameters passed to the PCAN-Basic API functions
Public Const LOG_FUNCTION_LEAVE = &H4         ' Logs the exits from the PCAN-Basic API functions
Public Const LOG_FUNCTION_WRITE = &H8         ' Logs the CAN messages passed to the CAN_Write function
Public Const LOG_FUNCTION_READ = &H10         ' Logs the CAN messages received within the CAN_Read function
Public Const LOG_FUNCTION_ALL = &HFFFF        ' Logs all possible information within the PCAN-Basic API functions

' PCAN message types
'
Public Const PCAN_MESSAGE_STANDARD = &H0      ' The PCAN message is a CAN Standard Frame (11-bit identifier)
Public Const PCAN_MESSAGE_RTR = &H1           ' The PCAN message is a CAN Remote-Transfer-Request Frame
Public Const PCAN_MESSAGE_EXTENDED = &H2      ' The PCAN message is a CAN Extended Frame (29-bit identifier)
Public Const PCAN_MESSAGE_STATUS = &H80       ' The PCAN message represents a PCAN status message

' Frame Type / Initialization Mode
'
Public Const PCAN_MODE_STANDARD = PCAN_MESSAGE_STANDARD
Public Const PCAN_MODE_EXTENDED = PCAN_MESSAGE_EXTENDED

' Baud rate codes = BTR0/BTR1 register values for the CAN controller.
' You can define your own Baud rate with the BTROBTR1 register.
' Take a look at www.peak-system.com for our free software "BAUDTOOL"
' to calculate the BTROBTR1 register for every baudrate and sample point.
'
Public Const PCAN_BAUD_1M = &H14                '   1 MBit/s
Public Const PCAN_BAUD_800K = &H16              ' 800 kBit/s
Public Const PCAN_BAUD_500K = &H1C              ' 500 kBit/s
Public Const PCAN_BAUD_250K = &H11C             ' 250 kBit/s
Public Const PCAN_BAUD_125K = &H31C             ' 125 kBit/s
Public Const PCAN_BAUD_100K = &H432F            ' 100 kBit/s
Public Const PCAN_BAUD_95K = &HC34E             '  95,238 kBit/s
Public Const PCAN_BAUD_83K = &H4B14             '  83,333 kBit/s
Public Const PCAN_BAUD_50K = &H472F             '  50 kBit/s
Public Const PCAN_BAUD_47K = &H1414             '  47,619 kBit/s
Public Const PCAN_BAUD_33K = &H1D14             '  33,333 kBit/s
Public Const PCAN_BAUD_20K = &H532F             '  20 kBit/s
Public Const PCAN_BAUD_10K = &H672F             '  10 kBit/s
Public Const PCAN_BAUD_5K = &H7F7F              '   5 kBit/s

Public Const PCAN_TYPE_ISA = &H1              ' PCAN-ISA 82C200
Public Const PCAN_TYPE_ISA_SJA = &H9          ' PCAN-ISA SJA1000
Public Const PCAN_TYPE_ISA_PHYTEC = &H4       ' PHYTEC ISA
Public Const PCAN_TYPE_DNG = &H2              ' PCAN-Dongle 82C200
Public Const PCAN_TYPE_DNG_EPP = &H3          ' PCAN-Dongle EPP 82C200
Public Const PCAN_TYPE_DNG_SJA = &H5          ' PCAN-Dongle SJA1000
Public Const PCAN_TYPE_DNG_SJA_EPP = &H6      ' PCAN-Dongle EPP SJA1000

' CAN message
Public Type TPCANMsg
    ID As Long                    ' 11/29-bit message identifier
    MsgType As Byte               ' Type of the message
    LEN As Byte                   ' Data Length Code of the message (0..8)
    DATA(7) As Byte               ' Data of the message (DATA[0]..DATA[7])
End Type

Public Type TPCANTimestamp
    millis As Long                ' base-value: milliseconds: 0.. 2^32-1
    millis_overflow As Integer    ' roll-arounds of millis
    micros As Integer             ' microseconds: 0..999
End Type


''' <summary>
''' Initializes a PCAN Channel
''' </summary>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="Btr0Btr1">"The speed for the communication (BTR0BTR1 code)"</param>
''' <param name="HwType">"NON PLUG&PLAY: The type of hardware and operation mode"</param>
''' <param name="IOPort">"NON PLUG&PLAY: The I/O address for the parallel port"</param>
''' <param name="Interupt">"NON PLUG&PLAY: Interrupt number of the parallel port"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_Initialize Lib "PCANBasic.DLL" _
    (ByVal channel As Byte, _
    ByVal Btr0Btr1 As Integer, _
    Optional ByVal HwType As Byte = 0, _
    Optional ByVal IOPort As Long = 0, _
    Optional ByVal Interrupt As Integer = 0) As Long


''' <summary>
''' Uninitializes one or all PCAN Channels initialized by CAN_Initialize
''' </summary>
''' <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
''' uninitialize all initialized channels</remarks>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_Uninitialize Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte) As Long


''' <summary>
''' Resets the receive and transmit queues of the PCAN Channel  
''' </summary>
''' <remarks>
''' A reset of the CAN controller is not performed.
''' </remarks>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_Reset Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte) As Long


''' <summary>
''' Gets the current status of a PCAN Channel 
''' </summary>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_GetStatus Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte) As Long


''' <summary>
''' Reads a CAN message from the receive queue of a PCAN Channel 
''' </summary>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="MessageBuffer">"A TPCANMsg structure buffer to store the CAN message"</param>
''' <param name="TimestampBuffer">"A TPCANTimestamp structure buffer to get 
''' the reception time of the message. If this value is not desired, this parameter
''' should be passed as NULL"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_Read Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte, _
    ByRef MessageBuffer As TPCANMsg, _
    ByRef TimestampBuffer As TPCANTimestamp) As Long


''' <summary>
''' Transmits a CAN message 
''' </summary>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="MessageBuffer">"A TPCANMsg buffer with the message to be sent"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_Write Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte, _
    ByRef MessageBuffer As TPCANMsg) As Long


''' <summary>
''' Configures the reception filter. 
''' </summary>
''' <remarks>The message filter will be expanded with every call to 
''' this function. If it is desired to reset the filter, please use 
''' the CAN_SetValue function</remarks>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="FromID">"The lowest CAN ID to be received"</param>
''' <param name="ToID">"The highest CAN ID to be received"</param>
''' <param name="Mode">"Message type, Standard (11-bit identifier) or 
''' Extended (29-bit identifier)"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_FilterMessages Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte, _
    ByVal FromID As Long, _
    ByVal ToID As Long, _
    ByVal Mode As Byte) As Long


''' <summary>
''' Retrieves a PCAN Channel value
''' </summary>
''' <remarks>Parameters can be present or not according with the kind 
''' of Hardware (PCAN Channel) being used. If a parameter is not available,
''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="Parameter">"The TPCANParameter parameter to get"</param>
''' <param name="Buffer">"Buffer for the parameter value"</param>
''' <param name="BufferLength">"Size in bytes of the buffer"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_GetValue Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte, _
    ByVal Parameter As Byte, _
    ByRef Buffer As Any, _
    ByVal BufferLength As Long) As Long


''' <summary>
''' Configures or sets a PCAN Channel value 
''' </summary>
''' <remarks>Parameters can be present or not according with the kind 
''' of Hardware (PCAN Channel) being used. If a parameter is not available,
''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
''' <param name="Channel">"The handle of a PCAN Channel"</param>
''' <param name="Parameter">"The TPCANParameter parameter to set"</param>
''' <param name="Buffer">"Buffer with the value to be set"</param>
''' <param name="BufferLength">"Size in bytes of the buffer"</param>
''' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_SetValue Lib "PCANBasic.DLL" _
    (ByVal Channel As Byte, _
    ByVal Parameter As Byte, _
    ByRef Buffer As Any, _
    ByVal BufferLength As Long) As Long


' <summary>
' Returns a descriptive text of a given TPCANStatus error 
' code, in any desired language
' </summary>
' <remarks>The current languages available for translation are: 
' Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
' Italian (0x10) and French (0x0C)</remarks>
' <param name="Error">"A TPCANStatus error code"</param>
' <param name="Language">"Indicates a 'Primary language ID'"</param>
' <param name="Buffer">"Buffer for a null terminated char array"</param>
' <returns>"A TPCANStatus error code"</returns>
Public Declare Function CAN_GetErrorText Lib "PCANBasic.DLL" _
    (ByVal ErrorCode As Long, _
    ByVal Language As Integer, _
    ByVal Buffer As String) As Long