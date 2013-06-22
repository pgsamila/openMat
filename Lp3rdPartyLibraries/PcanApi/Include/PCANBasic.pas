//  PCANBasic.pas
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
//  Language: Pascal OO
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2012  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com
//
unit PCANBasic;

interface

type
    TPCANHandle = Byte;

{$Z4}
    /// <summary>
    /// Represents a PCAN status/error code
    /// </summary>
    TPCANStatus = (
        /// <summary>
        /// No error
        /// </summary>
        PCAN_ERROR_OK           = $00000,
        /// <summary>
        /// Transmit buffer in CAN controller is full
        /// </summary>
        PCAN_ERROR_XMTFULL      = $00001,
        /// <summary>
        /// CAN controller was read too late
        /// </summary>
        PCAN_ERROR_OVERRUN      = $00002,
        /// <summary>
        /// Bus error: an error counter reached the 'light' limit
        /// </summary>
        PCAN_ERROR_BUSLIGHT     = $00004,
        /// <summary>
        /// Bus error: an error counter reached the 'heavy' limit
        /// </summary>
        PCAN_ERROR_BUSHEAVY     = $00008,
        /// <summary>
        /// Bus error: the CAN controller is in bus-off state
        /// </summary>
        PCAN_ERROR_BUSOFF       = $00010,
        /// <summary>
        /// Mask for all bus errors
        /// </summary>
        PCAN_ERROR_ANYBUSERR    = Byte(PCAN_ERROR_BUSLIGHT) Or Byte(PCAN_ERROR_BUSHEAVY) Or Byte(PCAN_ERROR_BUSOFF),
        /// <summary>
        /// Receive queue is empty
        /// </summary>
        PCAN_ERROR_QRCVEMPTY    = $00020,
        /// <summary>
        /// Receive queue was read too late
        /// </summary>
        PCAN_ERROR_QOVERRUN     = $00040,
        /// <summary>
        /// Transmit queue is full
        /// </summary>
        PCAN_ERROR_QXMTFULL     = $00080,
        /// <summary>
        /// Test of the CAN controller hardware registers failed (no hardware found)
        /// </summary>
        PCAN_ERROR_REGTEST      = $00100,
        /// <summary>
        /// Driver not loaded
        /// </summary>
        PCAN_ERROR_NODRIVER     = $00200,
        /// <summary>
        /// Hardware already in use by a Net
        /// </summary>
        PCAN_ERROR_HWINUSE      = $00400,
        /// <summary>
        /// A Client is already connected to the Net
        /// </summary>
        PCAN_ERROR_NETINUSE     = $00800,
        /// <summary>
        /// Hardware handle is invalid
        /// </summary>
        PCAN_ERROR_ILLHW        = $01400,
        /// <summary>
        /// Net handle is invalid
        /// </summary>
        PCAN_ERROR_ILLNET       = $01800,
        /// <summary>
        /// Client handle is invalid
        /// </summary>
        PCAN_ERROR_ILLCLIENT    = $01C00,
        /// <summary>
        /// Mask for all handle errors
        /// </summary>
        PCAN_ERROR_ILLHANDLE    = Byte(PCAN_ERROR_ILLHW) Or Byte(PCAN_ERROR_ILLNET) Or Byte(PCAN_ERROR_ILLCLIENT),        
        /// <summary>
        /// Resource (FIFO, Client, timeout) cannot be created
        /// </summary>
        PCAN_ERROR_RESOURCE     = $02000,
        /// <summary>
        /// Invalid parameter
        /// </summary>
        PCAN_ERROR_ILLPARAMTYPE = $04000,
        /// <summary>
        /// Invalid parameter value
        /// </summary>
        PCAN_ERROR_ILLPARAMVAL  = $08000,
        /// <summary>
        /// Unknow error
        /// </summary>
        PCAN_ERROR_UNKNOWN      = $10000,
		/// <summary>
        /// Invalid data, function, or action
        /// </summary>
        PCAN_ERROR_ILLDATA      = $20000,
        /// <summary>
        /// Channel is not initialized
        /// </summary>
        PCAN_ERROR_INITIALIZE   = $40000
    );

{$Z1}
    /// <summary>
    /// Represents a PCAN device
    /// </summary>
    TPCANDevice = (
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
    );

    /// <summary>
    /// Represents a PCAN parameter to be read or set
    /// </summary>
    TPCANParameter = (
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
        PCAN_CONTROLLER_NUMBER   = 16
    );

    /// <summary>
    /// Represents the type of a PCAN message
    /// </summary>
    TPCANMessageType = (
        /// <summary>
        /// The PCAN message is a CAN Standard Frame (11-bit identifier)
        /// </summary>
        PCAN_MESSAGE_STANDARD  = $00,
        /// <summary>
        /// The PCAN message is a CAN Remote-Transfer-Request Frame
        /// </summary>
        PCAN_MESSAGE_RTR       = $01,
        /// <summary>
        /// The PCAN message is a CAN Extended Frame (29-bit identifier)
        /// </summary>
        PCAN_MESSAGE_EXTENDED  = $02,
        /// <summary>
        /// The PCAN message represents a PCAN status message
        /// </summary>
        PCAN_MESSAGE_STATUS    = $80
    );

    /// <summary>
    /// Represents a PCAN filter mode
    /// </summary>
    TPCANMode = (
        /// <summary>
        /// Mode is Standard (11-bit identifier)
        /// </summary>
        PCAN_MODE_STANDARD = Byte(PCAN_MESSAGE_STANDARD),
        /// <summary>
        /// Mode is Extended (29-bit identifier)
        /// </summary>
        PCAN_MODE_EXTENDED = Byte(PCAN_MESSAGE_EXTENDED)
    );

{$Z2}
    /// <summary>
    /// Represents a PCAN Baud rate register value
    /// </summary>
    TPCANBaudrate = (
        /// <summary>
        /// 1 MBit/s
        /// </summary>
        PCAN_BAUD_1M      = $0014,
        /// <summary>
        /// 800 kBit/s
        /// </summary>
        PCAN_BAUD_800K      = $0016,
        /// <summary>
        /// 500 kBit/s
        /// </summary>
        PCAN_BAUD_500K    = $001C,
        /// <summary>
        /// 250 kBit/s
        /// </summary>
        PCAN_BAUD_250K    = $011C,
        /// <summary>
        /// 125 kBit/s
        /// </summary>
        PCAN_BAUD_125K    = $031C,
        /// <summary>
        /// 100 kBit/s
        /// </summary>
        PCAN_BAUD_100K    = $432F,
        /// <summary>
        /// 95,238 kBit/s
        /// </summary>
        PCAN_BAUD_95K      = $C34E,
        /// <summary>
        /// 83,333 kBit/s
        /// </summary>
        PCAN_BAUD_83K      = $4B14,
        /// <summary>
        /// 50 kBit/s
        /// </summary>
        PCAN_BAUD_50K     = $472F,
        /// <summary>
        /// 47,619 kBit/s
        /// </summary>
        PCAN_BAUD_47K      = $1414,
        /// <summary>
        /// 33,333 kBit/s
        /// </summary>
        PCAN_BAUD_33K      = $1D14,
        /// <summary>
        /// 20 kBit/s
        /// </summary>
        PCAN_BAUD_20K     = $532F,
        /// <summary>
        /// 10 kBit/s
        /// </summary>
        PCAN_BAUD_10K     = $672F,
        /// <summary>
        /// 5 kBit/s
        /// </summary>
        PCAN_BAUD_5K      = $7F7F
    );

{$Z1}
    /// <summary>
    /// Represents the type of PCAN (non plug&play) hardware to be initialized
    /// </summary>
    TPCANType = (
        /// <summary>
        /// PCAN-ISA 82C200
        /// </summary>
        PCAN_TYPE_ISA           = $01,
        /// <summary>
        /// PCAN-ISA SJA1000
        /// </summary>
        PCAN_TYPE_ISA_SJA       = $09,
        /// <summary>
        /// PHYTEC ISA
        /// </summary>
        PCAN_TYPE_ISA_PHYTEC    = $04,
        /// <summary>
        /// PCAN-Dongle 82C200
        /// </summary>
        PCAN_TYPE_DNG           = $02,
        /// <summary>
        /// PCAN-Dongle EPP 82C200
        /// </summary>
        PCAN_TYPE_DNG_EPP       = $03,
        /// <summary>
        /// PCAN-Dongle SJA1000
        /// </summary>
        PCAN_TYPE_DNG_SJA       = $05,
        /// <summary>
        /// PCAN-Dongle EPP SJA1000
        /// </summary>
        PCAN_TYPE_DNG_SJA_EPP   = $06
    );

    /// <summary>
    /// Represents a PCAN message
    /// </summary>
    TPCANMsg = record
        /// <summary>
        /// 11/29-bit message identifier
        /// </summary>
        ID: Longword;
        /// <summary>
        /// Type of the message
        /// </summary>
        MSGTYPE: TPCANMessageType;
        /// <summary>
        /// Data Length Code of the message (0..8)
        /// </summary>
        LEN: Byte;
        /// <summary>
        /// Data of the message (DATA[0]..DATA[7])
        /// </summary>
        DATA: array[0..7] of Byte;
    end;

    /// <summary>
    /// Represents a timestamp of a received PCAN message.
    /// Total Microseconds = micros + 1000 * millis + 0xFFFFFFFF * 1000 * millis_overflow
    /// </summary>
    TPCANTimestamp = record
        /// <summary>
        /// Base-value: milliseconds: 0.. 2^32-1
        /// </summary>
        millis: Longword;
        /// <summary>
        /// Roll-arounds of millis
        /// </summary>
        millis_overflow: Word;
        /// <summary>
        /// Microseconds: 0..999
        /// </summary>
        micros: Word;
    end;
    PTPCANTimestamp = ^TPCANTimestamp;

    /// <summary>
    /// PCAN-Basic API class implementation
    /// </summary>
    TPCANBasic = class
    public
        class var
            /// <summary>
            /// Undefined/default value for a PCAN bus
            /// </summary>
            const PCAN_NONEBUS: TPCANHandle = $00;

            /// <summary>
            /// PCAN-ISA interface, channel 1
            /// </summary>
            const PCAN_ISABUS1: TPCANHandle = $21;
            /// <summary>
            /// PCAN-ISA interface, channel 2
            /// </summary>
            const PCAN_ISABUS2: TPCANHandle = $22;
            /// <summary>
            /// PCAN-ISA interface, channel 3
            /// </summary>
            const PCAN_ISABUS3: TPCANHandle = $23;
            /// <summary>
            /// PCAN-ISA interface, channel 4
            /// </summary>
            const PCAN_ISABUS4: TPCANHandle = $24;
            /// <summary>
            /// PCAN-ISA interface, channel 5
            /// </summary>
            const PCAN_ISABUS5: TPCANHandle = $25;
            /// <summary>
            /// PCAN-ISA interface, channel 6
            /// </summary>
            const PCAN_ISABUS6: TPCANHandle = $26;
            /// <summary>
            /// PCAN-ISA interface, channel 7
            /// </summary>
            const PCAN_ISABUS7: TPCANHandle = $27;
            /// <summary>
            /// PCAN-ISA interface, channel 8
            /// </summary>
            const PCAN_ISABUS8: TPCANHandle = $28;

            /// <summary>
            /// PPCAN-Dongle/LPT interface, channel 1
            /// </summary>
            const PCAN_DNGBUS1: TPCANHandle = $31;

            /// <summary>
            /// PCAN-PCI interface, channel 1
            /// </summary>
            const PCAN_PCIBUS1: TPCANHandle = $41;
            /// <summary>
            /// PCAN-PCI interface, channel 2
            /// </summary>
            const PCAN_PCIBUS2: TPCANHandle = $42;
            /// <summary>
            /// PCAN-PCI interface, channel 3
            /// </summary>
            const PCAN_PCIBUS3: TPCANHandle = $43;
            /// <summary>
            /// PCAN-PCI interface, channel 4
            /// </summary>
            const PCAN_PCIBUS4: TPCANHandle = $44;
            /// <summary>
            /// PCAN-PCI interface, channel 5
            /// </summary>
            const PCAN_PCIBUS5: TPCANHandle = $45;
            /// <summary>
            /// PCAN-PCI interface, channel 6
            /// </summary>
            const PCAN_PCIBUS6: TPCANHandle = $46;
            /// <summary>
            /// PCAN-PCI interface, channel 7
            /// </summary>
            const PCAN_PCIBUS7: TPCANHandle = $47;
            /// <summary>
            /// PCAN-PCI interface, channel 8
            /// </summary>
            const PCAN_PCIBUS8: TPCANHandle = $48;

            /// <summary>
            /// PCAN-USB interface, channel 1
            /// </summary>
            const PCAN_USBBUS1: TPCANHandle = $51;
            /// <summary>
            /// PCAN-USB interface, channel 2
            /// </summary>
            const PCAN_USBBUS2: TPCANHandle = $52;
            /// <summary>
            /// PCAN-USB interface, channel 3
            /// </summary>
            const PCAN_USBBUS3: TPCANHandle = $53;
            /// <summary>
            /// PCAN-USB interface, channel 4
            /// </summary>
            const PCAN_USBBUS4: TPCANHandle = $54;
            /// <summary>
            /// PCAN-USB interface, channel 5
            /// </summary>
            const PCAN_USBBUS5: TPCANHandle = $55;
            /// <summary>
            /// PCAN-USB interface, channel 6
            /// </summary>
            const PCAN_USBBUS6: TPCANHandle = $56;
            /// <summary>
            /// PCAN-USB interface, channel 7
            /// </summary>
            const PCAN_USBBUS7: TPCANHandle = $57;
            /// <summary>
            /// PCAN-USB interface, channel 8
            /// </summary>
            const PCAN_USBBUS8: TPCANHandle = $58;

            /// <summary>
            /// PCAN-PC Card interface, channel 1
            /// </summary>
            const PCAN_PCCBUS1: TPCANHandle = $61;
            /// <summary>
            /// PCAN-PC Card interface, channel 2
            /// </summary>
            const PCAN_PCCBUS2: TPCANHandle = $62;

            /// <summary>
            /// The PCAN parameter is not set (inactive)
            /// </summary>
            const PCAN_PARAMETER_OFF: Integer = 0;
            /// <summary>
            /// The PCAN parameter is set (active)
            /// </summary>
            const PCAN_PARAMETER_ON: Integer  = 1;
            /// <summary>
            /// The PCAN filter is closed. No messages will be received
            /// </summary>
            const PCAN_FILTER_CLOSE: Integer = 0;
            /// <summary>
            /// The PCAN filter is fully opened. All messages will be received
            /// </summary>
            const PCAN_FILTER_OPEN: Integer = 1;
            /// <summary>
            /// The PCAN filter is custom configured. Only registered
            /// messages will be received
            /// </summary>
            const PCAN_FILTER_CUSTOM: Integer = 2;
            /// <summary>
            /// The PCAN-Channel handle is illegal, or its associated hadware is not available
            /// </summary>
            const PCAN_CHANNEL_UNAVAILABLE: Integer = 0;
            /// <summary>
            /// The PCAN-Channel handle is available to be connected (Plug&Play Hardware: it means futhermore that the hardware is plugged-in)
            /// </summary>
            const PCAN_CHANNEL_AVAILABLE: Integer = 1;
            /// <summary>
            /// The PCAN-Channel handle is valid, and is already being used
            /// </summary>
            const PCAN_CHANNEL_OCCUPIED: Integer = 2;

            /// <summary>
            /// Logs system exceptions / errors
            /// </summary>
            const LOG_FUNCTION_DEFAULT: Integer = $00;
            /// <summary>
            /// Logs the entries to the PCAN-Basic API functions
            /// </summary>
            const LOG_FUNCTION_ENTRY: Integer = $01;
            /// <summary>
            /// Logs the parameters passed to the PCAN-Basic API functions
            /// </summary>
            const LOG_FUNCTION_PARAMETERS: Integer = $02;
            /// <summary>
            /// Logs the exits from the PCAN-Basic API functions
            /// </summary>
            const LOG_FUNCTION_LEAVE: Integer = $04;
            /// <summary>
            /// Logs the CAN messages passed to the CAN_Write function
            /// </summary>
            const LOG_FUNCTION_WRITE: Integer = $08;
            /// <summary>
            /// Logs the CAN messages received within the CAN_Read function
            /// </summary>
            const LOG_FUNCTION_READ: Integer = $10;
            /// <summary>
            /// Logs all possible information within the PCAN-Basic API functions
            /// </summary>            
            const LOG_FUNCTION_ALL: Integer = $FFFF;

        /// <summary>
        /// Initializes a PCAN Channel 
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        /// <param name="HwType">NON PLUG&PLAY: The type of hardware and operation mode</param>
        /// <param name="IOPort">NON PLUG&PLAY: The I/O address for the parallel port</param>
        /// <param name="Interrupt">NON PLUG&PLAY: Interrupt number of the parallel port</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Initialize(
            Channel: TPCANHandle; 
            Btr0Btr1: TPCANBaudrate; 
            HwType: TPCANType; 
            IOPort: LongWord;
            Interrupt: Word
            ): TPCANStatus; overload;

        /// <summary>
        /// Initializes a PCAN Channel
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Initialize(
            Channel: TPCANHandle; 
            Btr0Btr1: TPCANBaudrate
            ): TPCANStatus; overload;

        /// <summary>
        /// Uninitializes one or all PCAN Channels initialized by CAN_Initialize
        /// </summary>
        /// <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
        /// uninitialize all initialized channels</remarks>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Uninitialize(
            Channel: TPCANHandle
            ): TPCANStatus;

        /// <summary>
        /// Resets the receive and transmit queues of the PCAN Channel
        /// </summary>
        /// <remarks>A reset of the CAN controller is not performed</remarks>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Reset(
            Channel: TPCANHandle
            ): TPCANStatus;

        /// <summary>
        /// Gets the current status of a PCAN Channel
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <returns>A TPCANStatus error code</returns>
        class function GetStatus(
            Channel: TPCANHandle
            ): TPCANStatus;

        /// <summary>
        /// Reads a CAN message from the receive queue of a PCAN Channel
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>
        /// <param name="TimestampBuffer">A TPCANTimestamp structure buffer to get
        /// the reception time of the message</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Read(
            Channel: TPCANHandle; 
            var MessageBuffer: TPCANMsg; 
            var TimestampBuffer: TPCANTimestamp
            ):TPCANStatus; overload;

        /// <summary>
        /// Reads a CAN message from the receive queue of a PCAN Channel
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>        
        /// <returns>A TPCANStatus error code</returns>
        class function Read(
            Channel: TPCANHandle; 
            var MessageBuffer: TPCANMsg
            ): TPCANStatus; overload;

        /// <summary>
        ///  Transmits a CAN message 
        /// </summary>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="MessageBuffer">A TPCANMsg buffer with the message to be sent</param>
        /// <returns>A TPCANStatus error code</returns>
        class function Write(
            Channel: TPCANHandle; 
            var MessageBuffer: TPCANMsg
            ): TPCANStatus;

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
        class function FilterMessages(
            Channel: TPCANHandle; 
            FromID: LongWord; 
            ToID: LongWord; 
            Mode: TPCANMode
            ): TPCANStatus;

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
        class function GetValue(
            Channel: TPCANHandle;
            Parameter: TPCANParameter;
            NumericBuffer: PLongWord;
            BufferLength: LongWord
            ): TPCANStatus; overload;


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
        class function GetValue(
            Channel: TPCANHandle;
            Parameter: TPCANParameter;
            StringBuffer: PAnsiChar;
            BufferLength: LongWord
            ): TPCANStatus; overload;

        /// <summary>
        /// Configures or sets a PCAN Channel value
        /// </summary>
        /// <remarks>Parameters can be present or not according with the kind
        /// of Hardware (PCAN Channel) being used. If a parameter is not available,
        /// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="Parameter">The TPCANParameter parameter to set</param>
        /// <param name="NumericBuffer">Buffer with the value to be set</param>
        /// <param name="BufferLength">Size in bytes of the buffer</param>
        /// <returns>A TPCANStatus error code</returns>
        class function SetValue(
            Channel: TPCANHandle;
            Parameter: TPCANParameter;
            NumericBuffer: PLongWord;
            BufferLength: LongWord
            ): TPCANStatus; overload;

        /// <summary>
        /// Configures or sets a PCAN Channel value
        /// </summary>
        /// <remarks>Parameters can be present or not according with the kind
        /// of Hardware (PCAN Channel) being used. If a parameter is not available,
        /// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        /// <param name="Channel">The handle of a PCAN Channel</param>
        /// <param name="Parameter">The TPCANParameter parameter to set</param>
        /// <param name="StringBuffer">Buffer with the value to be set</param>
        /// <param name="BufferLength">Size in bytes of the buffer</param>
        /// <returns>A TPCANStatus error code</returns>
        class function SetValue(
            Channel: TPCANHandle;
            Parameter: TPCANParameter;
            StringBuffer: PAnsiChar;
            BufferLength: LongWord
            ): TPCANStatus; overload;

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
        class function GetErrorText(
            Error: TPCANStatus; 
            Language: Word; 
            StringBuffer: PAnsiChar
            ): TPCANStatus;
    end;

implementation
uses SysUtils;
const DLL_Name = 'PCANBASIC.DLL';

function CAN_Initialize(Channel: TPCANHandle; Btr0Btr1: TPCANBaudrate; HwType: TPCANType; IOPort: LongWord; Interrupt: Word): TPCANStatus; stdcall;
external DLL_Name;
function CAN_Uninitialize(Channel: TPCANHandle): TPCANStatus; stdcall;
external DLL_Name;
function CAN_Reset(Channel: TPCANHandle): TPCANStatus; stdcall;
external DLL_Name;
function CAN_GetStatus(Channel: TPCANHandle): TPCANStatus; stdcall;
external DLL_Name;
function CAN_Read(Channel: TPCANHandle; var MessageBuffer: TPCANMsg; TimestampBuffer: PTPCANTimestamp):TPCANStatus; overload; stdcall;
external DLL_Name;
function CAN_Write(Channel: TPCANHandle; var MessageBuffer: TPCANMsg): TPCANStatus; stdcall;
external DLL_Name;
function CAN_FilterMessages(Channel: TPCANHandle; FromID: LongWord; ToID: LongWord; Mode: TPCANMode): TPCANStatus; stdcall;
external DLL_Name;
function CAN_GetValue(Channel: TPCANHandle; Parameter: TPCANParameter; Buffer: Pointer; BufferLength: LongWord): TPCANStatus; stdcall;
external DLL_Name;
function CAN_SetValue(Channel: TPCANHandle; Parameter: TPCANParameter; Buffer: Pointer; BufferLength: LongWord): TPCANStatus; stdcall;
external DLL_Name;
function CAN_GetErrorText(Error: TPCANStatus; Language: Word; StringBuffer: PAnsiChar): TPCANStatus; stdcall;
external DLL_Name;

class function TPCANBasic.Initialize(Channel: TPCANHandle; Btr0Btr1: TPCANBaudrate; HwType: TPCANType; IOPort: LongWord; Interrupt: Word): TPCANStatus;
begin
    Result:= CAN_Initialize(Channel,Btr0Btr1,HwType,IOPort,Interrupt);
end;

class function TPCANBasic.Initialize(Channel: TPCANHandle; Btr0Btr1: TPCANBaudrate): TPCANStatus;
begin
    Result:= CAN_Initialize(Channel,Btr0Btr1,TPCANType(0), 0,0);
end;

class function TPCANBasic.Uninitialize(Channel: TPCANHandle): TPCANStatus;
begin
    Result:= CAN_Uninitialize(Channel);
end;

class function TPCANBasic.Reset(Channel: TPCANHandle): TPCANStatus;
begin
    Result:= CAN_Reset(Channel);
end;

class function TPCANBasic.GetStatus(Channel: TPCANHandle): TPCANStatus;
begin
    Result:= CAN_GetStatus(Channel);
end;

class function TPCANBasic.Read(Channel: TPCANHandle; var MessageBuffer: TPCANMsg; var TimestampBuffer: TPCANTimestamp):TPCANStatus;
begin
    Result:= CAN_Read(Channel, MessageBuffer, @TimestampBuffer);
end;

class function TPCANBasic.Read(Channel: TPCANHandle; var MessageBuffer: TPCANMsg):TPCANStatus;
begin
    Result:= CAN_Read(Channel, MessageBuffer, nil);
end;

class function TPCANBasic.Write(Channel: TPCANHandle; var MessageBuffer: TPCANMsg): TPCANStatus;
begin
    Result:= CAN_Write(Channel, MessageBuffer);
end;

class function TPCANBasic.FilterMessages(Channel: TPCANHandle; FromID: LongWord; ToID: LongWord; Mode: TPCANMode): TPCANStatus;
begin
    Result:= CAN_FilterMessages(Channel, FromID,ToID,Mode);
end;

class function TPCANBasic.GetValue(Channel: TPCANHandle; Parameter: TPCANParameter; NumericBuffer: PLongWord; BufferLength: LongWord): TPCANStatus;
begin
    Result:= CAN_GetValue(Channel, Parameter, NumericBuffer, BufferLength);
end;

class function TPCANBasic.GetValue(Channel: TPCANHandle; Parameter: TPCANParameter; StringBuffer: PAnsiChar; BufferLength: LongWord): TPCANStatus;
begin
    Result:= CAN_GetValue(Channel, Parameter, StringBuffer, BufferLength);
end;

class function TPCANBasic.SetValue(Channel: TPCANHandle; Parameter: TPCANParameter; NumericBuffer: PLongWord; BufferLength: LongWord): TPCANStatus;
begin
    Result:= CAN_SetValue(Channel, Parameter, NumericBuffer, BufferLength);
end;

class function TPCANBasic.SetValue(Channel: TPCANHandle; Parameter: TPCANParameter; StringBuffer: PAnsiChar; BufferLength: LongWord): TPCANStatus;
begin
    Result:= CAN_SetValue(Channel, Parameter, StringBuffer, BufferLength);
end;

class function TPCANBasic.GetErrorText(Error: TPCANStatus; Language: Word; StringBuffer: PAnsiChar): TPCANStatus;
begin
    Result:= CAN_GetErrorText(Error, Language, StringBuffer);
end;
end.
