/*
 OpenBCI 32bit Library
 Place the containing folder into your libraries folder insdie the arduino folder in your Documents folder
 This library will work with a single OpenBCI 32bit board, or
 an OpenBCI 32bit board with an OpenBCI Daisy Module attached.
 */

#include "OpenBCI.h"
#include "stm32f4xx_hal.h"
#include "serial.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "usb.h"
#include "main.h"
#include "spi.h"

#define false 0
#define true 1

uint8_t gains[8];

extern osSemaphoreId adcDataReadySemathoreHandle;

//static const char * const pcWelcomeMessage =
//	"\r\n ______     __  __     __   __     ______     ______   ______    " \
//	"\r\n/\\  ___\\   /\\ \\_\\ \\   /\\ \"-.\\ \\   /\\  __ \\   /\\  == \\ /\\  ___\\   " \
//	"\r\n\\ \\___  \\  \\ \\____ \\  \\ \\ \\-.  \\  \\ \\  __ \\  \\ \\  _-/ \\ \\___  \\  " \
//	"\r\n \\/\\_____\\  \\/\\_____\\  \\ \\_\\\\\"\\_\\  \\ \\_\\ \\_\\  \\ \\_\\    \\/\\_____\\ " \
//	"\r\n  \\/_____/   \\/_____/   \\/_/ \\/_/   \\/_/\\/_/   \\/_/     \\/_____/ " \
//	"\r\n                                                                 \r\n\0";

void
OpenBCITaskWorker ( void const * argument )
{
    initializeVariables ();

    // Bring the board up
    boardBegin ();

    for ( ;; )
    {
	if (streaming)
	{
	    if ( osOK == osSemaphoreWait(adcDataReadySemathoreHandle,100) )
	    {
		updateChannelData();
		sendChannelData();
	    }
	}
    }
}

void
OpenBCICommandTaskWorker ( void const * argument )
{

    for ( ;; )
    {
	char byte = 0;
	if ( usb_check_get_rx_byte( &byte ) )
	    processChar(byte);

	loop();

	osDelay(1);
    }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ADC_DRDY_Pin)
    {
	osSemaphoreRelease(adcDataReadySemathoreHandle);
    }
}

/**
 * @description: The function the OpenBCI board will call in setup. Turns sniff mode
 *  on and allows you to tap into the serial port that is broken out on the OpenBCI
 *  32bit board. You must alter this file:
 *   On Mac:
 *     `/Users/username/Documents/Arduino/hardware/chipkit-core/pic32/variants/openbci/Board_Defs.h`
 *   On Windows:
 *     `C:\Users\username\Documents\Arduino\hardware\chipkit-core\pic32\variants\openbci\Board_Defs.h`
 * Specifically lines `311` and `313` from `7` and `10` to `11` and `12` for
 *   `_SER1_TX_PIN` and `_SER1_RX_PIN` respectively. Check out this sweet gif if
 *   you are a visual person http://g.recordit.co/3jH01sMD6Y.gif
 *  You will need to reflash your board! But now you can connect to pins `11`
 *    `12` via a FTDI serial port driver, really any serial to USB driver would
 *    work. Remember to use 3V3, 115200 baud, and have a common ground!
 * @author: AJ Keller (@pushtheworldllc)
 */
void
beginDebug ( void )
{
    beginDebugBaudRate (OPENBCI_BAUD_RATE);
}

void
beginDebugBaudRate ( uint32_t baudRate )
{
    // Bring the board up
    uint8_t started = boardBeginDebugBaudRate (baudRate);

    if ( started )
    {
	vSerialPutString ("Board up\r\n");
	sendEOT ();
    }
    else
    {
	vSerialPutString ("Board err\r\n");
	sendEOT ();
    }
}

/**
 * @description Process one char at a time from serial port. This is the main
 *  command processor for the OpenBCI system. Considered mission critical for
 *  normal operation.
 * @param `character` {char} - The character to process.
 * @return {uint8_t} - `true` if the command was recognized, `false` if not
 */
uint8_t
processChar ( char character )
{
    if ( curBoardMode == BOARD_MODE_DEBUG || curDebugMode == DEBUG_MODE_ON )
    {
	vSerialPutString ("pC: ");
	vSerialPutCharLn (character);
    }

    if ( checkMultiCharCmdTimer () )
    { // we are in a multi char command
	switch ( getMultiCharCommand () )
	    {
	    case MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_CHANNEL:
		processIncomingChannelSettings (character);
		break;
	    case MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_LEADOFF:
		processIncomingLeadOffSettings (character);
		break;
	    case MULTI_CHAR_CMD_SETTINGS_BOARD_MODE:
		processIncomingBoardMode (character);
		break;
	    case MULTI_CHAR_CMD_SETTINGS_SAMPLE_RATE:
		processIncomingSampleRate (character);
		break;
	    case MULTI_CHAR_CMD_INSERT_MARKER:
		processInsertMarker (character);
		break;
	    default:
		break;
	    }
    }
    else
    { // Normal...
	switch ( character )
	    {
	    //TURN CHANNELS ON/OFF COMMANDS
	    case OPENBCI_CHANNEL_OFF_1:
		streamSafeChannelDeactivate (1);
		break;
	    case OPENBCI_CHANNEL_OFF_2:
		streamSafeChannelDeactivate (2);
		break;
	    case OPENBCI_CHANNEL_OFF_3:
		streamSafeChannelDeactivate (3);
		break;
	    case OPENBCI_CHANNEL_OFF_4:
		streamSafeChannelDeactivate (4);
		break;
	    case OPENBCI_CHANNEL_OFF_5:
		streamSafeChannelDeactivate (5);
		break;
	    case OPENBCI_CHANNEL_OFF_6:
		streamSafeChannelDeactivate (6);
		break;
	    case OPENBCI_CHANNEL_OFF_7:
		streamSafeChannelDeactivate (7);
		break;
	    case OPENBCI_CHANNEL_OFF_8:
		streamSafeChannelDeactivate (8);
		break;

	    case OPENBCI_CHANNEL_ON_1:
		streamSafeChannelActivate (1);
		break;
	    case OPENBCI_CHANNEL_ON_2:
		streamSafeChannelActivate (2);
		break;
	    case OPENBCI_CHANNEL_ON_3:
		streamSafeChannelActivate (3);
		break;
	    case OPENBCI_CHANNEL_ON_4:
		streamSafeChannelActivate (4);
		break;
	    case OPENBCI_CHANNEL_ON_5:
		streamSafeChannelActivate (5);
		break;
	    case OPENBCI_CHANNEL_ON_6:
		streamSafeChannelActivate (6);
		break;
	    case OPENBCI_CHANNEL_ON_7:
		streamSafeChannelActivate (7);
		break;
	    case OPENBCI_CHANNEL_ON_8:
		streamSafeChannelActivate (8);
		break;

		// TEST SIGNAL CONTROL COMMANDS
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_GROUND:
		activateAllChannelsToTestCondition (ADSINPUT_SHORTED,
						    ADSTESTSIG_NOCHANGE,
						    ADSTESTSIG_NOCHANGE);
		break;
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_1X_SLOW:
		activateAllChannelsToTestCondition (ADSINPUT_TESTSIG,
						    ADSTESTSIG_AMP_1X,
						    ADSTESTSIG_PULSE_SLOW);
		break;
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_1X_FAST:
		activateAllChannelsToTestCondition (ADSINPUT_TESTSIG,
						    ADSTESTSIG_AMP_1X,
						    ADSTESTSIG_PULSE_FAST);
		break;
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_DC:
		activateAllChannelsToTestCondition (ADSINPUT_TESTSIG,
						    ADSTESTSIG_AMP_2X,
						    ADSTESTSIG_DCSIG);
		break;
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_2X_SLOW:
		activateAllChannelsToTestCondition (ADSINPUT_TESTSIG,
						    ADSTESTSIG_AMP_2X,
						    ADSTESTSIG_PULSE_SLOW);
		break;
	    case OPENBCI_TEST_SIGNAL_CONNECT_TO_PULSE_2X_FAST:
		activateAllChannelsToTestCondition (ADSINPUT_TESTSIG,
						    ADSTESTSIG_AMP_2X,
						    ADSTESTSIG_PULSE_FAST);
		break;

		// CHANNEL SETTING COMMANDS
	    case OPENBCI_CHANNEL_CMD_SET: // This is a multi char command with a timeout
		startMultiCharCmdTimer (
			MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_CHANNEL);
		numberOfIncomingSettingsProcessedChannel = 1;
		break;

		// LEAD OFF IMPEDANCE DETECTION COMMANDS
	    case OPENBCI_CHANNEL_IMPEDANCE_SET:
		startMultiCharCmdTimer (
			MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_LEADOFF);
		numberOfIncomingSettingsProcessedLeadOff = 1;
		break;

	    case OPENBCI_CHANNEL_DEFAULT_ALL_SET: // reset all channel settings to default
		if ( !streaming )
		{
		    printAllStr ("updating channel settings to");
		    printAllStr (" default");
		    sendEOT ();
		}
		streamSafeSetAllChannelsToDefault ();
		break;
	    case OPENBCI_CHANNEL_DEFAULT_ALL_REPORT: // report the default settings
		reportDefaultChannelSettings ();
		break;

		// STREAM DATA AND FILTER COMMANDS
	    case OPENBCI_STREAM_START: // stream data
		streamStart (); // turn on the fire hose
		break;

	    case OPENBCI_STREAM_STOP: // stop streaming data
		streamStop();
		break;

		//  INITIALIZE AND VERIFY
	    case OPENBCI_MISC_SOFT_RESET:
		boardReset (); // initialize ADS and read device IDs
		break;

		//  QUERY THE ADS AND ACCEL REGITSTERS
	    case OPENBCI_MISC_QUERY_REGISTER_SETTINGS:
		if ( !streaming )
		{
		    printAllRegisters (); // print the ADS and accelerometer register values
		}
		break;

		// TIME SYNC
	    case OPENBCI_TIME_SET:
		// Set flag to send time packet
		if ( !streaming )
		{
		    printAllStr ("Time stamp ON");
		    sendEOT ();
		}
		curTimeSyncMode = TIME_SYNC_MODE_ON;
		setCurPacketType ();
		break;

	    case OPENBCI_TIME_STOP:
		// Stop the Sync
		if ( !streaming )
		{
		    printAllStr ("Time stamp OFF");
		    sendEOT ();
		}
		curTimeSyncMode = TIME_SYNC_MODE_OFF;
		setCurPacketType ();
		break;

		// BOARD TYPE SET TYPE
	    case OPENBCI_BOARD_MODE_SET:
		startMultiCharCmdTimer (MULTI_CHAR_CMD_SETTINGS_BOARD_MODE);
		optionalArgCounter = 0;
		break;

		// Sample rate set
	    case OPENBCI_SAMPLE_RATE_SET:
		startMultiCharCmdTimer (MULTI_CHAR_CMD_SETTINGS_SAMPLE_RATE);
		break;

		// Insert Marker into the EEG data stream
	    case OPENBCI_INSERT_MARKER:
		startMultiCharCmdTimer (MULTI_CHAR_CMD_INSERT_MARKER);
		break;

	    case OPENBCI_GET_VERSION:
		printAllStr ("v3.1.2");
		sendEOT ();
		break;
	    default:
		return false;
	    }
    }
    return true;
}

/**
 * Start the timer on multi char commands
 * @param cmd {char} the command received on the serial stream. See enum MULTI_CHAR_COMMAND
 * @returns void
 */
void
startMultiCharCmdTimer ( char cmd )
{
    if ( curDebugMode == DEBUG_MODE_ON )
    {
	vSerialPutString ("Start multi char: ");
	vSerialPutCharLn (cmd);
    }
    isMultiCharCmd = true;
    multiCharCommand = cmd;
    multiCharCmdTimeout = xTaskGetTickCount () + MULTI_CHAR_COMMAND_TIMEOUT_MS;
}

/**
 * End the timer on multi char commands
 * @param None
 * @returns void
 */
void
endMultiCharCmdTimer ( void )
{
    isMultiCharCmd = false;
    multiCharCommand = MULTI_CHAR_CMD_NONE;
}

/**
 * Check for valid on multi char commands
 * @param None
 * @returns {uint8_t} true if a multi char commands is active and the timer is running, otherwise False
 */
uint8_t
checkMultiCharCmdTimer ( void )
{
    if ( isMultiCharCmd )
    {
	if ( xTaskGetTickCount () < multiCharCmdTimeout )
	    return true;
	else
	{ // the timer has timed out - reset the multi char timeout
	    endMultiCharCmdTimer ();
	    printAllStr ("Timeout processing multi byte");
	    printAllStr (" message - please send all");
	    printAllStr (" commands at once as of v2");
	    sendEOT ();
	}
    }
    return false;
}

/**
 * To be called at some point in every loop function
 */
void
loop ( void )
{
    if ( isMultiCharCmd )
    {
	checkMultiCharCmdTimer ();
    }
}

/**
 * Gets the active multi char command
 * @param None
 * @returns {char} multiCharCommand
 */
char
getMultiCharCommand ( void )
{
    return multiCharCommand;
}

/**
 * Used to turn on or off time syncing/stamping, will change the current packet type!
 * @param yes {uint8_t} - True if you want to use it
 */
void
useTimeStamp ( uint8_t yes )
{
    curTimeSyncMode = yes ? TIME_SYNC_MODE_ON : TIME_SYNC_MODE_OFF;
    setCurPacketType ();
}

/**
 * @description: This is a function that is called once and confiures all pins on
 *                 the PIC32 uC
 * @author: AJ Keller (@pushtheworldllc)
 */
uint8_t
boardBegin ( void )
{
    // Do a soft reset
    boardReset ();

    return true;
}

void
boardBeginADSInterrupt ( void )
{
//  Startup for interrupt
//  setIntVector(_EXTERNAL_4_VECTOR, ADS_DRDY_Service); // connect interrupt to ISR
//  setIntPriority(_EXTERNAL_4_VECTOR, 4, 0);           // set interrupt priority and sub priority
//  clearIntFlag(_EXTERNAL_4_IRQ);                      // these two need to be done together
//  setIntEnable(_EXTERNAL_4_IRQ);                      // clear any flags before enabing the irq
}

/**
 * @description: This is a function that is called once and confiures all pins on
 *                 the PIC32 uC
 * @author: AJ Keller (@pushtheworldllc)
 */
uint8_t
boardBeginDebug ( void )
{
    return boardBeginDebugBaudRate (115200);
}

/**
 * @description: This is a function that is called once and confiures the Pic to run in secondary serial mode
 * @param baudRate {int} - The baudRate you want the secondary serial port to run at.
 * @author: AJ Keller (@pushtheworldllc)
 */
uint8_t
boardBeginDebugBaudRate ( int baudRate )
{
    curBoardMode = BOARD_MODE_DEBUG;
    curDebugMode = DEBUG_MODE_ON;

    // Startup for interrupt
    boardBeginADSInterrupt ();

    // Do a soft reset
    boardReset ();

    return true;
}

/**
 * @description: This is a function that can be called multiple times, this is
 *                 what we refer to as a `soft reset`. You will hear/see this
 *                 many times.
 * @author: AJ Keller (@pushtheworldllc)
 */
void
boardReset ( void )
{
    initialize (); // initalizes on-board ADS
    osDelay (500);
    configureLeadOffDetection (LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);

    //printlnAllStr ( pcWelcomeMessage );
    printlnAllStr ("OpenBCI V3 8-16 channel");

    printAllStr ("ADS1299 Device ID: 0x");
    printlnHex (ADS_getDeviceID (ON_BOARD));

    printlnAllStr ("LIS3DH Device ID: 0x33");

    printlnAllStr ("Firmware: v3.1.2");
    sendEOT ();
    osDelay (5);
}

/**
 * @description: Simple method to send the EOT over serial...
 * @author: AJ Keller (@pushtheworldllc)
 */
void
sendEOT ( void )
{
    printSerialStr ("$$$");
}

void
activateAllChannelsToTestCondition ( uint8_t testInputCode,
				     uint8_t amplitudeCode, uint8_t freqCode )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    //set the test signal to the desired state
    configureInternalTestSignal (amplitudeCode, freqCode);
    //change input type settings for all channels
    changeInputType (testInputCode);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
    else
    {
	printSuccess ();
	printAllStr ("Configured internal");
	printAllStr (" test signal.");
	sendEOT ();
    }
}

void
processIncomingBoardMode ( char c )
{
    if ( c == OPENBCI_BOARD_MODE_SET )
    {
	printSuccess ();
	printAllStr (getBoardMode ());
	sendEOT ();
    }
    else if ( (c >= '0') & (c <= '9') )
    {
	uint8_t digit = c - '0';
	if ( digit < BOARD_MODE_END_OF_MODES )
	{
	    setBoardMode (digit);
	    printSuccess ();
	    printAllStr (getBoardMode ());
	    sendEOT ();
	}
	else
	{
	    printFailure ();
	    printAllStr ("board mode value");
	    printAllStr (" out of bounds.");
	    sendEOT ();
	}
    }
    else
    {
	printFailure ();
	printAllStr ("invalid board mode value.");
	sendEOT ();
    }
    endMultiCharCmdTimer ();
}

/**
 * Used to set the board mode of the system.
 * @param newBoardMode The board mode to swtich to
 */
void
setBoardMode ( uint8_t newBoardMode )
{
    if ( curBoardMode == (BOARD_MODE) newBoardMode )
	return;
    curBoardMode = (BOARD_MODE) newBoardMode;
    switch ( curBoardMode )
	{
	case BOARD_MODE_ANALOG:
	    curAccelMode = ACCEL_MODE_OFF;
//	    beginPinsAnalog ();
	    break;
	case BOARD_MODE_DIGITAL:
	    curAccelMode = ACCEL_MODE_OFF;
//	    beginPinsDigital ();
	    break;
	case BOARD_MODE_DEBUG:
	    curDebugMode = DEBUG_MODE_ON;
//	    beginPinsDebug ();
//	    beginSerial1 ();
	    break;
	case BOARD_MODE_DEFAULT:
//	    endSerial1 ();
//	    beginPinsDefault ();
//	    endSerial0 ();
//	    beginSerial0BaudRate (OPENBCI_BAUD_RATE);
	    break;
	case BOARD_MODE_MARKER:
	    curAccelMode = ACCEL_MODE_OFF;
	    break;
	default:
	    break;
	}
    osDelay (10);
    setCurPacketType ();
}

void
setSampleRate ( uint8_t newSampleRateCode )
{
    curSampleRate = (SAMPLE_RATE) newSampleRateCode;
    initialize_ads ();
}

const char *
getSampleRate ()
{
    switch ( curSampleRate )
	{
	case SAMPLE_RATE_16000:
	    return "16000";
	case SAMPLE_RATE_8000:
	    return "8000";
	case SAMPLE_RATE_4000:
	    return "4000";
	case SAMPLE_RATE_2000:
	    return "2000";
	case SAMPLE_RATE_1000:
	    return "1000";
	case SAMPLE_RATE_500:
	    return "500";
	case SAMPLE_RATE_250:
	default:
	    return "250";
	}
}

const char *
getBoardMode ( void )
{
    switch ( curBoardMode )
	{
	case BOARD_MODE_DEBUG:
	    return "debug";
	case BOARD_MODE_ANALOG:
	    return "analog";
	case BOARD_MODE_DIGITAL:
	    return "digital";
	case BOARD_MODE_MARKER:
	    return "marker";
	case BOARD_MODE_BLE:
	    return "BLE";
	case BOARD_MODE_DEFAULT:
	default:
	    return "default";
	}
}

void
processIncomingSampleRate ( char c )
{
    if ( c == OPENBCI_SAMPLE_RATE_SET )
    {
	printSuccess ();
	printAllStr ("Sample rate is ");
	printAllStr (getSampleRate ());
	printAllStr ("Hz");
	sendEOT ();
    }
    else if ( (c >= '0') & (c <= '9') )
    {
	uint8_t digit = c - '0';
	if ( digit <= SAMPLE_RATE_250 )
	{
	    streamSafeSetSampleRate ((SAMPLE_RATE) digit);
	    if ( !streaming )
	    {
		printSuccess ();
		printAllStr ("Sample rate is ");
		printAllStr (getSampleRate ());
		printAllStr ("Hz");
		sendEOT ();
	    }
	}
	else
	{
	    if ( !streaming )
	    {
		printFailure ();
		printAllStr ("sample value out of bounds");
		sendEOT ();
	    }
	}
    }
    else
    {
	if ( !streaming )
	{
	    printFailure ();
	    printAllStr ("invalid sample value");
	    sendEOT ();
	}
    }
    endMultiCharCmdTimer ();
}

/**
 * @description When a '`x' is found on the serial port it is a signal to insert a marker
 *      of value x into the AUX1 stream (auxData[0]). This function sets the flag to indicate that a new marker
 *      is available. The marker will be inserted during the serial and sd write functions
 * @param character {char} - The character that will be inserted into the data stream
 */
void
processInsertMarker ( char c )
{
    markerValue = c;
    newMarkerReceived = true;
    endMultiCharCmdTimer ();
}

/**
 * @description When a 'x' is found on the serial port, we jump to this function
 *                  where we continue to read from the serial port and read the
 *                  remaining 7 bytes.
 */
void
processIncomingChannelSettings ( char character )
{

    if ( character == OPENBCI_CHANNEL_CMD_LATCH
	    && numberOfIncomingSettingsProcessedChannel
		    < OPENBCI_NUMBER_OF_BYTES_SETTINGS_CHANNEL - 1 )
    {
	// We failed somehow and should just abort
	numberOfIncomingSettingsProcessedChannel = 0;

	// put flag back down
	endMultiCharCmdTimer ();

	if ( !streaming )
	{
	    printFailure ();
	    printAllStr ("too few chars");
	    sendEOT ();
	}

	return;
    }
    switch ( numberOfIncomingSettingsProcessedChannel )
	{
	case 1: // channel number
	    currentChannelSetting = getChannelCommandForAsciiChar (character);
	    break;
	case 2: // POWER_DOWN
	    optionalArgBuffer7[0] = getNumberForAsciiChar (character);
	    break;
	case 3: // GAIN_SET
	    optionalArgBuffer7[1] = getGainForAsciiChar (character);
	    break;
	case 4: // INPUT_TYPE_SET
	    optionalArgBuffer7[2] = getNumberForAsciiChar (character);
	    break;
	case 5: // BIAS_SET
	    optionalArgBuffer7[3] = getNumberForAsciiChar (character);
	    break;
	case 6: // SRB2_SET
	    optionalArgBuffer7[4] = getNumberForAsciiChar (character);

	    break;
	case 7: // SRB1_SET
	    optionalArgBuffer7[5] = getNumberForAsciiChar (character);
	    break;
	case 8: // 'X' latch
	    if ( character != OPENBCI_CHANNEL_CMD_LATCH )
	    {
		if ( !streaming )
		{
		    printFailure ();
		    printAllStr ("Err: 9th char not X");
		    sendEOT ();
		}
		// We failed somehow and should just abort
		numberOfIncomingSettingsProcessedChannel = 0;

		// put flag back down
		endMultiCharCmdTimer ();
	    }
	    break;
	default: // should have exited
	    if ( !streaming )
	    {
		printFailure ();
		printAllStr ("Err: too many chars");
		sendEOT ();
	    }
	    // We failed somehow and should just abort
	    numberOfIncomingSettingsProcessedChannel = 0;

	    // put flag back down
	    endMultiCharCmdTimer ();
	    return;
	}

    // increment the number of bytes processed
    numberOfIncomingSettingsProcessedChannel++;

    if ( numberOfIncomingSettingsProcessedChannel
	    == (OPENBCI_NUMBER_OF_BYTES_SETTINGS_CHANNEL) )
    {
	// We are done processing channel settings...
	if ( !streaming )
	{
	    char buf[4];
	    printSuccess ();
	    printAllStr ("Channel set for ");
	    printAllStr (itoa (currentChannelSetting + 1, buf, 10));
	    sendEOT ();
	}

	channelSettings[currentChannelSetting][POWER_DOWN] =
		optionalArgBuffer7[0];
	channelSettings[currentChannelSetting][GAIN_SET] =
		optionalArgBuffer7[1];
	channelSettings[currentChannelSetting][INPUT_TYPE_SET] =
		optionalArgBuffer7[2];
	channelSettings[currentChannelSetting][BIAS_SET] =
		optionalArgBuffer7[3];
	channelSettings[currentChannelSetting][SRB2_SET] =
		optionalArgBuffer7[4];
	channelSettings[currentChannelSetting][SRB1_SET] =
		optionalArgBuffer7[5];

	// Set channel settings
	streamSafeChannelSettingsForChannel (
		currentChannelSetting + 1,
		channelSettings[currentChannelSetting][POWER_DOWN],
		channelSettings[currentChannelSetting][GAIN_SET],
		channelSettings[currentChannelSetting][INPUT_TYPE_SET],
		channelSettings[currentChannelSetting][BIAS_SET],
		channelSettings[currentChannelSetting][SRB2_SET],
		channelSettings[currentChannelSetting][SRB1_SET]);

	// Reset
	numberOfIncomingSettingsProcessedChannel = 0;

	// put flag back down
	endMultiCharCmdTimer ();
    }
}

/**
 * @description When a 'z' is found on the serial port, we jump to this function
 *                  where we continue to read from the serial port and read the
 *                  remaining 4 bytes.
 * @param `character` - {char} - The character you want to process...
 */
void
processIncomingLeadOffSettings ( char character )
{

    if ( character == OPENBCI_CHANNEL_IMPEDANCE_LATCH
	    && numberOfIncomingSettingsProcessedLeadOff
		    < OPENBCI_NUMBER_OF_BYTES_SETTINGS_LEAD_OFF - 1 )
    {
	// We failed somehow and should just abort
	// reset numberOfIncomingSettingsProcessedLeadOff
	numberOfIncomingSettingsProcessedLeadOff = 0;

	// put flag back down
	endMultiCharCmdTimer ();

	if ( !streaming )
	{
	    printFailure ();
	    printAllStr ("too few chars");
	    sendEOT ();
	}

	return;
    }
    switch ( numberOfIncomingSettingsProcessedLeadOff )
	{
	case 1: // channel number
	    currentChannelSetting = getChannelCommandForAsciiChar (character);
	    break;
	case 2: // pchannel setting
	    optionalArgBuffer7[0] = getNumberForAsciiChar (character);
	    break;
	case 3: // nchannel setting
	    optionalArgBuffer7[1] = getNumberForAsciiChar (character);
	    break;
	case 4: // 'Z' latch
	    if ( character != OPENBCI_CHANNEL_IMPEDANCE_LATCH )
	    {
		if ( !streaming )
		{
		    printFailure ();
		    printAllStr ("Err: 5th char not Z");
		    sendEOT ();
		}
		// We failed somehow and should just abort
		// reset numberOfIncomingSettingsProcessedLeadOff
		numberOfIncomingSettingsProcessedLeadOff = 0;

		// put flag back down
		endMultiCharCmdTimer ();
	    }
	    break;
	default: // should have exited
	    if ( !streaming )
	    {
		printFailure ();
		printAllStr ("Err: too many chars");
		sendEOT ();
	    }
	    // We failed somehow and should just abort
	    // reset numberOfIncomingSettingsProcessedLeadOff
	    numberOfIncomingSettingsProcessedLeadOff = 0;

	    // put flag back down
	    endMultiCharCmdTimer ();
	    return;
	}

    // increment the number of bytes processed
    numberOfIncomingSettingsProcessedLeadOff++;

    if ( numberOfIncomingSettingsProcessedLeadOff
	    == (OPENBCI_NUMBER_OF_BYTES_SETTINGS_LEAD_OFF) )
    {
	// We are done processing lead off settings...

	if ( !streaming )
	{
	    char buf[3];
	    printSuccess ();
	    printAllStr ("Lead off set for ");
	    printAllStr (itoa (currentChannelSetting + 1, buf, 10));
	    sendEOT ();
	}

	leadOffSettings[currentChannelSetting][PCHAN] = optionalArgBuffer7[0];
	leadOffSettings[currentChannelSetting][NCHAN] = optionalArgBuffer7[1];

	// Set lead off settings
	streamSafeLeadOffSetForChannel (
		currentChannelSetting + 1,
		leadOffSettings[currentChannelSetting][PCHAN],
		leadOffSettings[currentChannelSetting][NCHAN]);

	// reset numberOfIncomingSettingsProcessedLeadOff
	numberOfIncomingSettingsProcessedLeadOff = 0;

	// put flag back down
	endMultiCharCmdTimer ();
    }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<  BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void
initialize ()
{
//  pinMode(SD_SS, OUTPUT);
//  digitalWrite(SD_SS, HIGH); // de-select SDcard if present
//  pinMode(BOARD_ADS, OUTPUT);
//  digitalWrite(BOARD_ADS, HIGH);
//  pinMode(DAISY_ADS, OUTPUT);
//  digitalWrite(DAISY_ADS, HIGH);
//  pinMode(LIS3DH_SS, OUTPUT);
//  digitalWrite(LIS3DH_SS, HIGH);
//
//  spi.begin();
//  spi.setSpeed(4000000);      // use 4MHz for ADS and LIS3DH
//  spi.setMode(DSPI_MODE0);    // default to SD card mode!
    initialize_ads ();           // hard reset ADS, set pin directions
}

void
initializeVariables ( void )
{
    // Bools
    channelDataAvailable = false;
    commandFromSPI = false;
//    daisyPresent = false;
    endMultiCharCmdTimer (); // this initializes and resets the variables
    streaming = false;
    verbosity = false; // when verbosity is true, there will be Serial feedback

    // Nums
//    ringBufBLEHead = 0;
//    ringBufBLETail = 0;
    currentChannelSetting = 0;
    lastSampleTime = 0;
    numberOfIncomingSettingsProcessedChannel = 0;
    numberOfIncomingSettingsProcessedLeadOff = 0;
    sampleCounter = 0;
    sampleCounterBLE = 0;
    timeOfLastRead = 0;
    timeOfMultiByteMsgStart = 0;

    // Enums
    curAccelMode = ACCEL_MODE_ON;
    curBoardMode = BOARD_MODE_DEFAULT;
    curDebugMode = DEBUG_MODE_OFF;
    curPacketType = PACKET_TYPE_ACCEL;
    curSampleRate = SAMPLE_RATE_250;
    curTimeSyncMode = TIME_SYNC_MODE_OFF;

    // Structs
//  initializeSerialInfo(iSerial0);
//  initializeSerialInfo(iSerial1);
//  bufferBLEReset();
}

void
printAllRegisters ()
{
    if ( !isRunning )
    {
	printlnAll ();
	printlnAllStr ("Board ADS Registers");
	// printlnAll("");
	printADSregisters (BOARD_ADS);
//	if ( daisyPresent )
//	{
//	    printlnAll ();
//	    printlnAllStr ("Daisy ADS Registers");
//	    printADSregisters (DAISY_ADS);
//	}
	printlnAll ();
	sendEOT ();
    }
}

/**
 * Called from the .ino file as the main sender. Driven by board mode,
 *  sample number, and ultimately the current packer type.
 *
 */
void
sendChannelData ()
{
    sendChannelDataPacketType (curPacketType);
}

/**
 * @description Writes data to wifi and/or serial port.
 *
 *  Adds stop byte `OPENBCI_EOP_STND_ACCEL`. See `OpenBCI_32bit_Library_Definitions.h`
 */
void
sendChannelDataPacketType ( PACKET_TYPE packetType )
{
    sendChannelDataSerial (packetType);

    if ( packetType == PACKET_TYPE_RAW_AUX
	    || packetType == PACKET_TYPE_RAW_AUX_TIME_SYNC )
	zeroAuxData ();

    sampleCounter++;
}

/**
 * @description Writes channel data to serial port in the correct stream packet format.
 * @param `packetType` {PACKET_TYPE} - The type of packet to send
 *  Adds stop byte see `OpenBCI_32bit_Library.h` enum PACKET_TYPE
 */
void
sendChannelDataSerial ( PACKET_TYPE packetType )
{

//    writeSerial (OPENBCI_BOP);   // 1 byte - 0x41
    writeSerial (0xA0);   // 1 byte - 0x41
    writeSerial (sampleCounter); // 1 byte
    ADS_writeChannelData ();     // 24 bytes

    switch ( packetType )
	{
	case PACKET_TYPE_ACCEL:
	    writeAuxDataSerial (); // 6 bytes
//	    accelWriteAxisDataSerial (); // 6 bytes
	    break;
	case PACKET_TYPE_ACCEL_TIME_SET:
//	    sendTimeWithAccelSerial ();
	    curPacketType = PACKET_TYPE_ACCEL_TIME_SYNC;
	    break;
	case PACKET_TYPE_ACCEL_TIME_SYNC:
//	    sendTimeWithAccelSerial ();
	    break;
	case PACKET_TYPE_RAW_AUX_TIME_SET:
	    sendTimeWithRawAuxSerial ();
	    curPacketType = PACKET_TYPE_RAW_AUX_TIME_SYNC;
	    break;
	case PACKET_TYPE_RAW_AUX_TIME_SYNC:
	    sendTimeWithRawAuxSerial ();
	    break;
	case PACKET_TYPE_RAW_AUX:
	default:
	    writeAuxDataSerial (); // 6 bytes
	    break;
	}

    writeSerial ((uint8_t) (PCKT_END | packetType)); // 1 byte
}

/**
 * @description Writes channel data and `auxData` array to serial port in
 *  the correct stream packet format.
 *
 *  Adds stop byte `OPENBCI_EOP_STND_RAW_AUX`. See `OpenBCI_32bit_Library_Definitions.h`
 */
//void sendRawAuxWifi(void)
//{
//  writeAuxDataWifi(); // 6 bytes
//}

/**
 * Using publically available state variables to drive packet type settings
 */
void
setCurPacketType ( void )
{
    if ( curAccelMode == ACCEL_MODE_ON && curTimeSyncMode == TIME_SYNC_MODE_ON )
    {
	curPacketType = PACKET_TYPE_ACCEL_TIME_SET;
    }
    else if ( curAccelMode == ACCEL_MODE_OFF
	    && curTimeSyncMode == TIME_SYNC_MODE_ON )
    {
	curPacketType = PACKET_TYPE_RAW_AUX_TIME_SET;
    }
    else if ( curAccelMode == ACCEL_MODE_OFF
	    && curTimeSyncMode == TIME_SYNC_MODE_OFF )
    {
	curPacketType = PACKET_TYPE_RAW_AUX;
    }
    else
    { // default accel on mode
      // curAccelMode == ACCEL_MODE_ON && curTimeSyncMode == TIME_SYNC_MODE_OFF
	curPacketType = PACKET_TYPE_ACCEL;
    }
}

uint8_t
highByte ( short s )
{
    return (uint8_t) ((s & 0xFF00) >> 8);
}

uint8_t
lowByte ( short s )
{
    return (uint8_t) (s & 0x00FF);
}

/**
 * @description Writes channel data, `auxData[0]` 2 bytes, and 4 byte unsigned
 *  time stamp in ms to serial port in the correct stream packet format.
 *
 *  If the global variable `sendTimeSyncUpPacket` is `true` (set by `processChar`
 *   getting a time sync set `<` command) then:
 *      Adds stop byte `OPENBCI_EOP_RAW_AUX_TIME_SET` and sets `sendTimeSyncUpPacket`
 *      to `false`.
 *  Else if `sendTimeSyncUpPacket` is `false` then:
 *      Adds stop byte `OPENBCI_EOP_RAW_AUX_TIME_SYNCED`
 */
void
sendTimeWithRawAuxSerial ( void )
{
    writeSerial (highByte (auxData[0])); // 2 bytes of aux data
    writeSerial (lowByte (auxData[0]));
    writeTimeCurrentSerial (lastSampleTime); // 4 bytes
}

void
writeAuxDataSerial ( void )
{
    for ( int i = 0; i < 3; i++ )
    {
	writeSerial ((uint8_t) highByte (auxData[i])); // write 16 bit axis data MSB first
	writeSerial ((uint8_t) lowByte (auxData[i])); // axisData is array of type short (16bit)
    }
}

void
zeroAuxData ( void )
{
    for ( int i = 0; i < 3; i++ )
    {
	auxData[i] = 0; // reset auxData bytes to 0
    }
}

void
writeTimeCurrent ( void )
{
    uint32_t newTime = xTaskGetTickCount (); // serialize the number, placing the MSB in lower packets
    for ( int j = 3; j >= 0; j-- )
    {
	write ((uint8_t) (newTime >> (j * 8)));
    }
}

void
writeTimeCurrentSerial ( uint32_t newTime )
{
    // serialize the number, placing the MSB in lower packets
    for ( int j = 3; j >= 0; j-- )
    {
	writeSerial ((uint8_t) (newTime >> (j * 8)));
    }
}

//SPI communication method
uint8_t
xfer ( uint8_t _data )
{
    uint8_t inByte;
    //inByte = spi.transfer(_data);

    HAL_SPI_TransmitReceive (&hspi3, &_data, &inByte, 1, 1000);

    return inByte;
}

//SPI chip select method
//void csLow(int SS)
//{ // select an SPI slave to talk to
//  switch (SS)
//  {
//  case BOARD_ADS:
////    spi.setMode(DSPI_MODE1);
////    spi.setSpeed(4000000);
////    digitalWrite(BOARD_ADS, LOW);
//    break;
////  case LIS3DH_SS:
////    spi.setMode(DSPI_MODE3);
//    spi.setSpeed(4000000);
//    digitalWrite(LIS3DH_SS, LOW);
//    break;
//  case SD_SS:
//    spi.setMode(DSPI_MODE0);
//    spi.setSpeed(20000000);
//    digitalWrite(SD_SS, LOW);
//    break;
//  case DAISY_ADS:
//    spi.setMode(DSPI_MODE1);
//    spi.setSpeed(4000000);
//    digitalWrite(DAISY_ADS, LOW);
//    break;
//  case BOTH_ADS:
//    spi.setMode(DSPI_MODE1);
//    spi.setSpeed(4000000);
//    digitalWrite(BOARD_ADS, LOW);
//    digitalWrite(DAISY_ADS, LOW);
//    break;
//  default:
//    break;
//  }
//}

//void csHigh(int SS)
//{ // deselect SPI slave
//  switch (SS)
//  {
//  case BOARD_ADS:
////    digitalWrite(BOARD_ADS, HIGH);
//    break;
//  case LIS3DH_SS:
//    digitalWrite(LIS3DH_SS, HIGH);
//    break;
//  case SD_SS:
//    digitalWrite(SD_SS, HIGH);
//    break;
//  case DAISY_ADS:
//    digitalWrite(DAISY_ADS, HIGH);
//    break;
//  case BOTH_ADS:
//    digitalWrite(BOARD_ADS, HIGH);
//    digitalWrite(DAISY_ADS, HIGH);
//    break;
//  default:
//    break;
//  }
////  spi.setSpeed(20000000);
////  spi.setMode(DSPI_MODE0); // DEFAULT TO SD MODE!
//}

// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  ADS1299 FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void
initialize_ads ()
{
    /* Power cycle ADS1299 */
    HAL_GPIO_WritePin (ADC_PWDN_GPIO_Port, ADC_PWDN_Pin, GPIO_PIN_RESET);

    osDelay (20);

    HAL_GPIO_WritePin (ADC_PWDN_GPIO_Port, ADC_PWDN_Pin, GPIO_PIN_SET);

    // recommended power up sequence requiers >Tpor (~32mS)
    osDelay (50);

    HAL_GPIO_WritePin (ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);

    osDelay (1);

    HAL_GPIO_WritePin (ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_SET);

//  osDelay(1);

//  pinMode(ADS_RST, OUTPUT);
//  digitalWrite(ADS_RST, LOW);  // reset pin connected to both ADS ICs
//  delayMicroseconds(4);        // toggle reset pin
//  digitalWrite(ADS_RST, HIGH); // this will reset the Daisy if it is present
//  delayMicroseconds(20);       // recommended to wait 18 Tclk before using device (~8uS);

    // initalize the  data ready chip select and reset pins:
//  pinMode(ADS_DRDY, INPUT); // we get DRDY asertion from the on-board ADS
    osDelay (40);

    resetADS (BOARD_ADS); // reset the on-board ADS registers, and stop DataContinuousMode
    osDelay (10);

//  WREG(CONFIG1, (ADS1299_CONFIG1_DAISY | curSampleRate), BOARD_ADS); // tell on-board ADS to output its clk, set the data rate to 250SPS
//  osDelay(40);
//
//  resetADS(DAISY_ADS); // software reset daisy module if present
//  osDelay(10);
//  daisyPresent = smellDaisy(); // check to see if daisy module is present
//    daisyPresent = false;
//    if ( !daisyPresent )
//    {
	WREG (CONFIG1, (ADS1299_CONFIG1_DAISY_NOT | curSampleRate), BOARD_ADS); // turn off clk output if no daisy present
	numChannels = 8;                          // expect up to 8 ADS channels
//    }
//  else
//  {
//    numChannels = 16;                                                      // expect up to 16 ADS channels
//    WREG(CONFIG1, (ADS1299_CONFIG1_DAISY_NOT | curSampleRate), DAISY_ADS); // tell on-board ADS to output its clk, set the data rate to 250SPS
//    osDelay(40);
//  }

    // DEFAULT CHANNEL SETTINGS FOR ADS
    defaultChannelSettings[POWER_DOWN] = NO;               // on = NO, off = YES
    defaultChannelSettings[GAIN_SET] = ADS_GAIN24;            // Gain setting
    defaultChannelSettings[INPUT_TYPE_SET] = ADSINPUT_NORMAL; // input muxer setting
    defaultChannelSettings[BIAS_SET] = YES; // add this channel to bias generation
    defaultChannelSettings[SRB2_SET] = YES;       // connect this P side to SRB2
    defaultChannelSettings[SRB1_SET] = NO;                    // don't use SRB1

    for ( int i = 0; i < numChannels; i++ )
    {
	for ( int j = 0; j < 6; j++ )
	{
	    channelSettings[i][j] = defaultChannelSettings[j]; // assign default settings
	}
	useInBias[i] = true; // keeping track of Bias Generation
	useSRB2[i] = true;   // keeping track of SRB2 inclusion
    }
//    boardUseSRB1 = daisyUseSRB1 = false;
    boardUseSRB1 = false;

    writeChannelSettings (); // write settings to the on-board and on-daisy ADS if present

    WREG (CONFIG3, 0b11101100, BOTH_ADS);
    osDelay (1); // enable internal reference drive and etc.
    for ( int i = 0; i < numChannels; i++ )
    { // turn off the impedance measure signal
	leadOffSettings[i][PCHAN] = OFF;
	leadOffSettings[i][NCHAN] = OFF;
    }
    verbosity = false; // when verbosity is true, there will be Serial feedback
    firstDataPacket = true;

    streaming = false;
}

//////////////////////////////////////////////
///////////// STREAM METHODS /////////////////
//////////////////////////////////////////////

/**
 * @description Used to activate a channel, if running must stop and start after...
 * @param channelNumber int the channel you want to change
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeChannelActivate ( uint8_t channelNumber )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    // Activate the channel
    activateChannel (channelNumber);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to deactivate a channel, if running must stop and start after...
 * @param channelNumber int the channel you want to change
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeChannelDeactivate ( uint8_t channelNumber )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    // deactivate the channel
    deactivateChannel (channelNumber);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to set lead off for a channel, if running must stop and start after...
 * @param `channelNumber` - [byte] - The channel you want to change
 * @param `pInput` - [byte] - Apply signal to P input, either ON (1) or OFF (0)
 * @param `nInput` - [byte] - Apply signal to N input, either ON (1) or OFF (0)
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeLeadOffSetForChannel ( uint8_t channelNumber, uint8_t pInput,
				 uint8_t nInput )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    changeChannelLeadOffDetect (channelNumber);

    // leadOffSetForChannel(channelNumber, pInput, nInput);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to set lead off for a channel, if running must stop and start after...
 * @param see `.channelSettingsSetForChannel()` for parameters
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeChannelSettingsForChannel ( uint8_t channelNumber, uint8_t powerDown,
				      uint8_t gain, uint8_t inputType,
				      uint8_t bias, uint8_t srb2, uint8_t srb1 )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    writeChannelSettingsN (channelNumber);

    // channelSettingsSetForChannel(channelNumber, powerDown, gain, inputType, bias, srb2, srb1);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to report (Serial0.print) the default channel settings
 *                  if running must stop and start after...
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeReportAllChannelDefaults ( void )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    reportDefaultChannelSettings ();

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to set all channels on Board (and Daisy) to the default
 *                  channel settings if running must stop and start after...
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeSetAllChannelsToDefault ( void )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    setChannelsToDefault ();

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * @description Used to set the sample rate
 * @param sr {SAMPLE_RATE} - The sample rate to set to.
 * @author AJ Keller (@pushtheworldllc)
 */
void
streamSafeSetSampleRate ( SAMPLE_RATE sr )
{
    uint8_t wasStreaming = streaming;

    // Stop streaming if you are currently streaming
    if ( streaming )
    {
	streamStop ();
    }

    setSampleRate (sr);

    // Restart stream if need be
    if ( wasStreaming )
    {
	streamStart ();
    }
}

/**
 * Return an array of gains in coded ADS form i.e. 0-6 where 6 is x24 and so on.
 * @return  [description]
 */

uint8_t*
getGains ( void )
{
    //uint8_t gains[numChannels];
    for ( uint8_t i = 0; i < 8; i++ )
    {
	gains[i] = channelSettings[i][GAIN_SET];
    }
    return gains;
}

/**
 * @description Call this to start the streaming data from the ADS1299
 * @returns uint8_t if able to start streaming
 */
void
streamStart ()
{ // needs daisy functionality

    streaming = true;
    startADS ();
    if ( curBoardMode == BOARD_MODE_DEBUG || curDebugMode == DEBUG_MODE_ON )
    {
	vSerialPutStringLn ("ADS Started");
    }
}

/**
 * @description Call this to stop streaming from the ADS1299
 * @returns uint8_t if able to stop streaming
 */
void
streamStop ()
{
    streaming = false;
    stopADS ();
    if ( curBoardMode == BOARD_MODE_DEBUG || curDebugMode == DEBUG_MODE_ON )
    {
	vSerialPutStringLn ("ADS Stopped");
    }
}

//reset all the ADS1299's settings. Stops all data acquisition
void
resetADS ( int targetSS )
{
    int startChan, stopChan;
    if ( targetSS == BOARD_ADS )
    {
	startChan = 1;
	stopChan = 8;
    }
    if ( targetSS == DAISY_ADS )
    {
	startChan = 9;
	stopChan = 16;
    }
    RRESET (targetSS);  // send RESET command to default all registers
    SDATAC (targetSS); // exit Read Data Continuous mode to communicate with ADS
    osDelay (100);
    // turn off all channels
    for ( int chan = startChan; chan <= stopChan; chan++ )
    {
	deactivateChannel (chan);
    }
}

void
setChannelsToDefault ( void )
{
    for ( int i = 0; i < numChannels; i++ )
    {
	for ( int j = 0; j < 6; j++ )
	{
	    channelSettings[i][j] = defaultChannelSettings[j];
	}
	useInBias[i] = true; // keeping track of Bias Generation
	useSRB2[i] = true;   // keeping track of SRB2 inclusion
    }
//    boardUseSRB1 = daisyUseSRB1 = false;
    boardUseSRB1 = false;

    writeChannelSettings (); // write settings to on-board ADS

    for ( int i = 0; i < numChannels; i++ )
    { // turn off the impedance measure signal
	leadOffSettings[i][PCHAN] = OFF;
	leadOffSettings[i][NCHAN] = OFF;
    }
    changeChannelLeadOffDetect (); // write settings to all ADS

    WREG (MISC1, 0x00, BOARD_ADS); // open SRB1 switch on-board
//    if ( daisyPresent )
//    {
//	WREG (MISC1, 0x00, DAISY_ADS);
//    } // open SRB1 switch on-daisy
}

// void setChannelsToDefault(void){
//
//     // Reset the global channel settings array to default
//     resetChannelSettingsArrayToDefault(channelSettings);
//     // Write channel settings to board (and daisy) ADS
//     channelSettingsArraySetForAll();
//
//     // Reset the global lead off settings array to default
//     resetLeadOffArrayToDefault(leadOffSettings);
//     // Write lead off settings to board (and daisy) ADS
//     leadOffSetForAllChannels();
//
//     WREG(MISC1,0x00,BOARD_ADS);  // open SRB1 switch on-board
// }

/**
 * @description Writes the default channel settings over the serial port
 */
void
reportDefaultChannelSettings ( void )
{
    char buf[7];
    buf[0] = getDefaultChannelSettingForSettingAscii (POWER_DOWN); // on = NO, off = YES
    buf[1] = getDefaultChannelSettingForSettingAscii (GAIN_SET); // Gain setting
    buf[2] = getDefaultChannelSettingForSettingAscii (INPUT_TYPE_SET); // input muxer setting
    buf[3] = getDefaultChannelSettingForSettingAscii (BIAS_SET); // add this channel to bias generation
    buf[4] = getDefaultChannelSettingForSettingAscii (SRB2_SET); // connect this P side to SRB2
    buf[5] = getDefaultChannelSettingForSettingAscii (SRB1_SET); // don't use SRB1
    printAllStr ((const char *) buf);
    sendEOT ();
}

/**
 * @description Set all channels using global channelSettings array
 * @author AJ Keller (@pushtheworldllc)
 */
// void channelSettingsArraySetForAll(void) {
//     uint8_t channelNumberUpperLimit;
//     // The upper limit of the channels, either 8 or 16
//     channelNumberUpperLimit = daisyPresent ? OPENBCI_NUMBER_OF_CHANNELS_DAISY : OPENBCI_NUMBER_OF_CHANNELS_DEFAULT;
//     // Loop through all channels
//     for (uint8_t i = 1; i <= channelNumberUpperLimit; i++) {
//         // Set for this channel
//         channelSettingsSetForChannel(i, channelSettings[i][POWER_DOWN], channelSettings[i][GAIN_SET], channelSettings[i][INPUT_TYPE_SET], channelSettings[i][BIAS_SET], channelSettings[i][SRB2_SET], channelSettings[i][SRB1_SET]);
//     }
// }

/**
 * @description Set channel using global channelSettings array for channelNumber
 * @param `channelNumber` - [uint8_t] - 1-16 channel number
 * @author AJ Keller (@pushtheworldllc)
 */
// void channelSettingsArraySetForChannel(byte channelNumber) {
//     // contstrain the channel number to 0-15
//     char index = getConstrainedChannelNumber(channelNumber);
//     // Set for this channel
//     channelSettingsSetForChannel(channelNumber, channelSettings[index][POWER_DOWN], channelSettings[index][GAIN_SET], channelSettings[index][INPUT_TYPE_SET], channelSettings[index][BIAS_SET], channelSettings[index][SRB2_SET], channelSettings[index][SRB1_SET]);
// }

/**
 * @description To add a usability abstraction layer above channel setting commands. Due to the
 *          extensive and highly specific nature of the channel setting command chain.
 * @param `channelNumber` - [uint8_t] (1-16) for index, so convert channel to array prior
 * @param `powerDown` - [uint8_t] - YES (1) or NO (0)
 *          Powers channel down
 * @param `gain` - [uint8_t] - Sets the gain for the channel
 *          ADS_GAIN01 (0b00000000)	// 0x00
 *          ADS_GAIN02 (0b00010000)	// 0x10
 *          ADS_GAIN04 (0b00100000)	// 0x20
 *          ADS_GAIN06 (0b00110000)	// 0x30
 *          ADS_GAIN08 (0b01000000)	// 0x40
 *          ADS_GAIN12 (0b01010000)	// 0x50
 *          ADS_GAIN24 (0b01100000)	// 0x60
 * @param `inputType` - [uint8_t] - Selects the ADC channel input source, either:
 *          ADSINPUT_NORMAL     (0b00000000)
 *          ADSINPUT_SHORTED    (0b00000001)
 *          ADSINPUT_BIAS_MEAS  (0b00000010)
 *          ADSINPUT_MVDD       (0b00000011)
 *          ADSINPUT_TEMP       (0b00000100)
 *          ADSINPUT_TESTSIG    (0b00000101)
 *          ADSINPUT_BIAS_DRP   (0b00000110)
 *          ADSINPUT_BIAL_DRN   (0b00000111)
 * @param `bias` - [byte] (YES (1) -> Include in bias (default), NO (0) -> remove from bias)
 *          selects to include the channel input in bias generation
 * @param `srb2` - [byte] (YES (1) -> Connect this input to SRB2 (default),
 *                     NO (0) -> Disconnect this input from SRB2)
 *          Select to connect (YES) this channel's P input to the SRB2 pin. This closes
 *              a switch between P input and SRB2 for the given channel, and allows the
 *              P input to also remain connected to the ADC.
 * @param `srb1` - [byte] (YES (1) -> connect all N inputs to SRB1,
 *                     NO (0) -> Disconnect all N inputs from SRB1 (default))
 *          Select to connect (YES) all channels' N inputs to SRB1. This effects all pins,
 *              and disconnects all N inputs from the ADC.
 * @author AJ Keller (@pushtheworldllc)
 */
// void channelSettingsSetForChannel(byte channelNumber, byte powerDown, byte gain, byte inputType, byte bias, byte srb2, byte srb1) {
//     byte setting, targetSS;
//     // contstrain the channel number to 0-15
//     char index = getConstrainedChannelNumber(channelNumber);
//     // Get the slave select pin for this channel
//     targetSS = getTargetSSForConstrainedChannelNumber(index);
//     if (sniffMode && Serial1) {
//         if (targetSS == BOARD_ADS) {
//             Serial1.print("Set channel "); Serial1.print(channelNumber); Serial1.println(" settings");
//         }
//     }
//     // first, disable any data collection
//     SDATAC(targetSS); osDelay(1);      // exit Read Data Continuous mode to communicate with ADS
//     setting = 0x00;
//     // Set the power down bit
//     if(powerDown == YES) {
//         setting |= 0x80;
//     }
//     // Set the gain bits
//     setting |= gain;
//     // Set input type bits
//     setting |= inputType;
//     if(srb2 == YES){
//         setting |= 0x08; // close this SRB2 switch
//         useSRB2[index] = true;  // keep track of SRB2 usage
//     }else{
//         useSRB2[index] = false;
//     }
//     byte channelNumberRegister = 0x00;
//     // Since we are addressing 8 bit registers, we need to subtract 8 from the
//     //  channelNumber if we are addressing the Daisy ADS
//     if (targetSS == DAISY_ADS) {
//         channelNumberRegister = index - OPENBCI_NUMBER_OF_CHANNELS_DEFAULT;
//     } else {
//         channelNumberRegister = index;
//     }
//     WREG(CH1SET+channelNumberRegister, setting, targetSS);  // write this channel's register settings
//     // add or remove from inclusion in BIAS generation
//     setting = RREG(BIAS_SENSP,targetSS);       //get the current P bias settings
//     if(bias == YES){
//         useInBias[index] = true;
//         bitSet(setting,channelNumberRegister);    //set this channel's bit to add it to the bias generation
//     }else{
//         useInBias[index] = false;
//         bitClear(setting,channelNumberRegister);  // clear this channel's bit to remove from bias generation
//     }
//     WREG(BIAS_SENSP,setting,targetSS); osDelay(1); //send the modified byte back to the ADS
//     setting = RREG(BIAS_SENSN,targetSS);       //get the current N bias settings
//     if(bias == YES){
//         bitSet(setting,channelNumberRegister);    //set this channel's bit to add it to the bias generation
//     }else{
//         bitClear(setting,channelNumberRegister);  // clear this channel's bit to remove from bias generation
//     }
//     WREG(BIAS_SENSN,setting,targetSS); osDelay(1); //send the modified byte back to the ADS
//     byte startChan = targetSS == BOARD_ADS ? 0 : OPENBCI_CHANNEL_MAX_NUMBER_8 - 1;
//     byte endChan = targetSS == BOARD_ADS ? OPENBCI_CHANNEL_MAX_NUMBER_8 : OPENBCI_CHANNEL_MAX_NUMBER_16 - 1;
//     // if SRB1 is closed or open for one channel, it will be the same for all channels
//     if(srb1 == YES){
//         for(int i=startChan; i<endChan; i++){
//             channelSettings[i][SRB1_SET] = YES;
//         }
//         if(targetSS == BOARD_ADS) boardUseSRB1 = true;
//         if(targetSS == DAISY_ADS) daisyUseSRB1 = true;
//         setting = 0x20;     // close SRB1 swtich
//     }
//     if(srb1 == NO){
//         for(int i=startChan; i<endChan; i++){
//             channelSettings[i][SRB1_SET] = NO;
//         }
//         if(targetSS == BOARD_ADS) boardUseSRB1 = false;
//         if(targetSS == DAISY_ADS) daisyUseSRB1 = false;
//         setting = 0x00;     // open SRB1 switch
//     }
//     WREG(MISC1,setting,targetSS);
// }


// write settings for ALL 8 channels for a given ADS board
// channel settings: powerDown, gain, inputType, SRB2, SRB1
void
writeChannelSettings ()
{
    uint8_t use_SRB1 = false;
    uint8_t setting, startChan = 0, endChan = 8;
    uint8_t targetSS = 0;

    SDATAC (targetSS);
    osDelay (1); // exit Read Data Continuous mode to communicate with ADS

    for ( uint8_t i = startChan; i < endChan; i++ )
    { // write 8 channel settings
	setting = 0x00;
	if ( channelSettings[i][POWER_DOWN] == YES )
	{
	    setting |= 0x80;
	}
	setting |= channelSettings[i][GAIN_SET];       // gain
	setting |= channelSettings[i][INPUT_TYPE_SET]; // input code
	if ( channelSettings[i][SRB2_SET] == YES )
	{
	    setting |= 0x08;   // close this SRB2 switch
	    useSRB2[i] = true; // remember SRB2 state for this channel
	}
	else
	{
	    useSRB2[i] = false; // remember SRB2 state for this channel
	}
	WREG (CH1SET + (i - startChan), setting, targetSS); // write this channel's register settings

	// add or remove this channel from inclusion in BIAS generation
	setting = RREG (BIAS_SENSP, targetSS); //get the current P bias settings
	if ( channelSettings[i][BIAS_SET] == YES )
	{
	    //bitSet(setting, i - startChan);
	    setting |= 1 << (i - startChan);
	    useInBias[i] = true; //add this channel to the bias generation
	}
	else
	{
	    //bitClear(setting, i - startChan);
	    setting &= ~(1 << (i - startChan));
	    useInBias[i] = false; //remove this channel from bias generation
	}
	WREG (BIAS_SENSP, setting, targetSS);
	osDelay (1); //send the modified byte back to the ADS

	setting = RREG (BIAS_SENSN, targetSS); //get the current N bias settings
	if ( channelSettings[i][BIAS_SET] == YES )
	{
	    //bitSet(setting, i - startChan); //set this channel's bit to add it to the bias generation
	    setting |= 1 << (i - startChan);
	}
	else
	{
	    //bitClear(setting, i - startChan); // clear this channel's bit to remove from bias generation
	    setting &= ~(1 << (i - startChan));
	}
	WREG (BIAS_SENSN, setting, targetSS);
	osDelay (1); //send the modified byte back to the ADS

	if ( channelSettings[i][SRB1_SET] == YES )
	{
	    use_SRB1 = true; // if any of the channel setting closes SRB1, it is closed for all
	}
    } // end of CHnSET and BIAS settings
//    }   // end of board select loop
    if ( use_SRB1 )
    {
	for ( int i = startChan; i < endChan; i++ )
	{
	    channelSettings[i][SRB1_SET] = YES;
	}
	WREG (MISC1, 0x20, targetSS); // close SRB1 swtich
	if ( targetSS == BOARD_ADS )
	{
	    boardUseSRB1 = true;
	}
    }
    else
    {
	for ( int i = startChan; i < endChan; i++ )
	{
	    channelSettings[i][SRB1_SET] = NO;
	}
	WREG (MISC1, 0x00, targetSS); // open SRB1 switch
	if ( targetSS == BOARD_ADS )
	{
	    boardUseSRB1 = false;
	}
    }
}

// write settings for a SPECIFIC channel on a given ADS board
void
writeChannelSettingsN ( uint8_t N )
{

    uint8_t setting, startChan, endChan, targetSS;
    if ( N < 9 )
    { // channels 1-8 on board
	targetSS = BOARD_ADS;
	startChan = 0;
	endChan = 8;
    }
    // function accepts channel 1-8, must be 0 indexed to work with array

    //N = constrain(N - 1, startChan, endChan - 1); //subtracts 1 so that we're counting from 0, not 1
    if ( (N - 1) > (endChan - 1) )
	N = endChan - 1;
    else if ( (N - 1) < startChan )
	N = startChan;
    else
	N = N - 1;

    // first, disable any data collection
    SDATAC (targetSS);
    osDelay (1); // exit Read Data Continuous mode to communicate with ADS

    setting = 0x00;
    if ( channelSettings[N][POWER_DOWN] == YES )
	setting |= 0x80;
    setting |= channelSettings[N][GAIN_SET];       // gain
    setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
    if ( channelSettings[N][SRB2_SET] == YES )
    {
	setting |= 0x08;   // close this SRB2 switch
	useSRB2[N] = true; // keep track of SRB2 usage
    }
    else
    {
	useSRB2[N] = false;
    }
    WREG (CH1SET + (N - startChan), setting, targetSS); // write this channel's register settings

    // add or remove from inclusion in BIAS generation
    setting = RREG (BIAS_SENSP, targetSS); //get the current P bias settings
    if ( channelSettings[N][BIAS_SET] == YES )
    {
	useInBias[N] = true;
	//bitSet(setting, N - startChan); //set this channel's bit to add it to the bias generation
	setting &= ~(1 << (N - startChan));
    }
    else
    {
	useInBias[N] = false;
	//bitClear(setting, N - startChan); // clear this channel's bit to remove from bias generation
	setting &= ~(1 << (N - startChan));
    }
    WREG (BIAS_SENSP, setting, targetSS);
    osDelay (1);                        //send the modified byte back to the ADS
    setting = RREG (BIAS_SENSN, targetSS); //get the current N bias settings
    if ( channelSettings[N][BIAS_SET] == YES )
    {
	//bitSet(setting, N - startChan); //set this channel's bit to add it to the bias generation
	setting |= 1 << (N - startChan);

    }
    else
    {
	//bitClear(setting, N - startChan); // clear this channel's bit to remove from bias generation
	setting &= ~(1 << (N - startChan));
    }
    WREG (BIAS_SENSN, setting, targetSS);
    osDelay (1); //send the modified byte back to the ADS

    // if SRB1 is closed or open for one channel, it will be the same for all channels
    if ( channelSettings[N][SRB1_SET] == YES )
    {
	for ( int i = startChan; i < endChan; i++ )
	{
	    channelSettings[i][SRB1_SET] = YES;
	}
	if ( targetSS == BOARD_ADS )
	    boardUseSRB1 = true;
//	if ( targetSS == DAISY_ADS )
//	    daisyUseSRB1 = true;
	setting = 0x20; // close SRB1 swtich
    }
    if ( channelSettings[N][SRB1_SET] == NO )
    {
	for ( int i = startChan; i < endChan; i++ )
	{
	    channelSettings[i][SRB1_SET] = NO;
	}
	if ( targetSS == BOARD_ADS )
	    boardUseSRB1 = false;
//	if ( targetSS == DAISY_ADS )
//	    daisyUseSRB1 = false;
	setting = 0x00; // open SRB1 switch
    }
    WREG (MISC1, setting, targetSS);
}

//  deactivate the given channel.
void
deactivateChannel ( uint8_t N )
{
    uint8_t setting, startChan, endChan, targetSS;
    if ( N < 9 )
    {
	targetSS = BOARD_ADS;
	startChan = 0;
	endChan = 8;
    }

    SDATAC (targetSS);
    osDelay (1);       // exit Read Data Continuous mode to communicate with ADS
    //N = constrain(N - 1, startChan, endChan - 1); //subtracts 1 so that we're counting from 0, not 1
    if ( (N - 1) > (endChan - 1) )
	N = endChan - 1;
    else if ( (N - 1) < startChan )
	N = startChan;
    else
	N = N - 1;

    setting = RREG (CH1SET + (N - startChan), targetSS);
    osDelay (1);             // get the current channel settings
    //bitSet(setting, 7);   // set bit7 to shut down channel
    //bitClear(setting, 3); // clear bit3 to disclude from SRB2 if used

    setting |= (1 << 7);
    setting &= ~(1 << 3);

    WREG (CH1SET + (N - startChan), setting, targetSS);
    osDelay (1); // write the new value to disable the channel

    //remove the channel from the bias generation...
    setting = RREG (BIAS_SENSP, targetSS);
    osDelay (1);                         //get the current bias settings
//  bitClear(setting, N - startChan); //clear this channel's bit to remove from bias generation
    setting &= ~(1 << (N - startChan));

    WREG (BIAS_SENSP, setting, targetSS);
    osDelay (1); //send the modified byte back to the ADS

    setting = RREG (BIAS_SENSN, targetSS);
    osDelay (1);                         //get the current bias settings
    //bitClear(setting, N - startChan); //clear this channel's bit to remove from bias generation
    setting &= ~(1 << (N - startChan));

    WREG (BIAS_SENSN, setting, targetSS);
    osDelay (1); //send the modified byte back to the ADS

    leadOffSettings[N][0] = leadOffSettings[N][1] = NO;
    changeChannelLeadOffDetect (N + 1);
}

void
activateChannel ( uint8_t N )
{
    uint8_t setting, startChan, endChan, targetSS;
    if ( N < 9 )
    {
	targetSS = BOARD_ADS;
	startChan = 0;
	endChan = 8;
    }

    //N = constrain(N - 1, startChan, endChan - 1); // 0-7 or 8-15
    if ( (N - 1) > (endChan - 1) )
	N = endChan - 1;
    else if ( (N - 1) < startChan )
	N = startChan;
    else
	N = N - 1;

    SDATAC (targetSS); // exit Read Data Continuous mode to communicate with ADS
    setting = 0x00;
    //  channelSettings[N][POWER_DOWN] = NO; // keep track of channel on/off in this array  REMOVE?
    setting |= channelSettings[N][GAIN_SET];       // gain
    setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
    if ( useSRB2[N] == true )
    {
	channelSettings[N][SRB2_SET] = YES;
    }
    else
    {
	channelSettings[N][SRB2_SET] = NO;
    }
    if ( channelSettings[N][SRB2_SET] == YES )
    {
//    bitSet(setting, 3);
	setting |= (1 << 3);
    } // close this SRB2 switch
    WREG (CH1SET + (N - startChan), setting, targetSS);
    // add or remove from inclusion in BIAS generation
    if ( useInBias[N] )
    {
	channelSettings[N][BIAS_SET] = YES;
    }
    else
    {
	channelSettings[N][BIAS_SET] = NO;
    }
    setting = RREG (BIAS_SENSP, targetSS); //get the current P bias settings
    if ( channelSettings[N][BIAS_SET] == YES )
    {
//    bitSet(setting, N - startChan); //set this channel's bit to add it to the bias generation
	setting |= 1 << (N - startChan);
	useInBias[N] = true;
    }
    else
    {
	//bitClear(setting, N - startChan); // clear this channel's bit to remove from bias generation
	setting &= ~(1 << (N - startChan));

	useInBias[N] = false;
    }
    WREG (BIAS_SENSP, setting, targetSS);
    osDelay (1);                        //send the modified byte back to the ADS
    setting = RREG (BIAS_SENSN, targetSS); //get the current N bias settings
    if ( channelSettings[N][BIAS_SET] == YES )
    {
//    bitSet(setting, N - startChan); //set this channel's bit to add it to the bias generation
	setting |= 1 << (N - startChan);
    }
    else
    {
	//bitClear(setting, N - startChan); // clear this channel's bit to remove from bias generation
	setting &= ~(1 << (N - startChan));

    }
    WREG (BIAS_SENSN, setting, targetSS);
    osDelay (1); //send the modified byte back to the ADS

    setting = 0x00;
    if ( targetSS == BOARD_ADS && boardUseSRB1 == true )
	setting = 0x20;
//    if ( targetSS == DAISY_ADS && daisyUseSRB1 == true )
//	setting = 0x20;
    WREG (MISC1, setting, targetSS); // close all SRB1 swtiches
}

// change the lead off detect settings for all channels
void
changeChannelLeadOffDetect ()
{
    uint8_t startChan = 0, endChan = 8, targetSS = 0;

    for ( int b = 0; b < 2; b++ )
    {
	if ( b == 0 )
	{
	    targetSS = BOARD_ADS;
	    startChan = 0;
	    endChan = 8;
	}
//	if ( b == 1 )
//	{
//	    if ( !daisyPresent )
//	    {
//		return;
//	    }
//	    targetSS = DAISY_ADS;
//	    startChan = 8;
//	    endChan = 16;
//	}

	SDATAC (targetSS);
	osDelay (1); // exit Read Data Continuous mode to communicate with ADS
	uint8_t P_setting = RREG (LOFF_SENSP, targetSS);
	uint8_t N_setting = RREG (LOFF_SENSN, targetSS);

	for ( int i = startChan; i < endChan; i++ )
	{
	    if ( leadOffSettings[i][PCHAN] == ON )
	    {
		//bitSet(P_setting, i - startChan);
		P_setting |= 1 << (i - startChan);
	    }
	    else
	    {
		//bitClear(P_setting, i - startChan);
		P_setting &= ~(1 << (i - startChan));

	    }
	    if ( leadOffSettings[i][NCHAN] == ON )
	    {
		//bitSet(N_setting, i - startChan);
		N_setting |= 1 << (i - startChan);

	    }
	    else
	    {
//        bitClear(N_setting, i - startChan);
		N_setting &= ~(1 << (i - startChan));

	    }
	    WREG (LOFF_SENSP, P_setting, targetSS);
	    WREG (LOFF_SENSN, N_setting, targetSS);
	}
    }
}

// change the lead off detect settings for specified channel
void
changeChannelLeadOffDetectN ( uint8_t N )
{
    uint8_t targetSS, startChan, endChan;

    if ( N < 9 )
    {
	targetSS = BOARD_ADS;
	startChan = 0;
	endChan = 8;
    }
//    else
//    {
//	if ( !daisyPresent )
//	{
//	    return;
//	}
//	targetSS = DAISY_ADS;
//	startChan = 8;
//	endChan = 16;
//    }

    //N = constrain(N - 1, startChan, endChan - 1);
    if ( (N - 1) > (endChan - 1) )
	N = endChan - 1;
    else if ( (N - 1) < startChan )
	N = startChan;
    else
	N = N - 1;

    SDATAC (targetSS);
    osDelay (1); // exit Read Data Continuous mode to communicate with ADS
    uint8_t P_setting = RREG (LOFF_SENSP, targetSS);
    uint8_t N_setting = RREG (LOFF_SENSN, targetSS);

    if ( leadOffSettings[N][PCHAN] == ON )
    {
	P_setting |= 1 << (N - startChan);
    }
    else
    {
	P_setting &= ~(1 << (N - startChan));
    }
    if ( leadOffSettings[N][NCHAN] == ON )
    {
	N_setting |= 1 << (N - startChan);
    }
    else
    {
	N_setting &= ~(1 << (N - startChan));
    }
    WREG (LOFF_SENSP, P_setting, targetSS);
    WREG (LOFF_SENSN, N_setting, targetSS);
}

void
configureLeadOffDetection ( uint8_t amplitudeCode, uint8_t freqCode )
{
    amplitudeCode &= 0b00001100; //only these two bits should be used
    freqCode &= 0b00000011;      //only these two bits should be used

    uint8_t setting, targetSS = BOARD_ADS;
    setting = RREG (LOFF, targetSS); //get the current bias settings
    //reconfigure the byte to get what we want
    setting &= 0b11110000;    //clear out the last four bits
    setting |= amplitudeCode; //set the amplitude
    setting |= freqCode;      //set the frequency
    //send the config byte back to the hardware
    WREG (LOFF, setting, targetSS);
    osDelay (1); //send the modified byte back to the ADS
}

// //  deactivate the given channel.
// void deactivateChannel(byte N)
// {
//     byte setting, startChan, endChan, targetSS;
//     if(N < 9){
//         targetSS = BOARD_ADS; startChan = 0; endChan = 8;
//     }else{
//         if(!daisyPresent) { return; }
//         targetSS = DAISY_ADS; startChan = 8; endChan = 16;
//     }
//     SDATAC(targetSS); osDelay(1);      // exit Read Data Continuous mode to communicate with ADS
//     N = constrain(N-1,startChan,endChan-1);  //subtracts 1 so that we're counting from 0, not 1
//
//     setting = RREG(CH1SET+(N-startChan),targetSS); osDelay(1); // get the current channel settings
//     bitSet(setting,7);     // set bit7 to shut down channel
//     bitClear(setting,3);   // clear bit3 to disclude from SRB2 if used
//     WREG(CH1SET+(N-startChan),setting,targetSS); osDelay(1);     // write the new value to disable the channel
//
//     //remove the channel from the bias generation...
//     setting = RREG(BIAS_SENSP,targetSS); osDelay(1); //get the current bias settings
//     bitClear(setting,N-startChan);                  //clear this channel's bit to remove from bias generation
//     WREG(BIAS_SENSP,setting,targetSS); osDelay(1);   //send the modified byte back to the ADS
//
//     setting = RREG(BIAS_SENSN,targetSS); osDelay(1); //get the current bias settings
//     bitClear(setting,N-startChan);                  //clear this channel's bit to remove from bias generation
//     WREG(BIAS_SENSN,setting,targetSS); osDelay(1);   //send the modified byte back to the ADS
//
//     leadOffSettings[N][PCHAN] = leadOffSettings[N][NCHAN] = NO;
//     leadOffSetForChannel(N+1, NO, NO);
// }

// void activateChannel(byte N)
// {
//     byte setting, startChan, endChan, targetSS;
//     if(N < 9){
//         targetSS = BOARD_ADS; startChan = 0; endChan = 8;
//     }else{
//         if(!daisyPresent) { return; }
//         targetSS = DAISY_ADS; startChan = 8; endChan = 16;
//     }

//     N = constrain(N-1,startChan,endChan-1);  // 0-7 or 8-15

//     SDATAC(targetSS);  // exit Read Data Continuous mode to communicate with ADS
//     setting = 0x00;
//     //  channelSettings[N][POWER_DOWN] = NO; // keep track of channel on/off in this array  REMOVE?
//     setting |= channelSettings[N][GAIN_SET]; // gain
//     setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
//     if(useSRB2[N] == true){channelSettings[N][SRB2_SET] = YES;}else{channelSettings[N][SRB2_SET] = NO;}
//     if(channelSettings[N][SRB2_SET] == YES) {bitSet(setting,3);} // close this SRB2 switch
//     WREG(CH1SET+(N-startChan),setting,targetSS);
//     // add or remove from inclusion in BIAS generation
//     if(useInBias[N]){channelSettings[N][BIAS_SET] = YES;}else{channelSettings[N][BIAS_SET] = NO;}
//     setting = RREG(BIAS_SENSP,targetSS);       //get the current P bias settings
//     if(channelSettings[N][BIAS_SET] == YES){
//         bitSet(setting,N-startChan);    //set this channel's bit to add it to the bias generation
//         useInBias[N] = true;
//     }else{
//         bitClear(setting,N-startChan);  // clear this channel's bit to remove from bias generation
//         useInBias[N] = false;
//     }
//     WREG(BIAS_SENSP,setting,targetSS); osDelay(1); //send the modified byte back to the ADS
//     setting = RREG(BIAS_SENSN,targetSS);       //get the current N bias settings
//     if(channelSettings[N][BIAS_SET] == YES){
//         bitSet(setting,N-startChan);    //set this channel's bit to add it to the bias generation
//     }else{
//         bitClear(setting,N-startChan);  // clear this channel's bit to remove from bias generation
//     }
//     WREG(BIAS_SENSN,setting,targetSS); osDelay(1); //send the modified byte back to the ADS

//     setting = 0x00;
//     if(targetSS == BOARD_ADS && boardUseSRB1 == true) setting = 0x20;
//     if(targetSS == DAISY_ADS && daisyUseSRB1 == true) setting = 0x20;
//     WREG(MISC1,setting,targetSS);     // close all SRB1 swtiches
// }

//////////////////////////////////////////////
///////////// LEAD OFF METHODS ///////////////
//////////////////////////////////////////////

/**
 * @description Runs through the `leadOffSettings` global array to set/change
 *                  the lead off signals for all inputs of all channels.
 * @author AJ Keller (@pushtheworldllc)
 */
// void leadOffSetForAllChannels(void) {
//     byte channelNumberUpperLimit;
//     // The upper limit of the channels, either 8 or 16
//     channelNumberUpperLimit = daisyPresent ? OPENBCI_NUMBER_OF_CHANNELS_DAISY : OPENBCI_NUMBER_OF_CHANNELS_DEFAULT;
//     // Loop through all channels
//     for (int i = 1; i <= channelNumberUpperLimit; i++) {
//         leadOffSetForChannel((uint8_t)i,leadOffSettings[i-1][PCHAN],leadOffSettings[i-1][NCHAN]);
//     }
// }

/**
 * @description Used to set lead off for a channel
 * @param `channelNumber` - [byte] - The channel you want to change
 * @param `pInput` - [byte] - Apply signal to P input, either ON (1) or OFF (0)
 * @param `nInput` - [byte] - Apply signal to N input, either ON (1) or OFF (0)
 * @author AJ Keller (@pushtheworldllc)
 */
// void leadOffSetForChannel(byte channelNumber, byte pInput, byte nInput) {
//     // contstrain the channel number to 0-15
//     channelNumber = getConstrainedChannelNumber(channelNumber);
//     // Get the slave select pin for this channel
//     byte targetSS = getTargetSSForConstrainedChannelNumber(channelNumber);
//     // exit Read Data Continuous mode to communicate with ADS
//     SDATAC(targetSS);
//     osDelay(1);
//     // Read P register
//     byte P_setting = RREG(LOFF_SENSP,targetSS);
//     // Read N register
//     byte N_setting = RREG(LOFF_SENSN,targetSS);
//     // Since we are addressing 8 bit registers, we need to subtract 8 from the
//     //  channelNumber if we are addressing the Daisy ADS
//     if (targetSS == DAISY_ADS) {
//         channelNumber -= OPENBCI_NUMBER_OF_CHANNELS_DEFAULT;
//     }
//     // If pInput is ON then we want to set, otherwise we want to clear
//     if (pInput == ON) {
//         bitSet(P_setting, channelNumber);
//     } else {
//         bitClear(P_setting, channelNumber);
//     }
//     // Write to the P register
//     WREG(LOFF_SENSP,P_setting,targetSS);
//     // If nInput is ON then we want to set, otherwise we want to clear
//     if (nInput == ON) {
//         bitSet(N_setting, channelNumber);
//     } else {
//         bitClear(N_setting, channelNumber);
//     }
//     // Write to the N register
//     WREG(LOFF_SENSN,N_setting,targetSS);
// }

/**
 * @description This sets the LOFF register on the Board ADS and the Daisy ADS
 * @param `amplitudeCode` - [byte] - The amplitude of the of impedance signal.
 *                 See `.setleadOffForSS()` for complete description
 * @param `freqCode` - [byte] - The frequency of the impedance signal can be either.
 *                 See `.setleadOffForSS()` for complete description
 * @author AJ Keller (@pushtheworldllc)
 */
// void leadOffConfigureSignalForAll(byte amplitudeCode, byte freqCode)
// {
//     // Set the lead off detection for the on board ADS
//     leadOffConfigureSignalForTargetSS(BOARD_ADS, amplitudeCode, freqCode);
//     // if the daisy board is present, set that register as well
//     if (daisyPresent) {
//         leadOffConfigureSignalForTargetSS(DAISY_ADS, amplitudeCode, freqCode);
//     }
// }
/**
 * @description This sets the LOFF (lead off) register for the given ADS with slave
 *                  select
 * @param `targetSS` - [byte] - The Slave Select pin.
 * @param `amplitudeCode` - [byte] - The amplitude of the of impedance signal.
 *          LOFF_MAG_6NA        (0b00000000)
 *          LOFF_MAG_24NA       (0b00000100)
 *          LOFF_MAG_6UA        (0b00001000)
 *          LOFF_MAG_24UA       (0b00001100)
 * @param `freqCode` - [byte] - The frequency of the impedance signal can be either.
 *          LOFF_FREQ_DC        (0b00000000)
 *          LOFF_FREQ_7p8HZ     (0b00000001)
 *          LOFF_FREQ_31p2HZ    (0b00000010)
 *          LOFF_FREQ_FS_4      (0b00000011)
 * @author Joel/Leif/Conor (@OpenBCI) Summer 2014
 */
// void leadOffConfigureSignalForTargetSS(byte targetSS, byte amplitudeCode, byte freqCode) {
//     byte setting;
//     amplitudeCode &= 0b00001100;  //only these two bits should be used
//     freqCode &= 0b00000011;  //only these two bits should be used
//     setting = RREG(LOFF,targetSS); //get the current bias settings
//     //reconfigure the byte to get what we want
//     setting &= 0b11110000;  //clear out the last four bits
//     setting |= amplitudeCode;  //set the amplitude
//     setting |= freqCode;    //set the frequency
//     //send the config byte back to the hardware
//     WREG(LOFF,setting,targetSS); osDelay(1);  //send the modified byte back to the ADS
// }

//Configure the test signals that can be inernally generated by the ADS1299
void
configureInternalTestSignal ( uint8_t amplitudeCode, uint8_t freqCode )
{
    uint8_t targetSS = BOARD_ADS;
//    for ( int i = 0; i < 2; i++ )
//    {
//	if ( i == 0 )
//	{
//	    targetSS = BOARD_ADS;
//	}
//	if ( i == 1 )
//	{
//	    if ( daisyPresent == false )
//	    {
//		return;
//	    }
//	    targetSS = DAISY_ADS;
//	}
	if ( amplitudeCode == ADSTESTSIG_NOCHANGE )
	    amplitudeCode = (RREG (CONFIG2, targetSS) & (0b00000100));
	if ( freqCode == ADSTESTSIG_NOCHANGE )
	    freqCode = (RREG (CONFIG2, targetSS) & (0b00000011));
	freqCode &= 0b00000011;                //only the last two bits are used
	amplitudeCode &= 0b00000100;                     //only this bit is used
	uint8_t setting = 0b11010000 | freqCode | amplitudeCode; //compose the code
	WREG (CONFIG2, setting, targetSS);
	osDelay (1);
	if ( curBoardMode == BOARD_MODE_DEBUG || curDebugMode == DEBUG_MODE_ON )
	{
	    vSerialPutString ("Wrote to CONFIG2: ");
	    char str[16] =
		{ 0 };
	    vSerialPutString (itoa (setting, str, 2));
//      Serial1.print(setting, BIN);
	}
//    }
}

void
changeInputType ( uint8_t inputCode )
{

    for ( int i = 0; i < numChannels; i++ )
    {
	channelSettings[i][INPUT_TYPE_SET] = inputCode;
    }

    // OLD CODE REVERT
    //channelSettingsArraySetForAll();

    writeChannelSettings ();
}

// Start continuous data acquisition
void
startADS ( void ) // NEEDS ADS ADDRESS, OR BOTH?
{
    sampleCounter = 0;
    sampleCounterBLE = 0;
    firstDataPacket = true;
    RDATAC (BOTH_ADS); // enter Read Data Continuous mode
    osDelay (1);
    START (BOTH_ADS); // start the data acquisition
    osDelay (1);
    isRunning = true;
}

/**
 * @description Check status register to see if data is available from the ADS1299.
 * @returns {uint8_t} - `true` if data is available
 */
uint8_t
isADSDataAvailable ( void )
{
    //return (!(digitalRead(ADS_DRDY)));
    return !HAL_GPIO_ReadPin (ADC_DRDY_GPIO_Port, ADC_DRDY_Pin);
}

// CALLED WHEN DRDY PIN IS ASSERTED. NEW ADS DATA AVAILABLE!
void
updateChannelData ( void )
{
    // this needs to be reset, or else it will constantly flag us
    channelDataAvailable = false;

    lastSampleTime = xTaskGetTickCount ();

    uint8_t downsample = false;
//  if (iSerial0.tx == false && iSerial1.baudRate > OPENBCI_BAUD_RATE_MIN_NO_AVG)
//  {
//    downsample = false;
//  }

    updateBoardDataDownSample (downsample);
//  if (daisyPresent)
//  {
//      updateDaisyDataDownSample(downsample);
//  }

    switch ( curBoardMode )
	{
	case BOARD_MODE_ANALOG:
//    auxData[0] = analogRead(A5);
//    auxData[1] = analogRead(A6);
	    break;
	case BOARD_MODE_DIGITAL:
//    auxData[0] = digitalRead(11) << 8 | digitalRead(12);
//    auxData[1] = (wifi.present ? 0 : digitalRead(13) << 8) | digitalRead(17);
//    auxData[2] = wifi.present ? 0 : digitalRead(18);
	    break;
	case BOARD_MODE_MARKER:
	    if ( newMarkerReceived )
	    {
		auxData[0] = (short) markerValue;
		newMarkerReceived = false;
	    }
	    break;
	case BOARD_MODE_BLE:
	case BOARD_MODE_DEBUG:
	case BOARD_MODE_DEFAULT:
	case BOARD_MODE_END_OF_MODES:
	    break;
	}
}

void
updateBoardData ( void )
{
    updateBoardDataDownSample (true);
}

void
updateBoardDataDownSample ( uint8_t downsample )
{
//  uint8_t inByte;
    int byteCounter = 0;

    uint8_t rx[27] = { 0 };

    HAL_SPI_Receive (&hspi3, rx, 27, 1000);

    boardStat = (rx[2] << 16) | (rx[1] << 8) | rx[0];

    for ( int i = 0; i < OPENBCI_ADS_CHANS_PER_BOARD; i++ )
    {

	boardChannelDataRaw[byteCounter++] = rx[i * 3 + 3];
	boardChannelDataRaw[byteCounter++] = rx[i * 3 + 4];
	boardChannelDataRaw[byteCounter++] = rx[i * 3 + 5];

	boardChannelDataInt[i] =  rx[i * 3 + 3];
	boardChannelDataInt[i] <<= 8;
	boardChannelDataInt[i] |= rx[i * 3 + 4];
	boardChannelDataInt[i] <<= 8;
	boardChannelDataInt[i] |= rx[i * 3 + 5];

    }

    // need to convert 24bit to 32bit if using the filter
    for ( int i = 0; i < OPENBCI_ADS_CHANS_PER_BOARD; i++ )
    { // convert 3 byte 2's compliment to 4 byte 2's compliment
	if ( (boardChannelDataInt[i] & (1 << 23)) != 0 )
	{
	    boardChannelDataInt[i] |= 0xFF000000;
	}
	else
	{
	    boardChannelDataInt[i] &= 0x00FFFFFF;
	}
    }

    if ( firstDataPacket == true )
    {
	firstDataPacket = false;
    }
}

// Stop the continuous data acquisition
void
stopADS ()
{
    STOP (BOTH_ADS); // stop the data acquisition
    osDelay (1);
    SDATAC (BOTH_ADS); // stop Read Data Continuous mode to communicate with ADS
    osDelay (1);
    isRunning = false;
}

void
printSerialInt ( int i )
{
    char str[20] =
	{ 0 };

    vSerialPutString (itoa (i, str, 10));

    usb_send_str (str);
}

void
printSerialChar ( char c )
{
    usb_send_char (c);
}

void
printSerialIntInt ( int c, int arg )
{
    char str[20] =
	{ 0 };

    itoa (c, str, arg);

    usb_send_str (str);
}

void
printSerialStr ( const char *c )
{
    if ( c != NULL )
    {
	for ( int i = 0; i < strlen (c); i++ )
	{
	    printSerialChar (c[i]);
	}
    }
}

void
printlnSerial ( void )
{
    //printSerialChar ('\r');
    printSerialChar ('\n');
}

void
printlnSerialChar ( char c )
{
    printSerialChar (c);
    printlnSerial ();
}

void
printlnSerialInt ( int c )
{
    printSerialInt (c);
    printlnSerial ();
}

void
printlnSerialIntInt ( int c, int arg )
{
    printSerialIntInt (c, arg);
    printlnSerial ();
}

void
printlnSerialStr ( const char *c )
{
    printSerialStr (c);
    printlnSerial ();
}

void
write ( uint8_t b )
{
    writeSerial (b);
}

void
writeSerial ( uint8_t c )
{
//    vSerialPutChar (c);
    usb_send_char (c);
}

void
ADS_writeChannelData ()
{
    //ADS_writeChannelDataAvgDaisy ();
    ADS_writeChannelDataNoAvgDaisy ();
}

void
ADS_writeChannelDataAvgDaisy ()
{
//  if (iSerial0.tx || (iSerial1.tx && iSerial1.baudRate <= OPENBCI_BAUD_RATE_MIN_NO_AVG))
//  {
//    if (daisyPresent)
//    {
//      // Code that runs with daisy present
//      if (sampleCounter % 2 != 0)
//      { //CHECK SAMPLE ODD-EVEN AND SEND THE APPROPRIATE ADS DATA
//        for (int i = 0; i < OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE; i++)
//        {
//          writeSerial(meanBoardDataRaw[i]);
//        }
//      }
//      else
//      {
//        for (int i = 0; i < OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE; i++)
//        {
//          writeSerial(meanDaisyDataRaw[i]);
//        }
//      }
//      // Code that runs without the daisy present
//    }
//    else
//    {
    for ( int i = 0; i < 24; i++ )
    {
	writeSerial (boardChannelDataRaw[i]);
    }
//    }
//  }
}

void
ADS_writeChannelDataNoAvgDaisy ()
{
//  if (iSerial1.tx && iSerial1.baudRate > OPENBCI_BAUD_RATE_MIN_NO_AVG)
//  {
    // Don't run this function if the serial baud rate is not greater then the
    // minimum
    // Always write board ADS data
    for ( int i = 0; i < OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE; i++ )
    {
	writeSerial (boardChannelDataRaw[i]);
    }

    // Only write daisy data if present
//    if (daisyPresent)
//    {
//      for (int i = 0; i < OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE; i++)
//      {
//        writeSerial(daisyChannelDataRaw[i]);
//      }
//    }
//  }
}

//print out the state of all the control registers
void
printADSregisters ( int targetSS )
{
    uint8_t prevverbosityState = verbosity;
    verbosity = true;                   // set up for verbosity output
    RREGS (0x00, 0x0C, targetSS);        // read out the first registers
    osDelay (10);               // stall to let all that data get read by the PC
    RREGS (0x0D, 0x17 - 0x0D, targetSS); // read out the rest
    verbosity = prevverbosityState;
}

uint8_t
ADS_getDeviceID ( int targetSS )
{ // simple hello world com check
    uint8_t data = RREG (ID_REG, targetSS);
    if ( verbosity )
    { // verbosity otuput
	printAllStr ("On Board ADS ID ");
	printHex (data);
	printlnAll ();
	sendEOT ();
    }
    return data;
}

//System Commands
void
WAKEUP ( int targetSS )
{
//  csLow(targetSS);
    xfer (_WAKEUP);
//  csHigh(targetSS);
    osDelay (1); //must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
}

void
STANDBY ( int targetSS )
{ // only allowed to send WAKEUP after sending STANDBY
//  csLow(targetSS);
    xfer (_STANDBY);
//  csHigh(targetSS);
}

void
RRESET ( int targetSS )
{ // reset all the registers to default settings
//  csLow(targetSS);
    xfer (_RESET);
    osDelay (1); //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
//  csHigh(targetSS);
}

void
START ( int targetSS )
{ //start data conversion
//  csLow(targetSS);
    xfer (_START); // KEEP ON-BOARD AND ON-DAISY IN SYNC
//  csHigh(targetSS);
}

void
STOP ( int targetSS )
{ //stop data conversion
//  csLow(targetSS);
    xfer (_STOP); // KEEP ON-BOARD AND ON-DAISY IN SYNC
//  csHigh(targetSS);
}

void
RDATAC ( int targetSS )
{
//  csLow(targetSS);
    xfer (_RDATAC); // read data continuous
//  csHigh(targetSS);

    osDelay (1);
//  delayMicroseconds(3);
}
void
SDATAC ( int targetSS )
{
//  csLow(targetSS);
    xfer (_SDATAC);
//  csHigh(targetSS);
    osDelay (1);
//  delayMicroseconds(10); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}

//  THIS NEEDS CLEANING AND UPDATING TO THE NEW FORMAT
void
RDATA ( int targetSS )
{
    uint8_t rx[27] =
	{ 0 };

    xfer (_RDATA);

    HAL_SPI_Receive (&hspi3, rx, 27, 1000);

//    boardStat = (boardStat << 8) | inByte; //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
    boardStat = (rx[2] << 16) | (rx[1] << 8) | rx[0];

    for ( int i = 0; i < 8; i++ )
    {
	boardChannelDataInt[i] = rx[i * 3 + 3] | (rx[i * 3 + 4] << 8)
		| (rx[i * 3 + 5] << 16);
    }

//  use in Stop Read Continuous mode when DRDY goes low
//  uint8_t inByte;     //  to read in one sample of the channels
//  csLow(targetSS); //  open SPI
//  xfer(_RDATA);    //  send the RDATA command
//  for (int i = 0; i < 3; i++)
//  { //  read in the status register and new channel data
//    inByte = xfer(0x00);
//    boardStat = (boardStat << 8) | inByte; //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
//  }
//  if (targetSS == BOARD_ADS)
//  {
//    for (int i = 0; i < 8; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      { //  read in the new channel data
//        inByte = xfer(0x00);
//        boardChannelDataInt[i] = (boardChannelDataInt[i] << 8) | inByte;
//      }
//    }

    for ( int i = 0; i < 8; i++ )
    {
//      if (bitRead(boardChannelDataInt[i], 23) == 1)
	if ( (boardChannelDataInt[i] & (1 << 23)) != 0 )
	{ // convert 3 byte 2's compliment to 4 byte 2's compliment
	    boardChannelDataInt[i] |= 0xFF000000;
	}
	else
	{
	    boardChannelDataInt[i] &= 0x00FFFFFF;
	}
    }
//  }
//  csHigh(targetSS); //  close SPI
}

uint8_t
RREG ( uint8_t _address, int targetSS )
{                                 //  reads ONE register at _address
    uint8_t tx[3] = { 0 };
    uint8_t rx[3] = { 0 };

    tx[0] = _address + 0x20; //  opcode1 RREG expects 001rrrrr where rrrrr = _address
    tx[1] = 0; 			//  opcode2
    tx[2] = 0;

    HAL_SPI_TransmitReceive (&hspi3, tx, rx, 3, 1000);

    regData[_address] = rx[2]; //  update mirror location with returned byte

    if ( verbosity )
    { //  verbosity output
	printRegisterName (_address);
	printHex (_address);
	printAllStr (", ");
	printHex (regData[_address]);
	printAllStr (", ");
	for ( uint8_t j = 0; j < 8; j++ )
	{
	    char buf[3];
	    int a = regData[_address] & (1 << (7 - j));
	    printAllStr (itoa (a, buf, 10));
	    if ( j != 7 )
		printAllStr (", ");
	}

	printlnAll ();
    }
    return regData[_address]; // return requested register value
}

// Read more than one register starting at _address
void
RREGS ( uint8_t _address, uint8_t _numRegistersMinusOne, int targetSS )
{

    uint8_t txbuf[_numRegistersMinusOne + 3];
    uint8_t rxbuf[_numRegistersMinusOne + 3];

    memset(txbuf,0,_numRegistersMinusOne + 3);
    memset(rxbuf,0,_numRegistersMinusOne + 3);

    txbuf[0] = _address + 0x20; //  opcode1 RREG expects 001rrrrr where rrrrr = _address
    txbuf[1] = _numRegistersMinusOne;	//  opcode2

    HAL_SPI_TransmitReceive (&hspi3, txbuf, rxbuf, _numRegistersMinusOne + 3, 1000);

    for ( int i = 0; i <= _numRegistersMinusOne; i++ )
    {
	regData[_address + i] = rxbuf[i + 2]; //  add register byte to mirror array
    }

//  uint8_t opcode1 = _address + 0x20; //  RREG expects 001rrrrr where rrrrr = _address
//  csLow(targetSS);                //  open SPI
//  xfer(opcode1);                  //  opcode1
//  xfer(_numRegistersMinusOne);    //  opcode2
//  for (int i = 0; i <= _numRegistersMinusOne; i++)
//  {
//    regData[_address + i] = xfer(0x00); //  add register byte to mirror array
//  }
//  csHigh(targetSS); //  close SPI

    if ( verbosity )
    { //  verbosity output
	for ( int i = 0; i <= _numRegistersMinusOne; i++ )
	{
	    printRegisterName (_address + i);
	    printHex (_address + i);
	    printAllStr (", ");
	    printHex (regData[_address + i]);
	    printAllStr (", ");
	    for ( int j = 0; j < 8; j++ )
	    {
		char buf[3];

		int a = regData[_address + i] & (1 << (7 - j));
		printAllStr (itoa (a, buf, 10));

//        printAllStr(itoa(bitRead(regData[_address + i], 7 - j), buf, 10));
		if ( j != 7 )
		    printAllStr (", ");
	    }
	    printlnAll ();
	    if ( !commandFromSPI )
		osDelay (30);
	}
    }
}

void
WREG ( uint8_t _address, uint8_t _value, int target_SS )
{                                 //  Write ONE register at _address
    uint8_t tx[3] = { 0 };

    tx[0] = _address + 0x40; // opcode1 WREG expects 010rrrrr where rrrrr = _address
    tx[1] = 0; 			// opcode2 Send number of registers to read -1
    tx[2] = _value;		// Write the value to the register

    HAL_SPI_Transmit (&hspi3, tx, 3, 1000);

//  uint8_t opcode1 = _address + 0x40; //  WREG expects 010rrrrr where rrrrr = _address
//  csLow(target_SS);               //  open SPI
//  xfer(opcode1);                  //  Send WREG command & address
//  xfer(0x00);                     //  Send number of registers to read -1
//  xfer(_value);                   //  Write the value to the register
//  csHigh(target_SS);              //  close SPI

    regData[_address] = _value;     //  update the mirror array

    if ( verbosity )
    { //  verbosity output
	printAllStr ("Register ");
	printHex (_address);
	printlnAllStr (" modified.");
	sendEOT ();
    }
}

void
WREGS ( uint8_t _address, uint8_t _numRegistersMinusOne, int targetSS )
{
    uint8_t buf[_numRegistersMinusOne + 3];

    buf[0] = _address + 0x40; //  opcode1 WREG expects 010rrrrr where rrrrr = _address
    buf[1] = _numRegistersMinusOne;//  opcode1 Send number of registers to write -1

    for ( int i = _address; i <= (_address + _numRegistersMinusOne); i++ )
    {
	buf[i - _address + 2] = regData[i]; //  Write to the registers
    }

    HAL_SPI_Transmit (&hspi3, buf, _numRegistersMinusOne + 3, 1000);

//  uint8_t opcode1 = _address + 0x40; //  WREG expects 010rrrrr where rrrrr = _address
//  csLow(targetSS);                //  open SPI
//  xfer(opcode1);                  //  Send WREG command & address
//  xfer(_numRegistersMinusOne);    //  Send number of registers to read -1
//  for (int i = _address; i <= (_address + _numRegistersMinusOne); i++)
//  {
//    xfer(regData[i]); //  Write to the registers
//  }
//  csHigh(targetSS);

    if ( verbosity )
    {
	printAllStr ("Registers ");
	printHex (_address);
	printAllStr (" to ");
	printHex (_address + _numRegistersMinusOne);
	printlnAllStr (" modified");
	sendEOT ();
    }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF ADS1299 FUNCTIONS  >>>>>>>>>>>>>>>>>>>>>>>>>
// ******************************************************************************

// String-Byte converters for ADS
void
printRegisterName ( uint8_t _address )
{
    switch ( _address )
	{
	case ID_REG:
	    printAllStr ("ADS_ID, ");
	    break;
	case CONFIG1:
	    printAllStr ("CONFIG1, ");
	    break;
	case CONFIG2:
	    printAllStr ("CONFIG2, ");
	    break;
	case CONFIG3:
	    printAllStr ("CONFIG3, ");
	    break;
	case LOFF:
	    printAllStr ("LOFF, ");
	    break;
	case CH1SET:
	    printAllStr ("CH1SET, ");
	    break;
	case CH2SET:
	    printAllStr ("CH2SET, ");
	    break;
	case CH3SET:
	    printAllStr ("CH3SET, ");
	    break;
	case CH4SET:
	    printAllStr ("CH4SET, ");
	    break;
	case CH5SET:
	    printAllStr ("CH5SET, ");
	    break;
	case CH6SET:
	    printAllStr ("CH6SET, ");
	    break;
	case CH7SET:
	    printAllStr ("CH7SET, ");
	    break;
	case CH8SET:
	    printAllStr ("CH8SET, ");
	    break;
	case BIAS_SENSP:
	    printAllStr ("BIAS_SENSP, ");
	    break;
	case BIAS_SENSN:
	    printAllStr ("BIAS_SENSN, ");
	    break;
	case LOFF_SENSP:
	    printAllStr ("LOFF_SENSP, ");
	    break;
	case LOFF_SENSN:
	    printAllStr ("LOFF_SENSN, ");
	    break;
	case LOFF_FLIP:
	    printAllStr ("LOFF_FLIP, ");
	    break;
	case LOFF_STATP:
	    printAllStr ("LOFF_STATP, ");
	    break;
	case LOFF_STATN:
	    printAllStr ("LOFF_STATN, ");
	    break;
	case GPIO:
	    printAllStr ("GPIO, ");
	    break;
	case MISC1:
	    printAllStr ("MISC1, ");
	    break;
	case MISC2:
	    printAllStr ("MISC2, ");
	    break;
	case CONFIG4:
	    printAllStr ("CONFIG4, ");
	    break;
	default:
	    break;
	}
}

// Used for printing HEX in verbosity feedback mode
void
printHex ( uint8_t _data )
{
    if ( _data < 0x10 )
	printAllStr ("0");
    char buf[4];
    // Serial.print(_data);
    printAllStr (itoa (_data, buf, 16));
    if ( commandFromSPI )
	osDelay (1);
}

void
printlnHex ( uint8_t _data )
{
    printHex (_data);
    printlnAll ();
}

void
printFailure ()
{
    printAllStr ("Failure: ");
}

void
printSuccess ()
{
    printAllStr ("Success: ");
}

void
printAllchar ( char c )
{
    printSerialChar (c);
//  if (wifi.present && wifi.tx)
//  {
//    wifi.sendStringMulti(c);
//    osDelay(1);
//  }
}

void
printAllStr ( const char *arr )
{
    printSerialStr (arr);
//  if (wifi.present && wifi.tx)
//  {
//    wifi.sendStringMulti(arr);
//    osDelay(1);
//  }
}

void
printlnAllStr ( const char *arr )
{
    printlnSerialStr (arr);
//  if (wifi.present && wifi.tx)
//  {
//    wifi.sendStringMulti(arr);
//    osDelay(1);
//    wifi.sendStringMulti("\n");
//    osDelay(1);
//  }
}

void
printlnAll ( void )
{
    printlnSerial ();
//  if (wifi.present && wifi.tx)
//  {
//    wifi.sendStringMulti("\n");
//    osDelay(1);
//  }
}

/**
 * @description Converts ascii character to byte value for channel setting bytes
 * @param `asciiChar` - [char] - The ascii character to convert
 * @return [char] - Byte number value of acsii character, defaults to 0
 * @author AJ Keller (@pushtheworldllc)
 */
char
getChannelCommandForAsciiChar ( char asciiChar )
{
    switch ( asciiChar )
	{
	case OPENBCI_CHANNEL_CMD_CHANNEL_1:
	    return 0x00;
	case OPENBCI_CHANNEL_CMD_CHANNEL_2:
	    return 0x01;
	case OPENBCI_CHANNEL_CMD_CHANNEL_3:
	    return 0x02;
	case OPENBCI_CHANNEL_CMD_CHANNEL_4:
	    return 0x03;
	case OPENBCI_CHANNEL_CMD_CHANNEL_5:
	    return 0x04;
	case OPENBCI_CHANNEL_CMD_CHANNEL_6:
	    return 0x05;
	case OPENBCI_CHANNEL_CMD_CHANNEL_7:
	    return 0x06;
	case OPENBCI_CHANNEL_CMD_CHANNEL_8:
	    return 0x07;
	case OPENBCI_CHANNEL_CMD_CHANNEL_9:
	    return 0x08;
	case OPENBCI_CHANNEL_CMD_CHANNEL_10:
	    return 0x09;
	case OPENBCI_CHANNEL_CMD_CHANNEL_11:
	    return 0x0A;
	case OPENBCI_CHANNEL_CMD_CHANNEL_12:
	    return 0x0B;
	case OPENBCI_CHANNEL_CMD_CHANNEL_13:
	    return 0x0C;
	case OPENBCI_CHANNEL_CMD_CHANNEL_14:
	    return 0x0D;
	case OPENBCI_CHANNEL_CMD_CHANNEL_15:
	    return 0x0E;
	case OPENBCI_CHANNEL_CMD_CHANNEL_16:
	    return 0x0F;
	default:
	    return 0x00;
	}
}

/**
 * @description Converts ascii '0' to number 0 and ascii '1' to number 1
 * @param `asciiChar` - [char] - The ascii character to convert
 * @return [char] - Byte number value of acsii character, defaults to 0
 * @author AJ Keller (@pushtheworldllc)
 */
char
getYesOrNoForAsciiChar ( char asciiChar )
{
    switch ( asciiChar )
	{
	case '1':
	    return ACTIVATE;
	case '0':
	default:
	    return DEACTIVATE;
	}
}

/**
 * @description Converts ascii character to get gain from channel settings
 * @param `asciiChar` - [char] - The ascii character to convert
 * @return [char] - Byte number value of acsii character, defaults to 0
 * @author AJ Keller (@pushtheworldllc)
 */
char
getGainForAsciiChar ( char asciiChar )
{

    char output = 0x00;

    if ( asciiChar < '0' || asciiChar > '6' )
    {
	asciiChar = '6'; // Default to 24
    }

    output = asciiChar - '0';

    return output << 4;
}

/**
 * @description Converts ascii character to get gain from channel settings
 * @param `asciiChar` - [char] - The ascii character to convert
 * @return [char] - Byte number value of acsii character, defaults to 0
 * @author AJ Keller (@pushtheworldllc)
 */
char
getNumberForAsciiChar ( char asciiChar )
{
    if ( asciiChar < '0' || asciiChar > '9' )
    {
	asciiChar = '0';
    }

    // Convert ascii char to number
    asciiChar -= '0';

    return asciiChar;
}

/**
 * @description Used to set the channelSettings array to default settings
 * @param `setting` - [byte] - The byte you need a setting for....
 * @returns - [byte] - Retuns the proper byte for the input setting, defualts to 0
 */
uint8_t
getDefaultChannelSettingForSetting ( uint8_t setting )
{
    switch ( setting )
	{
	case POWER_DOWN:
	    return NO;
	case GAIN_SET:
	    return ADS_GAIN24;
	case INPUT_TYPE_SET:
	    return ADSINPUT_NORMAL;
	case BIAS_SET:
	    return YES;
	case SRB2_SET:
	    return YES;
	case SRB1_SET:
	default:
	    return NO;
	}
}

/**
 * @description Used to set the channelSettings array to default settings
 * @param `setting` - [byte] - The byte you need a setting for....
 * @returns - [char] - Retuns the proper ascii char for the input setting, defaults to '0'
 */
char
getDefaultChannelSettingForSettingAscii ( uint8_t setting )
{
    switch ( setting )
	{
	case GAIN_SET: // Special case where GAIN_SET needs to be shifted first
	    return (ADS_GAIN24 >> 4) + '0';
	default: // All other settings are just adding the ascii value for '0'
	    return getDefaultChannelSettingForSetting (setting) + '0';
	}
}

/**
 * @description Convert user channelNumber for use in array indexs by subtracting 1,
 *                  also make sure N is not greater than 15 or less than 0
 * @param `channelNumber` - [byte] - The channel number
 * @return [byte] - Constrained channel number
 */
char
getConstrainedChannelNumber ( uint8_t channelNumber )
{
    if ( (channelNumber - 1) > (OPENBCI_NUMBER_OF_CHANNELS_DAISY - 1) )
	channelNumber = OPENBCI_NUMBER_OF_CHANNELS_DAISY - 1;
    else if ( (channelNumber - 1) < 0 )
	channelNumber = 0;
    else
	channelNumber = channelNumber - 1;

    return channelNumber;
}

/**
 * @description Get slave select pin for channelNumber
 * @param `channelNumber` - [byte] - The channel number
 * @return [byte] - Constrained channel number
 */
char
getTargetSSForConstrainedChannelNumber ( uint8_t channelNumber )
{
    // Is channelNumber in the range of default [0,7]
    if ( channelNumber < OPENBCI_NUMBER_OF_CHANNELS_DEFAULT )
    {
	return BOARD_ADS;
    }
    else
    {
	return DAISY_ADS;
    }
}

/**
 * @description Used to set the channelSettings array to default settings
 * @param `channelSettingsArray` - [byte **] - Takes a two dimensional array of
 *          length OPENBCI_NUMBER_OF_CHANNELS_DAISY by 6 elements
 */
void
resetChannelSettingsArrayToDefault (
	uint8_t channelSettingsArray[][OPENBCI_NUMBER_OF_CHANNEL_SETTINGS] )
{
    // Loop through all channels
    for ( int i = 0; i < OPENBCI_NUMBER_OF_CHANNELS_DAISY; i++ )
    {
	channelSettingsArray[i][POWER_DOWN] =
		getDefaultChannelSettingForSetting (POWER_DOWN); // on = NO, off = YES
	channelSettingsArray[i][GAIN_SET] = getDefaultChannelSettingForSetting (
		GAIN_SET);             // Gain setting
	channelSettingsArray[i][INPUT_TYPE_SET] =
		getDefaultChannelSettingForSetting (INPUT_TYPE_SET); // input muxer setting
	channelSettingsArray[i][BIAS_SET] = getDefaultChannelSettingForSetting (
		BIAS_SET);             // add this channel to bias generation
	channelSettingsArray[i][SRB2_SET] = getDefaultChannelSettingForSetting (
		SRB2_SET);             // connect this P side to SRB2
	channelSettingsArray[i][SRB1_SET] = getDefaultChannelSettingForSetting (
		SRB1_SET);             // don't use SRB1

	useInBias[i] = true; // keeping track of Bias Generation
	useSRB2[i] = true;   // keeping track of SRB2 inclusion
    }

//    boardUseSRB1 = daisyUseSRB1 = false;
    boardUseSRB1 = false;
}

/**
 * @description Used to set the channelSettings array to default settings
 * @param `channelSettingsArray` - [byte **] - A two dimensional array of
 *          length OPENBCI_NUMBER_OF_CHANNELS_DAISY by 2 elements
 */
void
resetLeadOffArrayToDefault (
	uint8_t leadOffArray[][OPENBCI_NUMBER_OF_LEAD_OFF_SETTINGS] )
{
    // Loop through all channels
    for ( int i = 0; i < OPENBCI_NUMBER_OF_CHANNELS_DAISY; i++ )
    {
	leadOffArray[i][PCHAN] = OFF;
	leadOffArray[i][NCHAN] = OFF;
    }
}

//OpenBCI_32bit_Library board;
