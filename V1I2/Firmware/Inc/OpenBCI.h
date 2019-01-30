/*
 * OpenBCI.h
 *
 *  Created on: 11 дек. 2018 г.
 *      Author: ai
 */

#include "OpenBCIdefinies.h"
#include "stm32f4xx_hal.h"

#ifndef OPENBCI_H_
#define OPENBCI_H_

//void __USER_ISR ADS_DRDY_Service(void);

// ENUMS
  typedef enum eACCEL_MODE {
    ACCEL_MODE_ON,
    ACCEL_MODE_OFF
  } ACCEL_MODE;

  typedef enum eTIME_SYNC_MODE {
    TIME_SYNC_MODE_ON,
    TIME_SYNC_MODE_OFF
  } TIME_SYNC_MODE;

  typedef enum eMARKER_MODE {
    MARKER_MODE_ON,
    MARKER_MODE_OFF
  } MARKER_MODE;

  typedef enum eDEBUG_MODE {
    DEBUG_MODE_ON,
    DEBUG_MODE_OFF
  } DEBUG_MODE;

  typedef enum eBOARD_MODE {
    BOARD_MODE_DEFAULT,
    BOARD_MODE_DEBUG,
    BOARD_MODE_ANALOG,
    BOARD_MODE_DIGITAL,
    BOARD_MODE_MARKER,
    BOARD_MODE_BLE,
    BOARD_MODE_END_OF_MODES  // This must be the last entry-insert any new board modes above this line
  } BOARD_MODE;

  typedef enum eMULTI_CHAR_COMMAND {
    MULTI_CHAR_CMD_NONE,
    MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_CHANNEL,
    MULTI_CHAR_CMD_PROCESSING_INCOMING_SETTINGS_LEADOFF,
    MULTI_CHAR_CMD_SERIAL_PASSTHROUGH,
    MULTI_CHAR_CMD_SETTINGS_BOARD_MODE,
    MULTI_CHAR_CMD_SETTINGS_SAMPLE_RATE,
    MULTI_CHAR_CMD_INSERT_MARKER
  } MULTI_CHAR_COMMAND;

  typedef enum ePACKET_TYPE {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_RAW_AUX,
    PACKET_TYPE_USER_DEFINED,
    PACKET_TYPE_ACCEL_TIME_SET,
    PACKET_TYPE_ACCEL_TIME_SYNC,
    PACKET_TYPE_RAW_AUX_TIME_SET,
    PACKET_TYPE_RAW_AUX_TIME_SYNC
  } PACKET_TYPE;

  typedef enum eSAMPLE_RATE {
    SAMPLE_RATE_16000,
    SAMPLE_RATE_8000,
    SAMPLE_RATE_4000,
    SAMPLE_RATE_2000,
    SAMPLE_RATE_1000,
    SAMPLE_RATE_500,
    SAMPLE_RATE_250
  } SAMPLE_RATE;

// STRUCTS
  typedef struct {
    uint32_t  baudRate;
    uint8_t   rx;
    uint8_t   tx;
  } SerialInfo;

  typedef struct {
    uint8_t   rx;
    uint8_t   tx;
  } SpiInfo;

//  typedef struct {
//    uint8_t sampleNumber;
//    uint8_t data[BLE_TOTAL_DATA_BYTES];
//    uint8_t ready;
//    uint8_t flushing;
//    uint8_t bytesFlushed;
//    uint8_t bytesLoaded;
//  } BLE;

  // Start up functions
//  OpenBCI_32bit_Library();
  uint8_t accelHasNewData(void);
  void    accelUpdateAxisData(void);
  void    accelWriteAxisDataSerial(void);
  void    activateAllChannelsToTestCondition(uint8_t testInputCode, uint8_t amplitudeCode, uint8_t freqCode);
  void    activateChannel(uint8_t);                  // enable the selected channel
  void    ADS_writeChannelData(void);
  void    ADS_writeChannelDataAvgDaisy(void);
  void    ADS_writeChannelDataNoAvgDaisy(void);
//  void    attachDaisy(void);
  void    begin(void);
  void    beginDebug(void);
  void    beginDebugBaudRate(uint32_t);
  void    beginPinsAnalog(void);
  void    beginPinsDebug(void);
  void    beginPinsDefault(void);
  void    beginPinsDigital(void);
//  void    beginSerial0(void);
//  void    beginSerial0BaudRate(uint32_t);
//  void    beginSerial1(void);
//  void    beginSerial1BaudRate(uint32_t);
  void    boardReset(void);
  void    changeChannelLeadOffDetect();
  void    changeChannelLeadOffDetectN(uint8_t N);
  void    channelSettingsArraySetForAll(void);
  void    channelSettingsArraySetForChannel(uint8_t N);
  void    channelSettingsSetForChannel(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  uint8_t checkMultiCharCmdTimer(void);
//  void    csLow(int);
//  void    csHigh(int);
  void    configureInternalTestSignal(uint8_t,uint8_t);
  void    configureLeadOffDetection(uint8_t, uint8_t);
  void    deactivateChannel(uint8_t);                // disable given channel 1-8(16)
//  void    disable_accel(void); // stop data acquisition and go into low power mode
//  void    enable_accel(uint8_t);  // start acceleromoeter with default settings
  void    endMultiCharCmdTimer(void);
//  void    endSerial0(void);
//  void    endSerial1(void);
  const char* getBoardMode(void);
  char    getChannelCommandForAsciiChar(char);
//  char    getCharSerial0(void);
//  char    getCharSerial1(void);
  char    getConstrainedChannelNumber(uint8_t);
  uint8_t getDefaultChannelSettingForSetting(uint8_t);
  char    getDefaultChannelSettingForSettingAscii(uint8_t);
  char    getGainForAsciiChar(char);
  uint8_t * getGains(void);
  char    getMultiCharCommand(void);
  char    getNumberForAsciiChar(char);
  const char* getSampleRate(void);
  char    getTargetSSForConstrainedChannelNumber(uint8_t);
  char    getYesOrNoForAsciiChar(char);
//  uint8_t hasDataSerial0(void);
//  uint8_t hasDataSerial1(void);
  uint8_t isADSDataAvailable(void);
  uint8_t isProcessingMultibyteMsg(void);
  void    leadOffConfigureSignalForAll(uint8_t, uint8_t);
  void    leadOffConfigureSignalForTargetSS(uint8_t, uint8_t, uint8_t);
  void    leadOffSetForAllChannels(void);
  void    leadOffSetForChannel(uint8_t, uint8_t, uint8_t);
  void    ledFlash(int);
  void    loop(void);
  void    printAllChar(char);
  void    printAllStr(const char *);
  void    printlnAll(void);
  void    printlnAllStr(const char *);
  void    printlnSerial(void);
  void    printlnSerialChar(char);
  void    printlnSerialInt(int);
  void    printlnSerialIntInt(int, int);
  void    printlnSerialStr(const char *);
  void    printSerialInt(int);
  void    printSerialChar(char);
  void    printSerialIntInt(int, int);
  void    printSerialStr(const char *);
  uint8_t processChar(char);
  uint8_t processCharWifi(char);
  void    processIncomingBoardMode(char);
  void    processIncomingSampleRate(char);
  void    processInsertMarker(char);
  void    processIncomingChannelSettings(char);
  void    processIncomingLeadOffSettings(char);
  void    reportDefaultChannelSettings(void);
//  void    removeDaisy(void);
  void    resetADS(int);     // reset all the ADS1299's settings
  void    resetChannelSettingsArrayToDefault(uint8_t channelSettingsArray[][OPENBCI_NUMBER_OF_CHANNEL_SETTINGS]);
  void    resetLeadOffArrayToDefault(uint8_t leadOffArray[][OPENBCI_NUMBER_OF_LEAD_OFF_SETTINGS]);
  void    startADS(void);
  void    stopADS(void);
  void    sendChannelData(void);
  void    sendChannelDataPacketType(PACKET_TYPE);
  void    setBoardMode(uint8_t);
  void    setChannelsToDefault(void);
  void    setCurPacketType(void);
  void    setSampleRate(uint8_t newSampleRateCode);
  void    sendEOT(void);
  void    setSerialInfo(SerialInfo, uint8_t, uint8_t, uint32_t);
  uint8_t smellDaisy(void);
  void    startMultiCharCmdTimer(char);
  void    streamSafeChannelDeactivate(uint8_t);
  void    streamSafeChannelActivate(uint8_t);
  void    streamSafeSetSampleRate(SAMPLE_RATE);
  void    streamSafeChannelSettingsForChannel(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  void    streamSafeSetAllChannelsToDefault(void);
  void    streamSafeReportAllChannelDefaults(void);
  void    streamSafeLeadOffSetForChannel(uint8_t, uint8_t, uint8_t);
  void    streamSafeTimeSendSyncSetPacket(void);
  void    streamStart(void);
  void    streamStop(void);
  void    tryMultiAbort(void);
  void    updateBoardData(void);
  void    updateBoardDataDownSample(uint8_t);
  void    updateChannelData(void);   // retrieve data from ADS
//  void    updateDaisyData(void);
//  void    updateDaisyDataDownSample(uint8_t);
  void    useAccel(uint8_t);
  void    useTimeStamp(uint8_t);
  void    write(uint8_t);
  void    writeAuxDataSerial(void);
  void    writeChannelSettings(void);
  void    writeChannelSettingsN(uint8_t);
  void    writeSerial(uint8_t);
  void    writeSpi(uint8_t);
  void    writeTimeCurrent(void);
  void    writeTimeCurrentSerial(uint32_t newTime);
  void    writeZeroAux(void);
  void    zeroAuxData(void);

  // Variables
  uint8_t boardUseSRB1;             // used to keep track of if we are using SRB1
//  uint8_t daisyPresent;
//  uint8_t daisyUseSRB1;
  uint8_t streaming;
  uint8_t useInBias[OPENBCI_NUMBER_OF_CHANNELS_DAISY];        // used to remember if we were included in Bias before channel power down
  uint8_t useSRB2[OPENBCI_NUMBER_OF_CHANNELS_DAISY];
  uint8_t verbosity; // turn on/off Serial verbosity

  uint8_t boardChannelDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];    // array to hold raw channel data
  uint8_t channelSettings[OPENBCI_NUMBER_OF_CHANNELS_DAISY][OPENBCI_NUMBER_OF_CHANNEL_SETTINGS];  // array to hold current channel settings
//  uint8_t daisyChannelDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];
  uint8_t defaultChannelSettings[OPENBCI_NUMBER_OF_CHANNEL_SETTINGS];  // default channel settings
  uint8_t lastBoardDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];
//  uint8_t lastDaisyDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];
  uint8_t leadOffSettings[OPENBCI_NUMBER_OF_CHANNELS_DAISY][OPENBCI_NUMBER_OF_LEAD_OFF_SETTINGS];  // used to control on/off of impedance measure for P and N side of each channel
  uint8_t meanBoardDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];
  uint8_t meanDaisyDataRaw[OPENBCI_NUMBER_BYTES_PER_ADS_SAMPLE];
  uint8_t sampleCounter;
  uint8_t sampleCounterBLE;

//  int ringBufBLEHead;
//  int ringBufBLETail;
  int boardChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];    // array used when reading channel data as ints
//  int daisyChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];    // array used when reading channel data as ints
  int lastBoardChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];
//  int lastDaisyChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];
  int meanBoardChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];
  int meanDaisyChannelDataInt[OPENBCI_NUMBER_CHANNELS_PER_ADS_SAMPLE];
  int numChannels;

  short auxData[3]; // This is user faceing
  short axisData[3];

  unsigned long lastSampleTime;

  volatile uint8_t channelDataAvailable;

  // ENUMS
  ACCEL_MODE curAccelMode;
  BOARD_MODE curBoardMode;
  DEBUG_MODE curDebugMode;
  PACKET_TYPE curPacketType;
  SAMPLE_RATE curSampleRate;
  TIME_SYNC_MODE curTimeSyncMode;

  // Stucts
//  BLE bufferBLE[BLE_RING_BUFFER_SIZE];

  // STRUCTS
//  SerialInfo iSerial0;
//  SerialInfo iSerial1;

  // Class Objects
//  DSPI0 spi;  // use DSPI library

// #ifdef __OpenBCI_Wifi_Master__
//  void    accelWriteAxisDataWifi(void);
//  void    ADS_writeChannelDataWifi(uint8_t daisy);
//  void    writeAuxDataWifi(void);
//  void    writeTimeCurrentWifi(uint32_t newTime);
// #endif

//private:

  uint8_t ADS_getDeviceID(int);
  void    boardBeginADSInterrupt(void);
  uint8_t boardBegin(void);
  uint8_t boardBeginDebug(void);
  uint8_t boardBeginDebugBaudRate(int);
  void    changeInputType(uint8_t);
//  int     getX(void);
//  int     getY(void);
//  int     getZ(void);
//  void    bufferBLEReset(void);
//  void    bufferBLEReset(BLE *);
  void    initialize(void);
//  void    initialize_accel(uint8_t);    // initialize
  void    initialize_ads(void);
  void    initializeSerialInfo(SerialInfo);
  void    initializeVariables(void);
  void    initializeSpiInfo(SpiInfo);
//  uint8_t    LIS3DH_getDeviceID(void);
//  uint8_t    LIS3DH_read(uint8_t);     // read a register on LIS3DH
//  int     LIS3DH_read16(uint8_t);    // read two bytes, used to get axis data
//  void    LIS3DH_write(uint8_t,uint8_t);   // write a register on LIS3DH
//  uint8_t LIS3DH_DataReady(void); // check LIS3DH_DRDY pin
//  uint8_t LIS3DH_DataAvailable(void); // check LIS3DH STATUS_REG2
//  void    LIS3DH_readAllRegs(void);
//  void    LIS3DH_writeAxisDataSerial(void);
//  void    LIS3DH_writeAxisDataForAxisSerial(uint8_t);
//  void    LIS3DH_updateAxisData(void);
//  void    LIS3DH_zeroAxisData(void);
  void    printADSregisters(int);
  void    printAllRegisters(void);
  void    printFailure();
  void    printHex(uint8_t);
  void    printlnHex(uint8_t);
  void    printRegisterName(uint8_t);
  void    printSuccess();
  void    RDATA(int);   // read data one-shot
  void    RDATAC(int);  // go into read data continuous mode
  void    RRESET(int);   // set all register values to default
  uint8_t RREG(uint8_t,int);            // read one ADS register
  void    RREGS(uint8_t,uint8_t,int);      // read multiple ADS registers
  void    SDATAC(int);  // get out of read data continuous mode
  void    sendChannelDataSerial(PACKET_TYPE);
//  void    sendChannelDataSerialBLE(PACKET_TYPE packetType);
  void    sendTimeWithAccelSerial(void);
  void    sendTimeWithRawAuxSerial(void);
  void    STANDBY(int); // go into low power mode
  void    START(int);   // start data acquisition
  void    STOP(int);    // stop data acquisition
  void    WAKEUP(int);  // get out of low power mode
  void    WREG(uint8_t,uint8_t,int);       // write one ADS register
  void    WREGS(uint8_t,uint8_t,int);      // write multiple ADS registers
  uint8_t    xfer(uint8_t);        // SPI Transfer function

  // Variables
  uint8_t commandFromSPI;
  uint8_t firstDataPacket;
  uint8_t isMultiCharCmd;  // A multi char command is in progress
  uint8_t isRunning;
  uint8_t settingBoardMode;
  uint8_t settingSampleRate;
  uint8_t newMarkerReceived;  // flag to indicate a new marker has been received
  uint8_t regData[24]; // array is used to mirror register data
  char    buffer[1];
  char    markerValue;
  char    multiCharCommand;  // The type of command
  uint8_t    currentChannelSetting;
  char    optionalArgBuffer5[5];
  char    optionalArgBuffer6[6];
  char    optionalArgBuffer7[7];
  int     boardStat; // used to hold the status register
  int     daisyStat;
  int     DRDYpinValue;
  int     lastDRDYpinValue;
  int     numberOfIncomingSettingsProcessedChannel;
  int     numberOfIncomingSettingsProcessedLeadOff;
  int     numberOfIncomingSettingsProcessedBoardType;
  uint8_t optionalArgCounter;
  unsigned long multiCharCmdTimeout;  // the timeout in millis of the current multi char command
  unsigned long timeOfLastRead;
  unsigned long timeOfMultiByteMsgStart;

//#ifdef __OpenBCI_Wifi_Master__
//  // functions
//  void    LIS3DH_writeAxisDataWifi(void);
//  void    LIS3DH_writeAxisDataForAxisWifi(uint8_t);
//  void    sendChannelDataWifi(PACKET_TYPE, uint8_t);
//  void    sendRawAuxWifi(void);
//  void    sendTimeWithAccelWifi(void);
//  void    sendTimeWithRawAuxWifi(void);
//#endif
//};

// This let's us call into the class from within the library if necessary
//extern OpenBCI_32bit_Library board;

#endif /* OPENBCI_H_ */
