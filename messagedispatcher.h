#ifndef MESSAGEDISPATCHER_H
#define MESSAGEDISPATCHER_H

#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <unordered_map>

#include "er4commlib_errorcodes.h"
#include "er4commlib_global.h"
#include "ftdieeprom.h"
#include "ftdieeprom56.h"
#include "ftdieepromdemo.h"
#include "commandcoder.h"

using namespace std;
using namespace er4CommLib;

//#define DEBUG_PRINT

#ifdef DEBUG_PRINT
//#define DEBUG_RAW_BIT_RATE_PRINT
#endif

#define SHORT_OFFSET_BINARY (static_cast <double> (0x8000))
#define SHORT_MAX (static_cast <double> (0x7FFF))
#define SHORT_MIN (-SHORT_MAX-1.0)
#define USHORT_MAX (static_cast <double> (0xFFFF))
#define INT18_MAX (static_cast <double> (0x1FFFF))
#define UINT28_MAX (static_cast <double> (0xFFFFFFF))

#define LSB_NOISE_ARRAY_SIZE 0x40000 /*! \todo FCON valutare che questo numero sia adeguato */ // ~250k
#define LSB_NOISE_ARRAY_MASK (LSB_NOISE_ARRAY_SIZE-1) // 0b11...1 for all bits of the array indexes

#define IIR_ORD 2
#define IIR_2_SIN_PI_8 (2.0*sin(M_PI/8.0))
#define IIR_2_COS_PI_8 (2.0*cos(M_PI/8.0))
#define IIR_2_COS_PI_8_2 (IIR_2_COS_PI_8*IIR_2_COS_PI_8)
#define IIR_2_SIN_3_PI_8 (2.0*sin(3.0*M_PI/8.0))
#define IIR_2_COS_3_PI_8 (2.0*cos(3.0*M_PI/8.0))
#define IIR_2_COS_3_PI_8_2 (IIR_2_COS_3_PI_8*IIR_2_COS_3_PI_8)

#define FTD_MAX_RESEND_TRIES 10
#define FTD_MAX_FPGA_RESET_TRIES 10
#define FTD_MAX_WRITE_TRIES 10

#define FTD_RX_WORD_SIZE (sizeof(uint16_t)) // 16 bit word
#define FTD_RX_SYNC_WORD_SIZE ((int)(2*FTD_RX_WORD_SIZE)) // could be different size
#define FTD_RX_BUFFER_SIZE 0x20000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
#define FTD_RX_BUFFER_MASK (FTD_RX_BUFFER_SIZE-1)
#define FTD_DEFAULT_MIN_FRAME_NUMBER 10
#define FTD_DEFAULT_FEW_FRAME_SLEEP 2000
#define FTD_FEW_PACKET_COEFF 0.01 /*!< = 10.0/1000.0: 10.0 because I want to get data once every 10ms, 1000 to convert sampling rate from Hz to kHz */
#define FTD_MAX_BYTES_TO_WAIT_FOR 2048

#define ER4CL_OUTPUT_BUFFER_SIZE 0x100000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
#define ER4CL_OUTPUT_BUFFER_MASK (ER4CL_OUTPUT_BUFFER_SIZE-1)

#define FTD_TX_WORD_SIZE (sizeof(uint8_t)) // 1 bite word
#define FTD_TX_SYNC_WORD_SIZE (FTD_TX_WORD_SIZE) // could be different size
#define FTD_TX_RAW_BUFFER_SIZE 0x1000 /*! \todo FCON valutare che questo numero sia adeguato */ // ~4k
#define FTD_TX_RAW_BUFFER_MASK (FTD_TX_RAW_BUFFER_SIZE-1) // 0b11...1 for all bits of the buffer indexes
#define FTD_TX_MSG_BUFFER_SIZE 0x100 /*! \todo FCON valutare che questo numero sia adeguato */ // 256
#define FTD_TX_MSG_BUFFER_MASK (FTD_TX_MSG_BUFFER_SIZE-1)

/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcher                                        *
 *                                                                                          *
\********************************************************************************************/

class MessageDispatcher {
public:

    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher(string deviceId);
    virtual ~MessageDispatcher();

    /************************\
     *  Connection methods  *
    \************************/

    ErrorCodes_t init();
    ErrorCodes_t deinit();
    virtual ErrorCodes_t connect(FtdiEeprom * ftdiEeprom);
    virtual ErrorCodes_t disconnect();
    void readDataFromDevice();
    void sendCommandsToDevice();

    static ErrorCodes_t getDeviceType(DeviceTuple_t tuple, DeviceTypes_t &type);

    /****************\
     *  Tx methods  *
    \****************/

    ErrorCodes_t turnOnLsbNoise(bool flag);
    ErrorCodes_t convertVoltageValue(uint16_t intValue, double &fltValue);
    ErrorCodes_t convertCurrentValue(uint16_t intValue, double &fltValue);
    ErrorCodes_t setVoltageRange(uint16_t voltageRangeIdx);
    virtual ErrorCodes_t setCurrentRange(uint16_t currentRangeIdx);
    virtual ErrorCodes_t setSamplingRate(uint16_t samplingRateIdx, bool applyFlag = true);

    virtual ErrorCodes_t digitalOffsetCompensation(bool on, bool applyFlag = true);
    ErrorCodes_t zap(uint16_t channelIdx, bool applyFlag = true);

    ErrorCodes_t resetDevice(bool reset, bool applyFlag = true);
    ErrorCodes_t resetDigitalOffsetCompensation(bool reset);

    ErrorCodes_t sendCommands();
    ErrorCodes_t applyDacExt(Measurement_t voltage, bool applyFlag = true);
    ErrorCodes_t setInterposerInserted(bool flag, bool applyFlag = true);
    ErrorCodes_t setConditioningCheck(bool flag, bool applyFlag = true);
    ErrorCodes_t setConditioningChannel(uint16_t channelIdx, bool applyFlag = true);
    ErrorCodes_t selectVoltageProtocol(unsigned int idx, bool applyFlag = false);
    ErrorCodes_t applyVoltageProtocol();
    ErrorCodes_t setProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setTriangularVoltage(Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setProtocolTime(unsigned int idx, Measurement_t time, bool applyFlag = false);
    ErrorCodes_t setTriangularTime(Measurement_t time, bool applyFlag = false);
    ErrorCodes_t setFsmFlag(unsigned int idx, bool flag, bool applyFlag = true);
    ErrorCodes_t setFsmVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = true);
    ErrorCodes_t setFsmThresholdCurrent(unsigned int idx, Measurement_t current, bool applyFlag = true);
    ErrorCodes_t setFsmTime(unsigned int idx, Measurement_t time, bool applyFlag = true);
    ErrorCodes_t setFsmInteger(unsigned int idx, unsigned int value, bool applyFlag = true);
    ErrorCodes_t setMovingAverageFilterDuration(Measurement_t duration, bool applyFlag = true);
    ErrorCodes_t set4thOrderFilterCutoffFrequency(Measurement_t cFrequency, bool applyFlag = true);
    ErrorCodes_t setRawDataFilterCutoffFrequency(Measurement_t cFrequency);
    ErrorCodes_t setConditioningProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setCheckingProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setConditioningProtocolTime(unsigned int idx, Measurement_t time, bool applyFlag = false);
    ErrorCodes_t activateFEResetDenoiser(bool flag, bool applyFlag = true);
    ErrorCodes_t activateDacIntFilter(bool flag, bool applyFlag = true);
    ErrorCodes_t activateDacExtFilter(bool flag, bool applyFlag = true);
    ErrorCodes_t setVcmForce(bool flag, bool applyFlag = true);

    /****************\
     *  Rx methods  *
    \****************/

    ErrorCodes_t isDeviceUpgradable(string &upgradeNotes, string &notificationTag);
    ErrorCodes_t getDeviceInfo(uint8_t &deviceVersion, uint8_t &deviceSubversion, uint32_t &firmwareVersion);

    ErrorCodes_t getQueueStatus(unsigned int * availableDataPackets, bool * bufferOverflowFlag, bool * dataLossFlag);
    ErrorCodes_t getDataPackets(uint16_t * &data, unsigned int packetsNumber, unsigned int * packetsRead);
    ErrorCodes_t purgeData();

    ErrorCodes_t getChannelsNumber(uint32_t &fsmStateChannelsNumber, uint32_t &voltageChannelsNumber, uint32_t &currentChannelsNumber);

    ErrorCodes_t getCurrentRanges(vector <RangedMeasurement_t> &currentRanges);
    ErrorCodes_t getCurrentRange(RangedMeasurement_t &currentRange);
    ErrorCodes_t getCheckingCurrentRange(RangedMeasurement_t &currentRange);
    ErrorCodes_t getConditioningCurrentRange(RangedMeasurement_t &currentRange);

    ErrorCodes_t getVoltageRanges(vector <RangedMeasurement_t> &voltageRanges);
    ErrorCodes_t getVoltageRange(RangedMeasurement_t &voltageRange);
    ErrorCodes_t getCheckingVoltageRange(RangedMeasurement_t &voltageRange);
    ErrorCodes_t getConditioningVoltageRange(RangedMeasurement_t &voltageRange);

    ErrorCodes_t getSamplingRates(vector <Measurement_t> &samplingRates, unsigned int &defaultOption);
    ErrorCodes_t getRealSamplingRates(vector <Measurement_t> &samplingRates);

    ErrorCodes_t getVoltageStimulusLpfs(vector <string> &filterOptions);

    ErrorCodes_t getLiquidJunctionControl(CompensationControl_t &control);

    ErrorCodes_t getDacExtRange(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getProtocolList(vector <string> &protocolsNames, unsigned int &triangularIdx);
    ErrorCodes_t getProtocolVoltage(vector <string> &voltageNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getTriangularVoltage(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getProtocolTime(vector <string> &timeNames, RangedMeasurement_t &range, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getTriangularTime(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getFsmFlag(vector <string> &flagNames, vector <bool> &defaultValues);
    ErrorCodes_t getFsmVoltage(vector <string> &voltageNames, RangedMeasurement_t &range, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getFsmThresholdCurrent(vector <string> &currentNames, RangedMeasurement_t &range, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getFsmTime(vector <string> &timeNames, RangedMeasurement_t &range, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getFsmInteger(vector <string> &integerNames, RangedMeasurement_t &range, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getMovingAverageFilterDuration(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t get4thOrderFilterCutoffFrequency(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getRawDataFilterCutoffFrequency(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getConditioningProtocolVoltage(vector <string> &voltageNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getCheckingProtocolVoltage(vector <string> &voltageNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getConditioningProtocolTime(vector <string> &timeNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);

protected:
    typedef enum {
        RxParseLookForHeader,
        RxParseLookForLength,
        RxParseLookForCrc
    } RxParsePhase_t;

    typedef enum {
        ResetIndexChip,
        ResetIndexDigitalOffsetCompensation,
        ResetIndexNum
    } ResetIndex_t;

    typedef enum {
        FtdStatusHeaderFound,
        FtdStatusLookingForHeader,
        FtdStatusBufferFinished
    } FtdBufferAnalaysisStatus_t;

    /*************\
     *  Methods  *
    \*************/

    ErrorCodes_t initFtdiChannel(FT_HANDLE * handle, char channel);
    virtual void initializeDevice() = 0;
    void initializeLsbNoise(bool nullValues = true);

    void storeDataFrames(unsigned int framesNum);
    void stackOutgoingMessage(vector <uint8_t> &txDataMessage);

    inline void int322uint16(int32_t from, vector <uint16_t> &to, size_t offset);
    inline void uint322uint16(uint32_t from, vector <uint16_t> &to, size_t offset);

    void computeMinimumPacketNumber();
    void initializeRawDataFilterVariables();
    void applyFilter(uint16_t channelIdx, uint16_t &x);

    /****************\
     *  Parameters  *
    \****************/

    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;

    string upgradeNotes = "NONE";
    string notificationTag = "UNDEFINED";

    char rxChannel;
    char txChannel;

    uint32_t rxSyncWord;
    uint8_t txSyncWord;

    int packetsPerFrame = 16;

    uint16_t fsmStateChannelsNum = 1;
    uint16_t voltageChannelsNum = 2;
    uint16_t currentChannelsNum = 2;
    uint16_t filteredCurrentChannelsNum = currentChannelsNum;
    uint16_t averagedCurrentChannelsNum = currentChannelsNum;
    uint16_t totalChannelsNum = fsmStateChannelsNum+voltageChannelsNum+
            currentChannelsNum+filteredCurrentChannelsNum+averagedCurrentChannelsNum;

    uint32_t currentRangesNum;
    uint16_t selectedCurrentRangeIdx = 0;
    vector <RangedMeasurement_t> currentRangesArray;

    uint32_t voltageRangesNum;
    uint16_t selectedVoltageRangeIdx = 0;
    vector <RangedMeasurement_t> voltageRangesArray;
    vector <uint16_t> voltageOffsetArray;
    uint16_t voltageOffset;

    uint32_t samplingRatesNum;
    vector <Measurement_t> samplingRatesArray;
    unsigned int defaultSamplingRateIdx = 0;
    vector <Measurement_t> realSamplingRatesArray;
    vector <Measurement_t> integrationStepArray;
    BoolRandomArrayCoder * samplingRateCoder;

    BoolArrayCoder * deviceResetCoder;
    BoolArrayCoder * digitalOffsetCompensationCoder;
    vector <BoolArrayCoder *> zapCoders;
    vector <bool> zapStates;

    bool interposerInserted = false;
    BoolArrayCoder * conditioningModalityCoder;
    BoolArrayCoder * conditioningChannelCoder;
    uint16_t sequencingVoltageRangeIdx = 0;
    uint16_t conditioningVoltageRangeIdx = 0;
    uint16_t checkingVoltageRangeIdx = 0;
    uint16_t sequencingCurrentRangeIdx = 0;
    uint16_t conditioningCurrentRangeIdx = 0;
    uint16_t checkingCurrentRangeIdx = 0;

    RangedMeasurement_t dacExtRange;
    DoubleOffsetBinaryCoder * dacExtCoder;
    Measurement_t dacExtDefault;

    vector <std::string> protocolsNames;
    unsigned int triangularIdx;
    BoolArrayCoder * protocolsSelectCoder;
    BoolArrayCoder * protocolStartCoder;

    unsigned int protocolVoltagesNum;
    vector <std::string> protocolVoltageNames;
    vector <RangedMeasurement_t> protocolVoltageRanges;
    vector <DoubleSignAbsCoder *> protocolVoltageCoders;
    vector <Measurement_t> protocolVoltageDefault;

    RangedMeasurement_t triangularVoltageRange;
    DoubleOffsetBinaryCoder * triangularVoltageCoder;
    Measurement_t triangularVoltageDefault;

    unsigned int protocolTimesNum;
    vector <std::string> protocolTimeNames;
    RangedMeasurement_t protocolTimeRange;
    vector <DoubleOffsetBinaryCoder *> protocolTimeCoders;
    vector <Measurement_t> protocolTimeDefault;

    RangedMeasurement_t triangularTimeRange;
    DoubleOffsetBinaryCoder * triangularTimeCoder;
    Measurement_t triangularTimeDefault;

    unsigned int fsmFlagsNum;
    vector <std::string> fsmFlagNames;
    uint32_t fsmFlagStatus = 0x00000000;
    vector <BoolArrayCoder *> fsmFlagCoders;
    vector <bool> fsmFlagDefault;

    unsigned int fsmVoltagesNum;
    vector <std::string> fsmVoltageNames;
    RangedMeasurement_t fsmVoltageRange;
    vector <DoubleTwosCompCoder *> fsmVoltageCoders;
    vector <Measurement_t> fsmVoltageDefault;

    unsigned int fsmCurrentsNum;
    vector <std::string> fsmCurrentNames;
    RangedMeasurement_t fsmThresholdCurrentRange;
    vector <DoubleOffsetBinaryCoder *> fsmThresholdCurrentCoders;
    vector <Measurement_t> fsmThresholdCurrentDefault;

    unsigned int fsmTimesNum;
    vector <std::string> fsmTimeNames;
    RangedMeasurement_t fsmTimeRange;
    vector <DoubleOffsetBinaryCoder *> fsmTimeCoders;
    vector <Measurement_t> fsmTimeDefault;

    unsigned int fsmIntegersNum;
    vector <std::string> fsmIntegerNames;
    RangedMeasurement_t fsmIntegerRange;
    vector <BoolArrayCoder *> fsmIntegerCoders;
    vector <Measurement_t> fsmIntegerDefault;

    Measurement_t integrationStep;

    RangedMeasurement_t movingAverageFilterDurationRange;
    DoubleTwosCompCoder * movingAverageFilterKxCoder;
    Measurement_t movingAverageFilterDurationDefault;
    Measurement_t movingAverageFilterDuration;

    RangedMeasurement_t fourthOrderFilterCutoffFrequencyRange;
    DoubleTwosCompCoder * fourthOrderFilterA01Coder;
    DoubleTwosCompCoder * fourthOrderFilterA02Coder;
    DoubleTwosCompCoder * fourthOrderFilterK0Coder;
    DoubleTwosCompCoder * fourthOrderFilterA11Coder;
    DoubleTwosCompCoder * fourthOrderFilterA12Coder;
    DoubleTwosCompCoder * fourthOrderFilterK1Coder;
    Measurement_t fourthOrderFilterCutoffFrequencyDefault;
    Measurement_t fourthOrderFilterCutoffFrequency;

    RangedMeasurement_t rawDataFilterCutoffFrequencyRange;
    Measurement_t rawDataFilterCutoffFrequencyDefault;
    Measurement_t rawDataFilterCutoffFrequency = {30.0, UnitPfxKilo, "Hz"};
    double iir1Num[IIR_ORD+1];
    double iir1Den[IIR_ORD+1];
    double iir2Num[IIR_ORD+1];
    double iir2Den[IIR_ORD+1];

    double ** iirX;
    double ** iirU;
    double ** iirY;

    uint16_t iirOff = 0;

    unsigned int conditioningVoltagesNum;
    vector <std::string> conditioningProtocolVoltageNames;
    vector <RangedMeasurement_t> conditioningProtocolVoltageRanges;
    vector <DoubleOffsetBinaryCoder *> conditioningProtocolVoltageCoders;
    vector <Measurement_t> conditioningProtocolVoltageDefault;

    unsigned int checkingVoltagesNum;
    vector <std::string> checkingProtocolVoltageNames;
    vector <RangedMeasurement_t> checkingProtocolVoltageRanges;
    vector <DoubleSignAbsCoder *> checkingProtocolVoltageCoders;
    vector <Measurement_t> checkingProtocolVoltageDefault;

    unsigned int conditioningTimesNum;
    vector <std::string> conditioningProtocolTimeNames;
    vector <RangedMeasurement_t> conditioningProtocolTimeRanges;
    vector <DoubleOffsetBinaryCoder *> conditioningProtocolTimeCoders;
    vector <Measurement_t> conditioningProtocolTimeDefault;

    BoolArrayCoder * fEResetDenoiserCoder;
    BoolArrayCoder * dacIntFilterCoder;
    BoolArrayCoder * dacExtFilterCoder;
    BoolArrayCoder * vcIntCoder;
    BoolArrayCoder * vcmForceCoder;

    /***************\
     *  Variables  *
    \***************/

    string deviceId;

    FtdiEeprom * ftdiEeprom = nullptr;
    FT_HANDLE * ftdiRxHandle = nullptr;
    FT_HANDLE * ftdiTxHandle = nullptr;

    bool connected = false;
    bool threadsStarted = false;
    bool stopConnectionFlag = false;

    /*! Read data buffer management */
    mutable mutex readDataMtx;
    int readFrameLength;
    unsigned long minFrameNumber = FTD_DEFAULT_MIN_FRAME_NUMBER;
    unsigned int fewFramesSleep = FTD_DEFAULT_FEW_FRAME_SLEEP;
    unsigned char * readDataBuffer = nullptr; /*!< Buffer used in the read to store the data received from the device */
    unsigned int bufferReadOffset = 0; /*!< Device Rx buffer offset position in which data are collected by the outputDataBuffer */
    unsigned int bufferWriteOffset = 0; /*!< Device Rx buffer offset position in which data are written by FTDI device */
    unsigned int bytesLeftFromPreviousRead = 0; /*!< Accounts for bytes written from device Rx buffer but not yet written to BufferManager */
    bool bufferDataLossFlag = false; /*!< Set to true by a loss of data, reset by a status read by the user */
    bool exitOnSyncWord = false; /*!< Tells if the last buffer analysis returned with a syncword found */

    /*! Output data buffer management */
    uint16_t ** outputDataBuffer = nullptr; /*!< Buffer used to share received and converted data with the user */
    unsigned int outputBufferReadOffset = 0; /*!< outputDataBuffer offset position in which data are collected by the user */
    unsigned int outputBufferWriteOffset = 0; /*!< outputDataBuffer offset position in which data are converted from readDataBuffer */
    unsigned int outputBufferAvailablePackets = 0; /*!< Number of packets available for the user to read */
    bool outputBufferOverflowFlag = false; /*!< Set to true by an overflow of outputDataBuffer, reset by a status read by the user */

    /*! Write data buffer management */
    uint8_t * txRawBuffer; /*!< Raw outgoing data to the device */
    vector <uint8_t> * txMsgBuffer; /*!< Buffer of arrays of bytes to communicate to the device */
    uint32_t txMsgBufferWriteOffset = 0; /*!< Offset of the part of buffer to be written */
    uint32_t txMsgBufferReadLength = 0; /*!< Length of the part of the buffer to be processed */
    uint16_t txDataBytes;
    unsigned int maxOutputPacketsNum;
    vector <uint8_t> txStatus; /*!< Status of the bytes written */

    double * lsbNoiseArray;
    uint32_t lsbNoiseIdx = 0;

    double currentResolution = 1.0;
    double voltageResolution = 1.0;

    double voltageOffsetCorrected = 0.0; /*!< Value currently corrected in applied voltages by the device (expressed in the unit of the liquid junction control) */
    double voltageOffsetCorrection = 0.0; /*!< Value to be used to correct the measured votlage values (expressed in the unit of current voltage range) */

    RangedMeasurement_t voltageRange;
    RangedMeasurement_t currentRange;

    Measurement_t samplingRate = {200.0, UnitPfxKilo, "Hz"};

    double stepsOnLastSweep;
    uint16_t protocolItemsNum;
    uint16_t protocolItemIndex;

    CompensationControl_t liquidJunctionControl;
    double liquidJunctionResolution = 1.0;
    double liquidJunctionOffsetBinary = 0.0;

    uint16_t zapDurationHwRegisterOffset = 0;

    /********************************************\
     *  Multi-thread synchronization variables  *
    \********************************************/

    thread rxThread;
    thread txThread;

    condition_variable rxMsgBufferNotEmpty;
    condition_variable rxMsgBufferNotFull;

    mutable mutex txMutex;
    condition_variable txMsgBufferNotEmpty;
    condition_variable txMsgBufferNotFull;

    mutable mutex txAckMutex;

    condition_variable txAckCv;

#ifdef DEBUG_PRINT
    FILE * fid;
#endif
};

#endif // MESSAGEDISPATCHER_H
