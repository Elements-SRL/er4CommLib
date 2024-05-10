#ifndef MESSAGEDISPATCHER_H
#define MESSAGEDISPATCHER_H

#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <unordered_map>
#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include "er4commlib_errorcodes.h"
#include "er4commlib_global.h"
#include "ftdieeprom.h"
#include "ftdieeprom56.h"
#include "ftdieepromdemo.h"
#include "commandcoder.h"
#include "calibrationeeprom.h"

#ifdef DEBUG_PRINT
//#define DEBUG_RAW_BIT_RATE_PRINT
#endif

#define SHORT_OFFSET_BINARY (static_cast <double> (0x8000))
#define SHORT_MAX (static_cast <double> (0x7FFF))
#define SHORT_MIN (-SHORT_MAX-1.0)
#define USHORT_MAX (static_cast <double> (0xFFFF))
#define UINT10_MAX (static_cast <double> (0x3FF))
#define INT14_MAX (static_cast <double> (0x1FFF))
#define UINT14_MAX (static_cast <double> (0x3FFF))
#define INT18_MAX (static_cast <double> (0x1FFFF))
#define UINT28_MAX (static_cast <double> (0xFFFFFFF))
#define INT28_MAX (static_cast <double> (0x7FFFFFF))
#define INT28_MIN (-INT28_MAX-1.0)

#define UINT16_POSITIVE_SATURATION 0x7FFE
#define UINT16_NEGATIVE_SATURATION 0x8000 /*! \todo FCON dovrebbe essere 0x8001 */
#define UINT16_CURRENT_RANGE_INCREASE_MINIMUM_THRESHOLD 0x7800 /*!< Equivalent to 15/16 of range */
#define UINT16_CURRENT_RANGE_INCREASE_MAXIMUM_THRESHOLD 0x8800 /*!< Equivalent to -15/16 of range */
#define UINT16_CURRENT_RANGE_DECREASE_MINIMUM_THRESHOLD 0xF800 /*!< Equivalent to 1/16 of range */
#define UINT16_CURRENT_RANGE_DECREASE_MAXIMUM_THRESHOLD 0x0800 /*!< Equivalent to -1/16 of range */

#define LSB_NOISE_ARRAY_SIZE 0x40000 /*! \todo FCON valutare che questo numero sia adeguato */ // ~250k
#define LSB_NOISE_ARRAY_MASK (LSB_NOISE_ARRAY_SIZE-1) // 0b11...1 for all bits of the array indexes

#define IIR_ORD 2
#define IIR_2_SIN_PI_4 (2.0*sin(M_PI/4.0))
#define IIR_2_COS_PI_4 (2.0*cos(M_PI/4.0))
#define IIR_2_COS_PI_4_2 (IIR_2_COS_PI_4*IIR_2_COS_PI_4)

#define FTD_MAX_RESEND_TRIES 10
#define FTD_MAX_FPGA_RESET_TRIES 10
#define FTD_MAX_WRITE_TRIES 10

#define FTD_RX_WORD_SIZE (sizeof(uint16_t)) // 16 bit word
#define FTD_RX_SYNC_WORD_SIZE (static_cast <int> (2*FTD_RX_WORD_SIZE))
#define FTD_RX_INFO_WORD_SIZE (static_cast <int> (FTD_RX_WORD_SIZE))
#define FTD_RX_BUFFER_SIZE 0x100000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
#define FTD_RX_BUFFER_MASK (FTD_RX_BUFFER_SIZE-1)
#define FTD_DEFAULT_MIN_READ_FRAME_NUMBER 10
#define FTD_DEFAULT_MIN_STORE_FRAME_NUMBER 10
#define FTD_DEFAULT_FEW_FRAME_SLEEP 2000
#define FTD_FEW_PACKET_COEFF 0.05 /*!< = 50.0/1000.0: 50.0 because I want to get data once every 50ms, 1000 to convert sampling rate from Hz to kHz */
#define FTD_MAX_BYTES_TO_WAIT_FOR 2048 /*! Max FTDI buffer size = 4k, so wait no more than half full to read it */

#define ER4CL_OUTPUT_BUFFER_SIZE 0x400000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
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

class ER4COMMLIBSHARED_EXPORT MessageDispatcher {
public:
    typedef enum ConnectionStatus{
        Connected,
        Calibrating,
        Paused,
        Disconnected,
    } ConnectionStatus_t;
    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher(std::string deviceId);
    virtual ~MessageDispatcher();

    /************************\
     *  Connection methods  *
    \************************/

    static er4cl::ErrorCodes_t detectDevices(std::vector <std::string> &deviceIds);
    static er4cl::ErrorCodes_t connectDevice(std::string deviceId, MessageDispatcher * &messageDispatcher);
    virtual er4cl::ErrorCodes_t disconnectDevice();
    virtual er4cl::ErrorCodes_t pauseConnection(ConnectionStatus_t pauseFlag);
    void readDataFromDevice();
    void sendCommandsToDevice();

    /****************\
     *  Tx methods  *
    \****************/

    er4cl::ErrorCodes_t turnOnLsbNoise(bool flag);
    er4cl::ErrorCodes_t convertVoltageValue(uint16_t uintValue, double &fltValue);
    er4cl::ErrorCodes_t convertCurrentValue(uint16_t uintValue, uint16_t channelIdx, double &fltValue);
    er4cl::ErrorCodes_t convertGpValue(uint16_t uintValue, uint16_t channelIdx, double &fltValue);
    er4cl::ErrorCodes_t setVoltageRange(uint16_t voltageRangeIdx, bool applyFlag = true);
    er4cl::ErrorCodes_t setVoltageReferenceRange(uint16_t voltageRangeIdx, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t setGpRange(uint16_t gpRangeIdx, uint16_t channelIdx, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t setSamplingRate(uint16_t samplingRateIdx, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t setOversamplingRatio(uint16_t oversamplingRatioIdx, bool applyFlag = true);

    er4cl::ErrorCodes_t setVoltageStimulusLpf(uint16_t filterIdx, bool applyFlag = true);
    er4cl::ErrorCodes_t setVoltageReferenceLpf(uint16_t filterIdx, bool applyFlag = true);

    virtual er4cl::ErrorCodes_t selectStimulusChannel(uint16_t channelIdx, bool on, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t digitalOffsetCompensation(uint16_t channelIdx, bool on, bool applyFlag = true);
    virtual er4cl::ErrorCodes_t digitalOffsetCompensationAutostop(bool on, bool applyFlag = true);
    er4cl::ErrorCodes_t zap(uint16_t channelIdx, bool applyFlag = true);
    er4cl::ErrorCodes_t switchChannelOn(uint16_t channelIdx, bool on, bool applyFlag = true);
    er4cl::ErrorCodes_t switchVcSel0(bool on);
    er4cl::ErrorCodes_t switchVcSel1(bool on);

    er4cl::ErrorCodes_t turnOnDigitalOutput(bool on);
    er4cl::ErrorCodes_t turnLedOn(uint16_t ledIndex, bool on);
    er4cl::ErrorCodes_t enableFrontEndResetDenoiser(bool on);

    er4cl::ErrorCodes_t resetDevice();
    er4cl::ErrorCodes_t holdDeviceReset(bool flag);
    er4cl::ErrorCodes_t resetDigitalOffsetCompensation();
    er4cl::ErrorCodes_t resetCalib();
    er4cl::ErrorCodes_t resetDigitalOffsetCompensation(bool reset);

    er4cl::ErrorCodes_t sendCommands();
    er4cl::ErrorCodes_t selectVoltageProtocol(unsigned int idx, bool applyFlag = false);
    er4cl::ErrorCodes_t applyVoltageProtocol();
    virtual er4cl::ErrorCodes_t setProtocolVoltage(unsigned int idx, er4cl::Measurement_t voltage, bool applyFlag = false);
    er4cl::ErrorCodes_t setProtocolTime(unsigned int idx, er4cl::Measurement_t time, bool applyFlag = false);
    er4cl::ErrorCodes_t setProtocolSlope(unsigned int idx, er4cl::Measurement_t slope, bool applyFlag = false);
    er4cl::ErrorCodes_t setProtocolFrequency(unsigned int idx, er4cl::Measurement_t frequency, bool applyFlag = false);
    er4cl::ErrorCodes_t setProtocolAdimensional(unsigned int idx, er4cl::Measurement_t adimensional, bool applyFlag = false);
    er4cl::ErrorCodes_t checkSelectedProtocol(unsigned int idx, std::string &message);
    er4cl::ErrorCodes_t checkProtocolVoltage(unsigned int idx, er4cl::Measurement_t voltage, std::string &message);
    er4cl::ErrorCodes_t checkProtocolTime(unsigned int idx, er4cl::Measurement_t time, std::string &message);
    er4cl::ErrorCodes_t checkProtocolSlope(unsigned int idx, er4cl::Measurement_t slope, std::string &message);
    er4cl::ErrorCodes_t checkProtocolFrequency(unsigned int idx, er4cl::Measurement_t frequency, std::string &message);
    er4cl::ErrorCodes_t checkProtocolAdimensional(unsigned int idx, er4cl::Measurement_t adimensional, std::string &message);
    er4cl::ErrorCodes_t setVoltageOffset(unsigned int idx, er4cl::Measurement_t voltage, bool applyFlag = true);
    er4cl::ErrorCodes_t checkVoltageOffset(unsigned int idx, er4cl::Measurement_t voltage, std::string &message);
    er4cl::ErrorCodes_t applyInsertionPulse(er4cl::Measurement_t voltage, er4cl::Measurement_t duration);
    er4cl::ErrorCodes_t applyReferencePulse(er4cl::Measurement_t voltage, er4cl::Measurement_t duration);
    er4cl::ErrorCodes_t applyReferencePulseTrain(er4cl::Measurement_t voltage, er4cl::Measurement_t duration, er4cl::Measurement_t period, uint16_t number);
    er4cl::ErrorCodes_t overrideReferencePulse(bool flag, bool applyFlag = true);

    er4cl::ErrorCodes_t setRawDataFilter(er4cl::Measurement_t cutoffFrequency, bool lowPassFlag, bool activeFlag);
    er4cl::ErrorCodes_t applyDacExt(er4cl::Measurement_t voltage, bool applyFlag = true);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(unsigned int idx, er4cl::Measurement_t voltage, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave1Time(unsigned int idx, er4cl::Measurement_t time, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(unsigned int idx, er4cl::Measurement_t voltage, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave2Time(unsigned int idx, er4cl::Measurement_t time, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave2Duration(unsigned int idx, er4cl::Measurement_t time, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave2Period(unsigned int idx, er4cl::Measurement_t time, bool applyFlag = false);
    er4cl::ErrorCodes_t setFastReferencePulseProtocolWave2PulseNumber(unsigned int idx, uint16_t pulsesNumber, bool applyFlag = false);

    /*! Device specific controls */

    er4cl::ErrorCodes_t setCustomFlag(uint16_t idx, bool flag, bool applyFlag);
    er4cl::ErrorCodes_t setCustomDouble(uint16_t idx, double value, bool applyFlag);

    virtual er4cl::ErrorCodes_t resetWasherError();
    virtual er4cl::ErrorCodes_t setWasherPresetSpeeds(std::vector <int8_t> speedValues);
    virtual er4cl::ErrorCodes_t startWasher(uint16_t speedIdx);
    virtual er4cl::ErrorCodes_t updateWasherState();
    virtual er4cl::ErrorCodes_t updateWasherPresetSpeeds();

    er4cl::ErrorCodes_t setCompensationsChannel(uint16_t channelIdx);
    er4cl::ErrorCodes_t turnCFastCompensationOn(bool on);
    er4cl::ErrorCodes_t setCFastCompensationOptions(uint16_t optionIdx);
    er4cl::ErrorCodes_t setCFastCapacitance(er4cl::Measurement_t capacitance);

    er4cl::ErrorCodes_t setTtlPulseTrain(er4cl::Measurement_t pulseDuration, er4cl::Measurement_t pulseDelay, er4cl::Measurement_t period, unsigned int numberOfPulses);
    er4cl::ErrorCodes_t startTtlPulseTrain();

    er4cl::ErrorCodes_t setDebugBit(uint16_t byteOffset, uint16_t bitOffset, bool status);
    er4cl::ErrorCodes_t setDebugByte(uint16_t byteOffset, uint16_t byteValue);
    void setMaxOutputPacketsNum(unsigned int maxPackets);

    /****************\
     *  Rx methods  *
    \****************/

    er4cl::ErrorCodes_t isDeviceUpgradable(std::string &upgradeNotes, std::string &notificationTag);
    er4cl::ErrorCodes_t getDeviceInfo(uint8_t &deviceVersion, uint8_t &deviceSubversion, uint32_t &firmwareVersion);

    er4cl::ErrorCodes_t getQueueStatus(er4cl::QueueStatus_t &status);
    er4cl::ErrorCodes_t getDataPackets(uint16_t * &data, unsigned int packetsNumber, unsigned int &packetsRead);
    er4cl::ErrorCodes_t getAllDataPackets(uint16_t * &data, uint16_t * &unfilteredData, unsigned int packetsNumber, unsigned int &packetsRead);
    er4cl::ErrorCodes_t purgeData();

    er4cl::ErrorCodes_t getChannelsNumber(uint32_t &voltageChannelsNumber, uint32_t &currentChannelsNumber, uint32_t &gpChannelsNumber);

    er4cl::ErrorCodes_t getCurrentRanges(std::vector <er4cl::RangedMeasurement_t> &currentRanges, std::vector <uint16_t> &defaultOptions);
    er4cl::ErrorCodes_t getCurrentRange(er4cl::RangedMeasurement_t &currentRange, uint16_t channelIdx);
    er4cl::ErrorCodes_t hasIndependentCurrentRanges();

    er4cl::ErrorCodes_t getGpRanges(std::vector <std::vector <er4cl::RangedMeasurement_t>> &gpRanges, std::vector <uint16_t> &defaultOptions, std::vector <std::string> &names);
    er4cl::ErrorCodes_t getGpRange(er4cl::RangedMeasurement_t &gpRange, uint16_t channelIdx);

    er4cl::ErrorCodes_t getVoltageRanges(std::vector <er4cl::RangedMeasurement_t> &voltageRanges, uint16_t &defaultOption, std::vector <std::string> &extensions);
    er4cl::ErrorCodes_t getVoltageRange(er4cl::RangedMeasurement_t &voltageRange);
    virtual er4cl::ErrorCodes_t getVoltageReferenceRanges(std::vector <er4cl::RangedMeasurement_t> &ranges, uint16_t &defaultOption);
    er4cl::ErrorCodes_t getVoltageReferenceRange(er4cl::RangedMeasurement_t &range);

    er4cl::ErrorCodes_t getSamplingRates(std::vector <er4cl::Measurement_t> &samplingRates, uint16_t &defaultOption);
    er4cl::ErrorCodes_t getSamplingRate(er4cl::Measurement_t &samplingRate);
    er4cl::ErrorCodes_t getRealSamplingRates(std::vector <er4cl::Measurement_t> &samplingRates);
    er4cl::ErrorCodes_t getRealSamplingRate(er4cl::Measurement_t &samplingRate);
    er4cl::ErrorCodes_t getOversamplingRatios(std::vector <uint16_t> &oversamplingRatios);
    er4cl::ErrorCodes_t getOversamplingRatio(uint16_t &oversamplingRatio);

    er4cl::ErrorCodes_t getVoltageStimulusLpfs(std::vector <er4cl::Measurement_t> &filterOptions, uint16_t &defaultOption, int16_t &voltageRangeIdx);
    er4cl::ErrorCodes_t getVoltageReferenceLpfs(std::vector <er4cl::Measurement_t> &filterOptions, uint16_t &defaultOption, int16_t &voltageRangeIdx);

    er4cl::ErrorCodes_t hasSelectStimulusChannel(bool &selectStimulusChannelFlag, bool &singleChannelSSCFlag);
    er4cl::ErrorCodes_t hasDigitalOffsetCompensation(bool &digitalOffsetCompensationFlag, bool &singleChannelDOCFlag, bool &selectableDOCAutostopFlag);
    er4cl::ErrorCodes_t hasZap(bool &zappableDeviceFlag, bool &singleChannelZapFlag);
    er4cl::ErrorCodes_t hasChannelOn(bool &channelOnFlag, bool &singleChannelOnFlag);
    er4cl::ErrorCodes_t getSwitchedOnChannels(uint32_t &channelsMask);
    er4cl::ErrorCodes_t hasDigitalOffsetCompensationReset();

    er4cl::ErrorCodes_t hasDigitalOutput();
    er4cl::ErrorCodes_t hasFrontEndResetDenoiser();

    er4cl::ErrorCodes_t getLiquidJunctionControl(er4cl::CompensationControl_t &control);

    er4cl::ErrorCodes_t getProtocolList(std::vector <std::string> &names, std::vector <std::string> &images, std::vector <std::vector <uint16_t>> &voltages, std::vector <std::vector <uint16_t>> &times, std::vector <std::vector <uint16_t>> &slopes, std::vector <std::vector <uint16_t>> &frequencies, std::vector <std::vector <uint16_t>> &adimensionals);
    er4cl::ErrorCodes_t getTriangularProtocolIdx(uint16_t &idx);
    er4cl::ErrorCodes_t getSealTestProtocolIdx(uint16_t &idx);
    er4cl::ErrorCodes_t getProtocolVoltage(std::vector <std::string> &voltageNames, std::vector <er4cl::RangedMeasurement_t> &ranges, std::vector <er4cl::Measurement_t> &defaultValues);
    er4cl::ErrorCodes_t getProtocolTime(std::vector <std::string> &timeNames, std::vector <er4cl::RangedMeasurement_t> &ranges, std::vector <er4cl::Measurement_t> &defaultValues);
    er4cl::ErrorCodes_t getProtocolSlope(std::vector <std::string> &slopeNames, std::vector <er4cl::RangedMeasurement_t> &ranges, std::vector <er4cl::Measurement_t> &defaultValues);
    er4cl::ErrorCodes_t getProtocolFrequency(std::vector <std::string> &frequencyNames, std::vector <er4cl::RangedMeasurement_t> &ranges, std::vector <er4cl::Measurement_t> &defaultValues);
    er4cl::ErrorCodes_t getProtocolAdimensional(std::vector <std::string> &adimensionalNames, std::vector <er4cl::RangedMeasurement_t> &ranges, std::vector <er4cl::Measurement_t> &defaultValues);
    er4cl::ErrorCodes_t getVoltageOffsetControls(er4cl::RangedMeasurement_t &voltageRange);
    er4cl::ErrorCodes_t getInsertionPulseControls(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &durationRange);
    er4cl::ErrorCodes_t hasReferencePulseControls(bool &referencePulseImplemented, bool &overrideReferencePulseImplemented);
    er4cl::ErrorCodes_t getReferencePulseControls(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &durationRange);
    er4cl::ErrorCodes_t hasReferencePulseTrainControls(bool &referencePulseImplemented, bool &overrideReferencePulseImplemented);
    er4cl::ErrorCodes_t getReferencePulseTrainControls(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &durationRange, er4cl::RangedMeasurement_t &periodRange, uint16_t &pulsesNumber);
    er4cl::ErrorCodes_t getEdhFormat(std::string &format);
    er4cl::ErrorCodes_t getRawDataFilterCutoffFrequency(er4cl::RangedMeasurement_t &range, er4cl::Measurement_t &defaultValue);
    er4cl::ErrorCodes_t getLedsNumber(uint16_t &ledsNumber);
    er4cl::ErrorCodes_t getLedsColors(std::vector <uint32_t> &ledsColors);
    er4cl::ErrorCodes_t getFastReferencePulseProtocolWave1Range(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &timeRange, uint16_t &nPulse);
    er4cl::ErrorCodes_t getFastReferencePulseProtocolWave2Range(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &timeRange, er4cl::RangedMeasurement_t &durationRange, uint16_t &nPulse);
    er4cl::ErrorCodes_t getFastReferencePulseTrainProtocolWave2Range(er4cl::RangedMeasurement_t &voltageRange, er4cl::RangedMeasurement_t &timeRange, er4cl::RangedMeasurement_t &durationRange, er4cl::RangedMeasurement_t &waitTimeRange, uint16_t &pulsesPerTrain, uint16_t &nTrains);

    /*! Calibration methods */

    er4cl::ErrorCodes_t getCalibrationEepromSize(uint32_t &size);
    er4cl::ErrorCodes_t writeCalibrationEeprom(std::vector <uint32_t> value, std::vector <uint32_t> address, std::vector <uint32_t> size);
    er4cl::ErrorCodes_t readCalibrationEeprom(std::vector <uint32_t> &value, std::vector <uint32_t> address, std::vector <uint32_t> size);

    /*! Device specific controls */

    er4cl::ErrorCodes_t getCustomFlags(std::vector <std::string> &customFlags, std::vector <bool> &customFlagsDefault);
    er4cl::ErrorCodes_t getCustomDoubles(std::vector <std::string> &customDoubles, std::vector <er4cl::RangedMeasurement_t> &customDoublesRanges, std::vector <double> &customDoublesDefault);

    er4cl::ErrorCodes_t hasNanionTemperatureController();
    virtual er4cl::ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature);
    er4cl::ErrorCodes_t hasWasherControls();
    virtual er4cl::ErrorCodes_t getWasherSpeedRange(er4cl::RangedMeasurement_t &range);
    virtual er4cl::ErrorCodes_t getWasherStatus(er4cl::WasherStatus_t &status, er4cl::WasherError_t &error);
    virtual er4cl::ErrorCodes_t getWasherPresetSpeeds(std::vector <int8_t> &speedValue);

    er4cl::ErrorCodes_t hasCFastCompensation();
    er4cl::ErrorCodes_t getCFastCompensationOptions(std::vector <std::string> &options);
    er4cl::ErrorCodes_t getCFastCapacitanceControl(er4cl::CompensationControl_t &control);

    er4cl::ErrorCodes_t hasTtlPulseTrain();

    virtual er4cl::ErrorCodes_t updateVoltageOffsetCompensations(std::vector <er4cl::Measurement_t> &offsets);

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

    enum CompensationsTypes {
        CompensationCFast,
        CompensationsNum
    };
    /*************\
     *  Methods  *
    \*************/

    static er4cl::ErrorCodes_t getDeviceType(DeviceTuple_t tuple, er4cl::DeviceTypes_t &type);

    virtual er4cl::ErrorCodes_t connect(FtdiEeprom * ftdiEeprom);
    er4cl::ErrorCodes_t init();
    er4cl::ErrorCodes_t deinit();
    er4cl::ErrorCodes_t initFtdiChannel(FT_HANDLE * handle, char channel);
    virtual void initializeDevice();
    virtual bool checkProtocolValidity(std::string &message) = 0;

    void initializeLsbNoise(bool nullValues = true);
    void initializeCompensations();

    void processCurrentData(uint16_t channelIdx, uint16_t &x, uint16_t &unfilteredX);

    void initializeFerdMemory();
    virtual void setFerdParameters();
    double frontEndResetDenoise(uint16_t channelIdx, double x);

    virtual void storeDataFrames(unsigned int framesNum);
    void stackOutgoingMessage(std::vector <uint8_t> &txDataMessage);

    inline void int322uint16(int32_t from, std::vector <uint16_t> &to, size_t offset);
    inline void uint322uint16(uint32_t from, std::vector <uint16_t> &to, size_t offset);

    void computeMinimumPacketNumber();
    void initializeRawDataFilterVariables();
    void computeFilterCoefficients();
    double applyFilter(uint16_t channelIdx, double x);
    void manageVoltageReference();

    /****************\
     *  Parameters  *
    \****************/

    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
    er4cl::CalibrationEeprom * calEeprom = nullptr;

    std::string upgradeNotes = "NONE";
    std::string notificationTag = "UNDEFINED";

    char spiChannel = 'A';
    char rxChannel;
    char txChannel;

    uint32_t rxSyncWord;
    uint8_t txSyncWord;

    int packetsPerFrame = 16;

    uint16_t voltageChannelsNum = 1;
    uint16_t currentChannelsNum = 1;
    uint16_t gpChannelsNum = 0;
    uint16_t totalChannelsNum = voltageChannelsNum+currentChannelsNum+gpChannelsNum;

    uint32_t currentRangesNum;
    uint32_t selectedCurrentRangeIdx = 0;
    std::vector <uint16_t> selectedCurrentRangesIdx;
    std::vector <er4cl::RangedMeasurement_t> currentRangesArray;
    std::vector <uint16_t> defaultCurrentRangesIdx;
    std::vector <BoolRandomArrayCoder *> currentRangeCoders;

    uint32_t voltageRangesNum;
    uint16_t selectedVoltageRangeIdx = 0;
    std::vector <er4cl::RangedMeasurement_t> voltageRangesArray;
    std::vector <std::string> voltageRangesExtensions;
    uint16_t defaultVoltageRangeIdx = 0;
    BoolRandomArrayCoder * voltageRangeCoder;


    uint32_t voltageReferenceRangesNum = 0;
    uint16_t selectedVoltageReferenceRangeIdx = 0;
    std::vector <er4cl::RangedMeasurement_t> voltageReferenceRangesArray;
    uint16_t defaultVoltageReferenceRangeIdx = 0;
    BoolRandomArrayCoder * voltageReferenceRangeCoder;

    int16_t voltageRangeDivider = 1; /*! Divides the voltage data received from the amplifier */

    std::vector <uint32_t> gpRangesNum;
    std::vector <uint16_t> selectedGpRangesIdx;
    std::vector <std::vector <er4cl::RangedMeasurement_t>> gpRangesArray;
    std::vector <uint16_t> defaultGpRangesIdx;
    std::vector <BoolRandomArrayCoder *> gpRangeCoders;
    std::vector <std::string> gpNames;

    uint32_t samplingRatesNum;
    std::vector <er4cl::Measurement_t> samplingRatesArray;
    uint16_t defaultSamplingRateIdx = 0;
    std::vector <er4cl::Measurement_t> realSamplingRatesArray;
    std::vector <er4cl::Measurement_t> integrationStepArray;
    BoolRandomArrayCoder * samplingRateCoder;

    bool oversamplingImplemented = false;
    uint32_t oversamplingRatiosNum = 1;
    std::vector <uint16_t> oversamplingRatiosArray;
    BoolRandomArrayCoder * oversamplingRatioCoder;

    bool selectStimulusChannelFlag = false;
    bool singleChannelSSCFlag = false;
    bool digitalOffsetCompensationFlag = false;
    bool singleChannelDOCFlag = false;
    bool selectableDOCAutostopFlag = false;
    bool zappableDeviceFlag = false;
    bool singleChannelZapFlag = false;
    bool channelOnFlag = false;
    bool singleChannelOnFlag = false;

    bool resetCalibrationFlag = false;

    BoolArrayCoder * selectStimulusChannelCoder;
    std::vector <bool> selectStimulusChannelStates;

    BoolArrayCoder * deviceResetCoder;
    BoolArrayCoder * calibResetCoder;
    BoolArrayCoder * digitalOffsetCompensationCoder;
    BoolArrayCoder * digitalOffsetCompensationAutostopCoder;
    std::vector <bool> digitalOffsetCompensationStates;
    bool digitalOffsetCompensationResetFlag = false;
    BoolArrayCoder * digitalOffsetCompensationResetCoder;

    BoolArrayCoder * zapCoder;
    std::vector <bool> zapStates;

    BoolArrayCoder * channelOnCoder;
    std::vector <bool> channelOnStates;

    BoolArrayCoder * VcSel0Coder;
    BoolArrayCoder * VcSel1Coder;
    BoolArrayCoder * digOutCoder;

    std::vector <er4cl::RangedMeasurement_t> protocolVoltageRangesArray;
    std::vector <er4cl::RangedMeasurement_t> protocolTimeRangesArray;
    std::vector <er4cl::RangedMeasurement_t> protocolSlopeRangesArray;
    std::vector <er4cl::RangedMeasurement_t> protocolFrequencyRangesArray;

    std::vector <std::string> protocolsNames;
    std::vector <std::string> protocolsImages;
    std::vector <std::vector <uint16_t>> protocolsAvailableVoltages;
    std::vector <std::vector <uint16_t>> protocolsAvailableTimes;
    std::vector <std::vector <uint16_t>> protocolsAvailableSlopes;
    std::vector <std::vector <uint16_t>> protocolsAvailableFrequencies;
    std::vector <std::vector <uint16_t>> protocolsAvailableAdimensionals;
    BoolArrayCoder * protocolsSelectCoder;
    BoolArrayCoder * protocolStartCoder;
    uint16_t defaultProtocol;
    uint16_t selectedProtocol;
    uint16_t triangularProtocolIdx = 0;
    uint16_t sealTestProtocolIdx = 0;

    unsigned int protocolVoltagesNum;
    std::vector <std::string> protocolVoltageNames;
    std::vector <er4cl::RangedMeasurement_t> protocolVoltageRanges;
    std::vector <DoubleCoder *> protocolVoltageCoders;
    std::vector <er4cl::Measurement_t> protocolVoltageDefault;
    std::vector <er4cl::Measurement_t> selectedProtocolVoltage;

    unsigned int protocolTimesNum;
    std::vector <std::string> protocolTimeNames;
    std::vector <er4cl::RangedMeasurement_t> protocolTimeRanges;
    std::vector <DoubleCoder *> protocolTimeCoders;
    std::vector <er4cl::Measurement_t> protocolTimeDefault;
    std::vector <er4cl::Measurement_t> selectedProtocolTime;

    unsigned int protocolSlopesNum;
    std::vector <std::string> protocolSlopeNames;
    std::vector <er4cl::RangedMeasurement_t> protocolSlopeRanges;
    std::vector <DoubleCoder *> protocolSlopeCoders;
    std::vector <er4cl::Measurement_t> protocolSlopeDefault;
    std::vector <er4cl::Measurement_t> selectedProtocolSlope;

    unsigned int protocolFrequenciesNum;
    std::vector <std::string> protocolFrequencyNames;
    std::vector <er4cl::RangedMeasurement_t> protocolFrequencyRanges;
    std::vector <DoubleCoder *> protocolFrequencyCoders;
    std::vector <er4cl::Measurement_t> protocolFrequencyDefault;
    std::vector <er4cl::Measurement_t> selectedProtocolFrequency;

    unsigned int protocolAdimensionalsNum;
    std::vector <std::string> protocolAdimensionalNames;
    std::vector <er4cl::RangedMeasurement_t> protocolAdimensionalRanges;
    std::vector <DoubleCoder *> protocolAdimensionalCoders;
    std::vector <er4cl::Measurement_t> protocolAdimensionalDefault;
    std::vector <er4cl::Measurement_t> selectedProtocolAdimensional;

    bool ttlPulseTrainImplementedFlag = false;
    DoubleCoder * ttlPulseTrainDelayCoder = nullptr;
    DoubleCoder * ttlPulseTrainDurationCoder = nullptr;
    DoubleCoder * ttlPulseTrainPeriodCoder = nullptr;
    BoolArrayCoder * ttlPulseTrainPulsesNumberCoder = nullptr;
    BoolArrayCoder * ttlPulseTrainStartCoder = nullptr;

    std::vector <er4cl::Measurement_t> selectedVoltageOffset;
    er4cl::Measurement_t minSelectedVoltageOffset = {0, er4cl::UnitPfxMilli, "V"};
    er4cl::Measurement_t maxSelectedVoltageOffset = {0, er4cl::UnitPfxMilli, "V"};

    bool voltageOffsetControlImplemented = false;
    er4cl::RangedMeasurement_t voltageOffsetRange;
    std::vector <DoubleCoder *> voltageOffsetCoders;

    bool insertionPulseImplemented = false;
    er4cl::RangedMeasurement_t insertionPulseVoltageRange;
    er4cl::RangedMeasurement_t insertionPulseDurationRange;
    DoubleCoder * insertionPulseVoltageCoder;
    DoubleCoder * insertionPulseDurationCoder;
    BoolCoder * insertionPulseApplyCoder;

    bool referencePulseImplemented = false;
    bool referencePulseTrainImplemented = false;
    er4cl::RangedMeasurement_t referencePulseVoltageRange;
    er4cl::RangedMeasurement_t referencePulseDurationRange;
    er4cl::RangedMeasurement_t referencePulsePeriodRange;
    uint16_t referencePulseNumber;
    DoubleCoder * referencePulseVoltageCoder;
    DoubleCoder * referencePulseDurationCoder;
    DoubleCoder * referencePulseWaitTimeCoder;
    BoolArrayCoder * referencePulseNumberCoder;
    BoolCoder * referencePulseApplyCoder;

    bool overrideReferencePulseImplemented = false;
    BoolCoder * overrideReferencePulseApplyCoder;

    bool fastPulseProtocolImplementatedFlag = false;
    bool fastPulseTrainProtocolImplementatedFlag = false;
    uint16_t fastPulseW1num = 0;
    std::vector <DoubleCoder *> fastPulseW1VoltageCoder;
    std::vector <er4cl::Measurement_t> fastPulseW1Voltages;
    er4cl::RangedMeasurement_t fastPulseW1VoltageRange;
    std::vector <DoubleCoder *> fastPulseW1TimeCoder;
    std::vector <er4cl::Measurement_t> fastPulseW1Times;
    er4cl::RangedMeasurement_t fastPulseW1TimeRange;

    uint16_t fastPulseW2num = 0;
    std::vector <DoubleCoder *> fastPulseW2VoltageCoder;
    std::vector <er4cl::Measurement_t> fastPulseW2Voltages;
    er4cl::RangedMeasurement_t fastPulseW2VoltageRange;
    std::vector <DoubleCoder *> fastPulseW2TimeCoder;
    std::vector <er4cl::Measurement_t> fastPulseW2Times;
    er4cl::RangedMeasurement_t fastPulseW2TimeRange;
    std::vector <DoubleCoder *> fastPulseW2DurationCoder;
    std::vector <er4cl::Measurement_t> fastPulseW2Durations;
    er4cl::RangedMeasurement_t fastPulseW2DurationRange;
    std::vector <DoubleCoder *> fastPulseW2WaitTimeCoder;
    std::vector <er4cl::Measurement_t> fastPulseW2Periods;
    er4cl::RangedMeasurement_t fastPulseW2PeriodRange;
    std::vector <BoolArrayCoder *> fastPulseW2NumberCoder;
    std::vector <uint16_t> fastPulseW2PulsesNumbers;

    uint16_t customFlagsNum = 0;
    std::vector <std::string> customFlagsNames;
    std::vector <bool> customFlagsDefault;
    std::vector <BoolArrayCoder *> customFlagsCoders;

    uint16_t customDoublesNum = 0;
    std::vector <std::string> customDoublesNames;
    std::vector <er4cl::RangedMeasurement_t> customDoublesRanges;
    std::vector <double> customDoublesDefault;
    std::vector <DoubleCoder *> customDoublesCoders;

    std::string edhFormat;

    /*! Filter */
    er4cl::RangedMeasurement_t rawDataFilterCutoffFrequencyRange;
    er4cl::Measurement_t rawDataFilterCutoffFrequencyDefault;

    /*! Digital output */
    bool digOutImplementedFlag = false;

    /*! Front end denoiser */
    bool ferdImplementedFlag = false;
    unsigned int maxFerdSize = 1;

    bool dacIntFilterAvailable = false;
    std::vector <er4cl::Measurement_t> voltageStimulusLpfOptions;
    uint16_t voltageStimulusLpfOptionsNum = 0;
    uint16_t voltageStimulusLpfDefaultOption = 0;
    int16_t voltageStimulusLpfRange = -1;
    BoolRandomArrayCoder * dacIntFilterCoder;
    bool dacExtFilterAvailable = false;
    std::vector <er4cl::Measurement_t> voltageReferenceLpfOptions;
    uint16_t voltageReferenceLpfOptionsNum = 0;
    uint16_t voltageReferenceLpfDefaultOption = 0;
    int16_t voltageReferenceLpfRange = -1;
    BoolRandomArrayCoder * dacExtFilterCoder;

    uint16_t ledsNum = 0;
    std::vector <BoolArrayCoder *> ledsCoders;
    std::vector <uint32_t> ledsColorsArray;

    bool dacExtControllableFlag = false; /*! This is true if the voltage applied on the external DAC is directly controllable by the user, not through protocols */
    bool invertedDacExtFlag = false; /*! Negate the DAC value before applying it */
    std::vector <DoubleOffsetBinaryCoder *> dacExtCoders;
    er4cl::Measurement_t dacExtDefault = {0.0, er4cl::UnitPfxNone, "V"};
    er4cl::Measurement_t voltageReference = {0.0, er4cl::UnitPfxNone, "V"};
    int16_t voltageReferenceOffset = 0; /*! Value added to returned voltage data to accoutn for the voltage applied on the reference */

    /*! Device specific parameters */

    bool nanionTemperatureControllerFlag = false;
    bool washerControlFlag = false;

    std::vector <bool> compensationsEnabledArray[CompensationsNum]; /*! Compensations actually enabled on device */

    uint16_t compensationsSettingChannel = 0;

    std::vector <double> cFastCapacitance;
    std::vector <bool> cFastCompensationFlag;
    er4cl::CompensationControl_t cFastCompensationControl;
    std::vector <std::string> cFastCompensationOptions;
    std::vector <DoubleCoder *> cFastControlCoders;
    std::vector <BoolArrayCoder *> cFastOnCoders;

    BoolArrayCoder * bitDebugCoder = nullptr;
    BoolArrayCoder * byteDebugCoder = nullptr;

    /***************\
     *  Variables  *
    \***************/

    std::string deviceId;

    FtdiEeprom * ftdiEeprom = nullptr;
    FT_HANDLE * ftdiRxHandle = nullptr;
    FT_HANDLE * ftdiTxHandle = nullptr;

    bool connected = false;
    ConnectionStatus_t connectionStatus = ConnectionStatus_t::Disconnected;
    bool threadsStarted = false;
    bool stopConnectionFlag = false;

    /*! Read data buffer management */
    mutable std::mutex readDataMtx;
    int readFrameLength;
    uint8_t nullInfoStruct;
    uint8_t * infoStructPtr = &nullInfoStruct;
    unsigned int infoStructSize = 1;
    unsigned long minReadFrameNumber = FTD_DEFAULT_MIN_READ_FRAME_NUMBER; /*!< Minimum number of frames before they are read from the FTDI driver buffer */
    unsigned long minStoreFrameNumber = FTD_DEFAULT_MIN_STORE_FRAME_NUMBER; /*!< Minimum number of frames before they are stored in the commlib output buffer */
    unsigned int fewFramesSleep = FTD_DEFAULT_FEW_FRAME_SLEEP;
    unsigned char * readDataBuffer = nullptr; /*!< Buffer used in the read to store the data received from the device */
    unsigned int bufferReadOffset = 0; /*!< Device Rx buffer offset position in which data are collected by the outputDataBuffer */
    unsigned int bufferWriteOffset = 0; /*!< Device Rx buffer offset position in which data are written by FTDI device */
    unsigned int bytesReadFromDriver = 0; /*!< Accounts for bytes written from device Rx buffer but not yet written to the output buffer */
    bool bufferDataLossFlag = false; /*!< Set to true by a loss of data, reset by a status read by the user */
    bool exitOnSyncWord = false; /*!< Tells if the last buffer analysis returned with a syncword found */

    /*! Output data buffer management */
    uint16_t * outputDataArray = nullptr; /*! Array used to return data via getDataPackets method */
    uint16_t * outputUnfilteredDataArray = nullptr; /*! Array used to return unfiltered data via getAllDataPackets method */
    uint16_t ** outputDataBuffer = nullptr; /*!< Buffer used to share received and converted data with the user */
    uint16_t ** outputUnfiteredDataBuffer = nullptr; /*!< Buffer used to share unfiltered data with the user */
    unsigned int outputBufferReadOffset = 0; /*!< outputDataBuffer offset position in which data are collected by the user */
    unsigned int outputBufferWriteOffset = 0; /*!< outputDataBuffer offset position in which data are converted from readDataBuffer */
    unsigned int outputBufferAvailablePackets = 0; /*!< Number of packets available for the user to read */
    bool outputBufferOverflowFlag = false; /*!< Set to true by an overflow of outputDataBuffer, reset by a status read by the user */
    bool bufferSaturationFlag = false; /*!< Set to true by data saturating the front end range */
    bool bufferIncreaseCurrentRangeFlag = false; /*!< Set to true by data indicating that the front end range might be increased */
    bool bufferDecreaseCurrentRangeFlag = false; /*!< Set to true by data indicating that the front end range might be decreased */
    bool deviceCommunicationErrorFlag = false; /*!< Set to true by failures in communication with the device */

    /*! Write data buffer management */
    uint8_t * txRawBuffer; /*!< Raw outgoing data to the device */
    std::vector <uint8_t> * txMsgBuffer; /*!< Buffer of arrays of bytes to communicate to the device */
    uint32_t txMsgBufferWriteOffset = 0; /*!< Offset of the part of buffer to be written */
    uint32_t txMsgBufferReadLength = 0; /*!< Length of the part of the buffer to be processed */
    uint16_t txDataBytes;
    unsigned int maxOutputPacketsNum;
    std::vector <uint8_t> txStatus; /*!< Status of the bytes written */

    double * lsbNoiseArray;
    uint32_t lsbNoiseIdx = 0;

    /*! Front end configuration */
    std::vector <double> currentResolutions;
    double voltageResolution = 1.0;
    std::vector <double> gpResolutions;
    std::vector <double> gpOffsets;

    double voltageOffsetCorrected = 0.0; /*!< Value currently corrected in applied voltages by the device (expressed in the unit of the liquid junction control) */
    double voltageOffsetCorrection = 0.0; /*!< Value to be used to correct the measured votlage values (expressed in the unit of current voltage range) */

    er4cl::Measurement_t voltageOffsetCompensationGain = {1.0, er4cl::UnitPfxNone, "V"};

    bool independentCurrentRangesFlag = false;
    er4cl::RangedMeasurement_t voltageRange;
    er4cl::RangedMeasurement_t voltageReferenceRange;
    std::vector <er4cl::RangedMeasurement_t> currentRanges;
    std::vector <er4cl::RangedMeasurement_t> gpRanges;

    uint16_t selectedSamplingRateIdx = 0;
    er4cl::Measurement_t baseSamplingRate = {1.0, er4cl::UnitPfxKilo, "Hz"};
    er4cl::Measurement_t samplingRate = {1.0, er4cl::UnitPfxKilo, "Hz"}; /*!< baseSamplingRate*oversamplingRatio */
    er4cl::Measurement_t integrationStep = {1.0, er4cl::UnitPfxMilli, "Hz"};

    uint16_t selectedOversamplingRatioIdx = 0;
    uint16_t oversamplingRatio = 1;

    /*! Filter */
    bool rawDataFilterLowPassFlag = true;
    bool rawDataFilterActiveFlag = false;
    er4cl::Measurement_t rawDataFilterCutoffFrequency = {30.0, er4cl::UnitPfxKilo, "Hz"};
    double iirNum[IIR_ORD+1];
    double iirDen[IIR_ORD+1];

    double ** iirX = nullptr;
    double ** iirY = nullptr;

    uint16_t iirOff = 0;

    /*! Front end denoiser */
    bool ferdFlag = false;
    bool ferdInhibition = true;

    unsigned int ferdL = 1; /*!< buffer length */
    double ferdD = 1.0; /*!< inverse of buffer length */
    unsigned int ferdIdx = 0;
    double ferdK; /*!< first order filter coefficient */

    std::vector <std::vector <double>> ferdY;
    std::vector <double> ferdY0;
    std::vector <double> ferdM;

    er4cl::CompensationControl_t liquidJunctionControl;
    double liquidJunctionResolution = 1.0;
    double liquidJunctionOffsetBinary = 0.0;

    /********************************************\
     *  Multi-thread synchronization variables  *
    \********************************************/

    std::thread rxThread;
    std::thread txThread;

    mutable std::mutex connectionMutex;

    std::condition_variable rxMsgBufferNotEmpty;
    std::condition_variable rxMsgBufferNotFull;

    mutable std::mutex txMutex;
    std::condition_variable txMsgBufferNotEmpty;
    std::condition_variable txMsgBufferNotFull;

#ifdef DEBUG_PRINT
    FILE * fid;
#endif
};

class MessageDispatcherLegacyEdr3 : public MessageDispatcher {
public:
    MessageDispatcherLegacyEdr3(std::string deviceId);
    virtual ~MessageDispatcherLegacyEdr3();

protected:
    virtual void storeDataFrames(unsigned int framesNum) override;

    uint16_t rawVoltageZero = 0;
};

#endif // MESSAGEDISPATCHER_H
