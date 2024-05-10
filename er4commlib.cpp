#define MASS_CALL0(func) {\
    if (msgDisps.empty()) {\
        return ErrorDeviceNotConnected;\
    }\
    ErrorCodes_t err;\
    for (auto md : msgDisps) {\
        if ((err = md->func()) != Success) {\
            return err;\
        }\
    }\
    return Success;\
}

#define MASS_CALL1(func, arg) {\
    if (msgDisps.empty()) {\
        return ErrorDeviceNotConnected;\
    }\
    ErrorCodes_t err;\
    for (auto md : msgDisps) {\
        if ((err = md->func(arg)) != Success) {\
            return err;\
        }\
    }\
    return Success;\
}

#define MASS_CALL2(func, arg1, arg2) {\
    if (msgDisps.empty()) {\
        return ErrorDeviceNotConnected;\
    }\
    ErrorCodes_t err;\
    for (auto md : msgDisps) {\
        if ((err = md->func(arg1, arg2)) != Success) {\
            return err;\
        }\
    }\
    return Success;\
}

#define MASS_CALL3(func, arg1, arg2, arg3) {\
    if (msgDisps.empty()) {\
        return ErrorDeviceNotConnected;\
    }\
    ErrorCodes_t err;\
    for (auto md : msgDisps) {\
        if ((err = md->func(arg1, arg2, arg3)) != Success) {\
            return err;\
        }\
    }\
    return Success;\
}

#define MASS_CALL4(func, arg1, arg2, arg3, arg4) {\
    if (msgDisps.empty()) {\
        return ErrorDeviceNotConnected;\
    }\
    ErrorCodes_t err;\
    for (auto md : msgDisps) {\
        if ((err = md->func(arg1, arg2, arg3, arg4)) != Success) {\
            return err;\
        }\
    }\
    return Success;\
}

#define CALL_FIRST0(func) {\
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->func();\
}

#define CALL_FIRST1(func, arg) {\
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->func(arg);\
}

#define CALL_FIRST2(func, arg1, arg2) {\
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->func(arg1, arg2);\
}

#define CALL_FIRST3(func, arg1, arg2, arg3) {\
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->func(arg1, arg2, arg3);\
}

/*! Samples discard parameters */
#define SD_PID_KP (1.0e-5)
#define SD_PID_KI (5.0e-10)
#define SD_KF_STATE_ERROR (1.0e-9)
#define SD_KF_MEASUREMENT_ERROR (1.0e-3)

#include "er4commlib.h"

#include <algorithm>

#include "messagedispatcher.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include "ftd2xx_win.h"

static std::vector <MessageDispatcher *> msgDisps;
static unsigned int voltageChannelsNum = 0;
static unsigned int currentChannelsNum = 0;
static unsigned int gpChannelsNum = 0;
static unsigned int totalChannelsNum = 0;
static unsigned int msgDispsNum = 0;
static unsigned int totalCurrentChannelsNum = 0;
static unsigned int totalTotalChannelsNum = 0;
static unsigned int msgDispCompensated = 0;
static uint16_t * bufferOut = nullptr;
static uint16_t * unfilteredBufferOut = nullptr;
/*! Samples discard variables */
static std::vector <double> sdPidError;
static std::vector <double> sdPidIntegralError;
static std::vector <double> sdKfStateVariance;
static std::vector <double> sdKfInnovation;
static std::vector <double> sdKfStateEstimate;
static std::vector <double> sdKfMeasurementVariance;
static std::vector <uint32_t> sdTotalReadSamples;
static std::vector <uint32_t> availableSamples;
static std::vector <uint32_t> sdSamplesToDiscard;

static void allocateSampleDiscardVariables();
static void resetSampleDiscardVariables();

//static FILE * fid = nullptr;

namespace er4CommLib {

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t detectDevices(
        std::vector <std::string> &deviceIds) {
    /*! Gets number of devices */
    return MessageDispatcher::detectDevices(deviceIds);
}

ErrorCodes_t connect(
        std::vector <std::string> deviceIds) {

    if (deviceIds.empty()) {
        return ErrorUnknown; /*! \todo FCON sostituire */
    }

    if (!msgDisps.empty()) {
        return ErrorDeviceAlreadyConnected;
    }

    uint8_t deviceVersion;
    uint8_t deviceSubversion;
    uint32_t firmwareVersion;

    uint8_t deviceVersionCheck;
    uint8_t deviceSubversionCheck;
    uint32_t firmwareVersionCheck;

    getDeviceInfo(deviceIds[0],
                  deviceVersion,
                  deviceSubversion,
                  firmwareVersion);

    for (auto&& deviceId : deviceIds) {
        getDeviceInfo(deviceId,
                      deviceVersionCheck,
                      deviceSubversionCheck,
                      firmwareVersionCheck);
        if (deviceVersion != deviceVersionCheck || deviceSubversion != deviceSubversionCheck || firmwareVersion != firmwareVersionCheck) {
            return ErrorConnectingDifferentDevices;
        }
    }

    msgDisps.resize(deviceIds.size());
    ErrorCodes_t err = Success;
    int c = 0;
    for (auto&& deviceId : deviceIds) {
        err = MessageDispatcher::connectDevice(deviceId, msgDisps[c++]);
        if (err != Success) {
            msgDisps.clear();
            return err;
        }
    }

    msgDispsNum = msgDisps.size();

    msgDisps[0]->getChannelsNumber(voltageChannelsNum, currentChannelsNum, gpChannelsNum);
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;
    totalCurrentChannelsNum = currentChannelsNum*msgDispsNum;
    totalTotalChannelsNum = voltageChannelsNum+totalCurrentChannelsNum;

    for (auto md : msgDisps) {
        md->setMaxOutputPacketsNum(ER4CL_DATA_ARRAY_SIZE/(totalCurrentChannelsNum+voltageChannelsNum));
    }
    bufferOut = new (std::nothrow) uint16_t[ER4CL_DATA_ARRAY_SIZE];
    unfilteredBufferOut = new (std::nothrow) uint16_t[ER4CL_DATA_ARRAY_SIZE];

    /*! Synchronization stuff */
    allocateSampleDiscardVariables();
    resetSampleDiscardVariables();

//    fid = fopen("pippo.dat", "wb+");
    return err;
}

ErrorCodes_t disconnect() {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    ErrorCodes_t err = Success;
    int c = 0;
    for (auto md : msgDisps) {
        err = md->disconnectDevice();
        delete msgDisps[c++];
    }
    msgDisps.clear();
    msgDispsNum = 0;

    if (bufferOut != nullptr) {
        delete [] bufferOut;
        bufferOut = nullptr;
    }

    if (unfilteredBufferOut != nullptr) {
        delete [] unfilteredBufferOut;
        unfilteredBufferOut = nullptr;
    }

//    if (fid != nullptr) {
//        fclose(fid);
//        fid = nullptr;
//    }

    return err;
}

/****************\
 *  Tx methods  *
\****************/

ErrorCodes_t sendCommands() {
    MASS_CALL0(sendCommands)
}

ErrorCodes_t selectVoltageProtocol(
        unsigned int idx) {
    MASS_CALL1(selectVoltageProtocol, idx)
}

ErrorCodes_t applyVoltageProtocol() {
    MASS_CALL0(applyVoltageProtocol)
}

ErrorCodes_t setProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage) {
    MASS_CALL2(setProtocolVoltage, idx, voltage)
}

ErrorCodes_t setProtocolTime(
        unsigned int idx,
        Measurement_t time) {
    MASS_CALL2(setProtocolTime, idx, time)
}

ErrorCodes_t setProtocolSlope(
        unsigned int idx,
        Measurement_t slope) {
    MASS_CALL2(setProtocolSlope, idx, slope)
}

ErrorCodes_t setProtocolFrequency(
        unsigned int idx,
        Measurement_t frequency) {
    MASS_CALL2(setProtocolFrequency, idx, frequency)
}

ErrorCodes_t setProtocolAdimensional(
        unsigned int idx,
        Measurement_t adimensional) {
    MASS_CALL2(setProtocolAdimensional, idx, adimensional)
}

ErrorCodes_t checkSelectedProtocol(
        unsigned int idx,
        std::string &message) {
    MASS_CALL2(checkSelectedProtocol, idx, message)
}

ErrorCodes_t checkProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage,
        std::string &message) {
    MASS_CALL3(checkProtocolVoltage, idx, voltage, message)
}

ErrorCodes_t checkProtocolTime(
        unsigned int idx,
        Measurement_t time,
        std::string &message) {
    MASS_CALL3(checkProtocolTime, idx, time, message)
}

ErrorCodes_t checkProtocolSlope(
        unsigned int idx,
        Measurement_t slope,
        std::string &message) {
    MASS_CALL3(checkProtocolSlope, idx, slope, message)
}

ErrorCodes_t checkProtocolFrequency(
        unsigned int idx,
        Measurement_t frequency,
        std::string &message) {
    MASS_CALL3(checkProtocolFrequency, idx, frequency, message)
}

ErrorCodes_t checkProtocolAdimensional(
        unsigned int idx,
        Measurement_t adimensional,
        std::string &message) {
    MASS_CALL3(checkProtocolAdimensional, idx, adimensional, message)
}

ErrorCodes_t setVoltageOffset(
        unsigned int channelIdx,
        Measurement_t voltage) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->setVoltageOffset(currentChannelsNum, voltage);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->setVoltageOffset(channelIdx % currentChannelsNum, voltage);
    }
    return ret;
}

ErrorCodes_t checkVoltageOffset(
        unsigned int channelIdx,
        Measurement_t voltage,
        std::string &message) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx >= totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    ret = msgDisps[channelIdx/currentChannelsNum]->checkVoltageOffset(channelIdx % currentChannelsNum, voltage, message);
    return ret;
}

ErrorCodes_t applyInsertionPulse(
        Measurement_t voltage,
        Measurement_t duration) {
    MASS_CALL2(applyInsertionPulse, voltage, duration)
}

ErrorCodes_t applyReferencePulse(
        Measurement_t voltage,
        Measurement_t duration) {
    MASS_CALL2(applyReferencePulse, voltage, duration)
}

ErrorCodes_t applyReferencePulseTrain(
        Measurement_t voltage,
        Measurement_t duration,
        Measurement_t period,
        uint16_t number) {
    MASS_CALL4(applyReferencePulseTrain, voltage, duration, period, number)
}

ErrorCodes_t overrideReferencePulse(
      bool flag) {
    MASS_CALL1(overrideReferencePulse, flag)
}

ErrorCodes_t setRawDataFilter(
        Measurement_t cutoffFrequency,
        bool lowPassFlag,
        bool activeFlag) {
    MASS_CALL3(setRawDataFilter, cutoffFrequency, lowPassFlag, activeFlag)
}

ErrorCodes_t applyDacExt(
        Measurement_t voltage) {
    MASS_CALL1(applyDacExt, voltage)
}

ErrorCodes_t setCustomFlag(
        uint16_t idx,
        bool flag) {
    MASS_CALL3(setCustomFlag, idx, flag, true)
}

ErrorCodes_t setCustomDouble(
        uint16_t idx,
        double value) {
    MASS_CALL3(setCustomDouble, idx, value, true)
}

ErrorCodes_t resetWasherError() {
    MASS_CALL0(resetWasherError)
}

ErrorCodes_t setWasherPresetSpeeds(
        std::vector <int8_t> speedValues) {
    MASS_CALL1(setWasherPresetSpeeds, speedValues)
}

ErrorCodes_t startWasher(
        uint16_t speedIdx) {
    MASS_CALL1(startWasher, speedIdx)
}

ErrorCodes_t updateWasherState() {
    MASS_CALL0(updateWasherState)
}

ErrorCodes_t updateWasherPresetSpeeds() {
    MASS_CALL0(updateWasherPresetSpeeds)
}

ErrorCodes_t setCurrentRange(
        uint16_t currentRangeIdx,
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->setCurrentRange(currentRangeIdx, currentChannelsNum);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->setCurrentRange(currentRangeIdx, channelIdx % currentChannelsNum);
    }
    return ret;
}

ErrorCodes_t setVoltageRange(
        uint16_t voltageRangeIdx) {
    MASS_CALL1(setVoltageRange, voltageRangeIdx)
}

ErrorCodes_t setVoltageReferenceRange(
        uint16_t voltageRangeIdx) {
    MASS_CALL1(setVoltageReferenceRange, voltageRangeIdx)
}

ErrorCodes_t setGpRange(
        ER4CL_ARGIN uint16_t gpRangeIdx,
        ER4CL_ARGIN uint16_t channelIdx) {
    MASS_CALL2(setGpRange, gpRangeIdx, channelIdx)
}

ErrorCodes_t setSamplingRate(
        uint16_t samplingRateIdx) {
    MASS_CALL1(setSamplingRate, samplingRateIdx)
}

ErrorCodes_t setOversamplingRatio(
        uint16_t oversamplingRatioIdx) {
    MASS_CALL1(setOversamplingRatio, oversamplingRatioIdx)
}

ErrorCodes_t setVoltageStimulusLpf(
        uint16_t filterIdx) {
    MASS_CALL1(setVoltageStimulusLpf, filterIdx)
}

ErrorCodes_t setVoltageReferenceLpf(
        uint16_t filterIdx) {
    MASS_CALL1(setVoltageReferenceLpf, filterIdx)
}

ErrorCodes_t selectStimulusChannel(
        uint16_t channelIdx,
        bool on) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->selectStimulusChannel(currentChannelsNum, on);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->selectStimulusChannel(channelIdx % currentChannelsNum, on);
    }
    return ret;
}

ErrorCodes_t digitalOffsetCompensation(
        uint16_t channelIdx,
        bool on) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->digitalOffsetCompensation(currentChannelsNum, on);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->digitalOffsetCompensation(channelIdx % currentChannelsNum, on);
    }
    return ret;
}

ErrorCodes_t digitalOffsetCompensationAutostop(
        bool on) {
    MASS_CALL1(digitalOffsetCompensationAutostop, on)
}

ErrorCodes_t zap(
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->zap(currentChannelsNum);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->zap(channelIdx % currentChannelsNum);
    }
    return ret;
}

ErrorCodes_t switchChannelOn(
        uint16_t channelIdx,
        bool on) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    if (channelIdx == totalCurrentChannelsNum) {
        for (auto md : msgDisps) {
            ret = md->switchChannelOn(currentChannelsNum, on);
        }

    } else {
        ret = msgDisps[channelIdx/currentChannelsNum]->switchChannelOn(channelIdx % currentChannelsNum, on);
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(
        uint32_t idx,
        Measurement_t voltage) {
    MASS_CALL2(setFastReferencePulseProtocolWave1Voltage, idx, voltage)
}

ErrorCodes_t setFastReferencePulseProtocolWave1Time(
        uint32_t idx,
        Measurement_t time) {
    MASS_CALL2(setFastReferencePulseProtocolWave1Time, idx, time)
}

ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(
        uint32_t idx,
        Measurement_t voltage) {
    MASS_CALL2(setFastReferencePulseProtocolWave2Voltage, idx, voltage)
}

ErrorCodes_t setFastReferencePulseProtocolWave2Time(
        uint32_t idx,
        Measurement_t time) {
    MASS_CALL2(setFastReferencePulseProtocolWave2Time, idx, time)
}

ErrorCodes_t setFastReferencePulseProtocolWave2Duration(
        uint32_t idx,
        Measurement_t time) {
    MASS_CALL2(setFastReferencePulseProtocolWave2Duration, idx, time)
}

ErrorCodes_t setFastReferencePulseProtocolWave2Period(
        uint32_t idx,
        Measurement_t time) {
    MASS_CALL2(setFastReferencePulseProtocolWave2Period, idx, time)
}

ErrorCodes_t setFastReferencePulseProtocolWave2PulseNumber(
        uint32_t idx,
        uint16_t pulseNumber) {
    MASS_CALL2(setFastReferencePulseProtocolWave2PulseNumber, idx, pulseNumber)
}

ErrorCodes_t turnOnDigitalOutput(
        bool on) {
    MASS_CALL1(turnOnDigitalOutput, on)
}

ErrorCodes_t turnLedOn(
        uint16_t ledIndex,
        bool on) {
    MASS_CALL2(turnLedOn, ledIndex, on)
}

ErrorCodes_t enableFrontEndResetDenoiser(
        bool on) {
    MASS_CALL1(enableFrontEndResetDenoiser, on)
}

ErrorCodes_t resetDevice() {
    MASS_CALL0(resetDevice)
}

ErrorCodes_t resetSynchronizationVariables() {
    resetSampleDiscardVariables();
    return Success;
}

ErrorCodes_t holdDeviceReset(
        bool flag) {
    MASS_CALL1(holdDeviceReset, flag)
}

ErrorCodes_t resetDigitalOffsetCompensation() {
    MASS_CALL0(resetDigitalOffsetCompensation)
}

ErrorCodes_t setCompensationsChannel(
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx >= totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    ErrorCodes_t ret = Success;
    msgDispCompensated = channelIdx/currentChannelsNum;

    ret = msgDisps[msgDispCompensated]->setCompensationsChannel(channelIdx % currentChannelsNum);
    return ret;
}

ErrorCodes_t turnCFastCompensationOn(
        bool on) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = Success;
    ret = msgDisps[msgDispCompensated]->turnCFastCompensationOn(on);

    return ret;
}

ErrorCodes_t setCFastCompensationOptions(
        uint16_t optionIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = Success;
    ret = msgDisps[msgDispCompensated]->setCFastCompensationOptions(optionIdx);

    return ret;
}

ErrorCodes_t setCFastCapacitance(
        Measurement_t value) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = Success;
    ret = msgDisps[msgDispCompensated]->setCFastCapacitance(value);

    return ret;
}

ErrorCodes_t setTtlPulseTrain(Measurement_t pulseDuration, Measurement_t pulseDelay, Measurement_t period, unsigned int numberOfPulses) {
    MASS_CALL4(setTtlPulseTrain, pulseDuration, pulseDelay, period, numberOfPulses)
}

ErrorCodes_t startTtlPulseTrain() {
    MASS_CALL0(startTtlPulseTrain)
}

ErrorCodes_t setDebugBit(
        uint16_t byteOffset,
        uint16_t bitOffset,
        bool status) {
    MASS_CALL3(setDebugBit, byteOffset, bitOffset, status)
}

ErrorCodes_t setDebugByte(
        uint16_t byteOffset,
        uint16_t byteValue) {
    MASS_CALL2(setDebugByte, byteOffset, byteValue)
}

/****************\
 *  Rx methods  *
\****************/

//ErrorCodes_t CommLib::isDeviceUpgradable(
//        string &upgradeNotes,
//        string &notificationTag) {
//    ErrorCodes_t ret = Success;
//    if (messageDispatcher != nullptr) {
//        ret = messageDispatcher->isDeviceUpgradable(upgradeNotes, notificationTag);

//    } else {
//        ret = ErrorDeviceNotConnected;
//    }
//    return ret;
//}

ErrorCodes_t getDeviceInfo(
        uint8_t &deviceVersion,
        uint8_t &deviceSubversion,
        uint32_t &firmwareVersion) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    return msgDisps[0]->getDeviceInfo(deviceVersion, deviceSubversion, firmwareVersion);
}

ErrorCodes_t getDeviceInfo(
        std::string deviceId,
        uint8_t &deviceVersion,
        uint8_t &deviceSubversion,
        uint32_t &firmwareVersion) {
    ErrorCodes_t ret = Success;
    /*! Initializes eeprom */
    /*! \todo FCON questa info dovrà essere appresa dal device detector e condivisa qui dal metodo connect */
    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
    if (deviceId == "ePatch Demo") {
        ftdiEepromId = FtdiEepromIdDemo;
    }

    /*! ftdiEeprom is deleted by the messageDispatcher if one is created successfully */
    FtdiEeprom * ftdiEeprom = nullptr;
    switch (ftdiEepromId) {
    case FtdiEepromId56:
        ftdiEeprom = new FtdiEeprom56(deviceId);
        break;

    case FtdiEepromIdDemo:
        ftdiEeprom = new FtdiEepromDemo(deviceId);
        break;
    }

    if (ftdiEeprom != nullptr) {
        DeviceTuple_t deviceTuple = ftdiEeprom->getDeviceTuple();

        deviceVersion = deviceTuple.version;
        deviceSubversion = deviceTuple.subversion;
        firmwareVersion = deviceTuple.fwVersion;

    } else {
        ret = ErrorEepromNotRecognized;
    }
    return ret;
}

ErrorCodes_t getQueueStatus(
        QueueStatus_t &status) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    ErrorCodes_t ret = Success;
    status.availableDataPackets = (std::numeric_limits <unsigned int> ::max)();
    QueueStatus_t statusn;
    int c = 0;
    for (auto md : msgDisps) {
        ErrorCodes_t retTemp = md->getQueueStatus(statusn);
        availableSamples[c] = statusn.availableDataPackets;
        if (c == 0) {
            status.availableDataPackets = statusn.availableDataPackets;
            status.bufferOverflowFlag = statusn.bufferOverflowFlag;
            status.lostDataFlag = statusn.lostDataFlag;
            status.saturationFlag = statusn.saturationFlag;
            status.currentRangeIncreaseFlag = statusn.currentRangeIncreaseFlag;
            status.currentRangeDecreaseFlag = statusn.currentRangeDecreaseFlag;
            status.communicationErrorFlag = statusn.communicationErrorFlag;

        } else {
            status.availableDataPackets = (std::min)(status.availableDataPackets, statusn.availableDataPackets);
            status.bufferOverflowFlag |= statusn.bufferOverflowFlag;
            status.lostDataFlag |= statusn.lostDataFlag;
            status.saturationFlag |= statusn.saturationFlag;
            status.currentRangeIncreaseFlag |= statusn.currentRangeIncreaseFlag;
            status.currentRangeDecreaseFlag |= statusn.currentRangeDecreaseFlag;
            status.communicationErrorFlag |= statusn.communicationErrorFlag;
        }
        c++;
        if (retTemp != Success && retTemp != WarningNoDataAvailable) {
            return retTemp;
        }

        ret = (ErrorCodes_t)((unsigned int)ret | (unsigned int)retTemp);
    }
    return ret;
}

ErrorCodes_t convertVoltageValue(
        uint16_t intValue,
        double &fltValue) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    return msgDisps[0]->convertVoltageValue(intValue, fltValue);
}

ErrorCodes_t convertCurrentValue(
        uint16_t intValue,
        uint16_t channelIdx,
        double &fltValue) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    return msgDisps[channelIdx/currentChannelsNum]->convertCurrentValue(intValue, channelIdx%currentChannelsNum, fltValue);
}

ErrorCodes_t convertGpValue(
        uint16_t intValue,
        uint16_t channelIdx,
        double &fltValue) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }
    return msgDisps[0]->convertGpValue(intValue, channelIdx, fltValue);
}

ErrorCodes_t readData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        uint16_t * &buffer) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (dataToRead == 0) {
        return WarningNoDataAvailable;
    }

    ErrorCodes_t ret = Success;
    if (msgDispsNum == 1) {
        return msgDisps[0]->getDataPackets(buffer, dataToRead, dataRead);
    }

    int c = 0;
    unsigned int bufferOutIdx;
    unsigned int bufferIdx;
    unsigned int dataToCopy;
    double sdPidControl= 0.0;
    double sdKfGain = 0.0;
    unsigned int discardThreshold = 0;
    for (auto md : msgDisps) {
        sdPidError[c] = (availableSamples[c] > dataToRead ? 1 : -1);
        sdPidIntegralError[c] += sdPidError[c];
        sdKfStateVariance[c] += SD_KF_STATE_ERROR;
        sdPidControl = SD_PID_KP*sdPidError[c]+SD_PID_KI*sdPidIntegralError[c];
        sdKfInnovation[c] = sdPidControl-sdKfStateEstimate[c];
        sdKfMeasurementVariance[c] = sdKfStateVariance[c]+SD_KF_MEASUREMENT_ERROR;
        sdKfGain = sdKfStateVariance[c]/sdKfMeasurementVariance[c];
        sdKfStateEstimate[c] += sdKfGain*sdKfInnovation[c];
        sdKfStateVariance[c] *= (1.0-sdKfGain);

        if (sdKfStateEstimate[c] > 0.0) {
            discardThreshold = (unsigned int)(1.0/sdKfStateEstimate[c]);
            if (sdTotalReadSamples[c] > discardThreshold) {
                sdSamplesToDiscard[c]++;
                sdTotalReadSamples[c] -= discardThreshold;
            }
        }

        ret = md->getDataPackets(buffer, dataToRead+sdSamplesToDiscard[c], dataRead);
        if (dataRead > dataToRead) {
            dataToCopy = dataToRead;
            sdSamplesToDiscard[c] -= dataRead-dataToRead;

        } else {
            dataToCopy = dataRead;
        }

        sdTotalReadSamples[c] += dataRead;

        if (c == 0) {
            bufferOutIdx = 0;
            bufferIdx = 0;
            for (unsigned int idx = 0; idx < dataToCopy; idx++) {
                for (unsigned int chIdx = 0; chIdx < totalChannelsNum; chIdx++) {
                    bufferOut[bufferOutIdx++] = buffer[bufferIdx++];
                }

                bufferOutIdx += totalTotalChannelsNum-totalChannelsNum;
            }

        } else {
            bufferOutIdx = voltageChannelsNum+c*currentChannelsNum;
            bufferIdx = voltageChannelsNum;
            for (unsigned int idx = 0; idx < dataToCopy; idx++) {
                for (unsigned int chIdx = 0; chIdx < currentChannelsNum; chIdx++) {
                    bufferOut[bufferOutIdx++] = buffer[bufferIdx++];
                }

                bufferOutIdx += totalTotalChannelsNum-totalChannelsNum;
                bufferIdx += voltageChannelsNum;
            }
        }
        c++;
    }
    buffer = bufferOut;
    return ret;
}

ErrorCodes_t readAllData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        uint16_t * &buffer,
        uint16_t * &unfilteredBuffer) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (dataToRead == 0) {
        return WarningNoDataAvailable;
    }

    ErrorCodes_t ret = Success;
    if (msgDispsNum == 1) {
        return msgDisps[0]->getAllDataPackets(buffer, unfilteredBuffer, dataToRead, dataRead);
    }

    unsigned int c = 0;
    unsigned int bufferOutIdx;
    unsigned int bufferIdx;
    unsigned int dataToCopy;
    double sdPidControl= 0.0;
    double sdKfGain = 0.0;
    unsigned int discardThreshold = 0;
//    double ds = 0.0;
    for (auto md : msgDisps) {
        sdPidError[c] = (availableSamples[c] > dataToRead ? 1 : -1);
        sdPidIntegralError[c] += sdPidError[c];
        sdKfStateVariance[c] += SD_KF_STATE_ERROR;
        sdPidControl = SD_PID_KP*sdPidError[c]+SD_PID_KI*sdPidIntegralError[c];
        sdKfInnovation[c] = sdPidControl-sdKfStateEstimate[c];
        sdKfMeasurementVariance[c] = sdKfStateVariance[c]+SD_KF_MEASUREMENT_ERROR;
        sdKfGain = sdKfStateVariance[c]/sdKfMeasurementVariance[c];
        sdKfStateEstimate[c] += sdKfGain*sdKfInnovation[c];
        sdKfStateVariance[c] *= (1.0-sdKfGain);

        if (sdKfStateEstimate[c] > 0.0) {
            discardThreshold = (unsigned int)(1.0/sdKfStateEstimate[c]);
            if (sdTotalReadSamples[c] > discardThreshold) {
                sdSamplesToDiscard[c]++;
//                ds = (double) sdSamplesToDiscard[c];
                sdTotalReadSamples[c] -= discardThreshold;
            }
        }

        ret = md->getAllDataPackets(buffer, unfilteredBuffer, dataToRead+sdSamplesToDiscard[c], dataRead);
        if (dataRead > dataToRead) {
            dataToCopy = dataToRead;
            sdSamplesToDiscard[c] -= dataRead-dataToRead;

        } else {
            dataToCopy = dataRead;
        }

        sdTotalReadSamples[c] += dataRead;

        if (c == 0) {
            bufferOutIdx = 0;
            bufferIdx = 0;
            for (unsigned int idx = 0; idx < dataToCopy; idx++) {
                for (unsigned int chIdx = 0; chIdx < totalChannelsNum; chIdx++) {
                    bufferOut[bufferOutIdx] = buffer[bufferIdx];
                    unfilteredBufferOut[bufferOutIdx++] = unfilteredBuffer[bufferIdx++];
                }

                bufferOutIdx += totalTotalChannelsNum-totalChannelsNum;
            }

        } else {
            bufferOutIdx = voltageChannelsNum+c*currentChannelsNum;
            bufferIdx = voltageChannelsNum;
            for (unsigned int idx = 0; idx < dataToCopy; idx++) {
                for (unsigned int chIdx = 0; chIdx < currentChannelsNum; chIdx++) {
                    bufferOut[bufferOutIdx] = buffer[bufferIdx];
                    unfilteredBufferOut[bufferOutIdx++] = unfilteredBuffer[bufferIdx++];
                }

                bufferOutIdx += totalTotalChannelsNum-totalChannelsNum+1;
                bufferIdx += voltageChannelsNum;
            }
        }
//        double pippo = (double) c;
//        fwrite((void *) &pippo, 8, 1, fid);
//        fwrite((void *) &sdPidError[c], 8, 1, fid);
//        fwrite((void *) &sdPidControl, 8, 1, fid);
//        fwrite((void *) &sdKfStateVariance[c], 8, 1, fid);
//        fwrite((void *) &ds, 8, 1, fid);
        dataRead = dataToCopy;
        c++;
    }

    buffer = bufferOut;
    unfilteredBuffer = unfilteredBufferOut;
    return ret;
}

ErrorCodes_t purgeData() {
    MASS_CALL0(purgeData)
}

ErrorCodes_t getChannelsNumber(
        uint32_t &voltageChannelsNum,
        uint32_t &currentChannelsNum,
        uint32_t &gpChannelsNum) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = msgDisps[0]->getChannelsNumber(voltageChannelsNum, currentChannelsNum, gpChannelsNum);
    currentChannelsNum *= msgDispsNum;
    return ret;
}

ErrorCodes_t getCurrentRanges(
        std::vector <RangedMeasurement_t> &currentRanges,
        std::vector <uint16_t> &defaultOptions) {
    CALL_FIRST2(getCurrentRanges, currentRanges, defaultOptions)
}

ErrorCodes_t getCurrentRange(
        RangedMeasurement_t &currentRange,
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > currentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    return msgDisps[channelIdx/currentChannelsNum]->getCurrentRange(currentRange, channelIdx % currentChannelsNum);
}

ErrorCodes_t hasIndependentCurrentRanges() {
    CALL_FIRST0(hasIndependentCurrentRanges)
}

ErrorCodes_t getGpRanges(
        std::vector <std::vector <RangedMeasurement_t>> &gpRanges,
        std::vector <uint16_t> &defaultOptions,
        std::vector <std::string> &names) {
    CALL_FIRST3(getGpRanges, gpRanges, defaultOptions, names)
}

ErrorCodes_t getGpRange(
        RangedMeasurement_t &gpRange,
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > gpChannelsNum) {
        return ErrorValueOutOfRange;
    }

    return msgDisps[channelIdx/gpChannelsNum]->getGpRange(gpRange, channelIdx % gpChannelsNum);
}

ErrorCodes_t getVoltageRanges(
        std::vector <RangedMeasurement_t> &voltageRanges,
        uint16_t &defaultOption,
        std::vector <std::string> &extensions) {
    CALL_FIRST3(getVoltageRanges, voltageRanges, defaultOption, extensions)
}

ErrorCodes_t getVoltageRange(
        RangedMeasurement_t &voltageRange) {
    CALL_FIRST1(getVoltageRange, voltageRange)
}

ErrorCodes_t getVoltageReferenceRanges(
        std::vector <RangedMeasurement_t> &ranges,
        uint16_t &defaultOption) {
    CALL_FIRST2(getVoltageReferenceRanges, ranges, defaultOption)
}

ErrorCodes_t getVoltageReferenceRange(
        RangedMeasurement_t &range) {
    CALL_FIRST1(getVoltageReferenceRange, range)
}

ErrorCodes_t getSamplingRates(
        std::vector <Measurement_t> &samplingRates,
        uint16_t &defaultOption) {
    CALL_FIRST2(getSamplingRates, samplingRates, defaultOption)
}

ErrorCodes_t getSamplingRate(
        Measurement_t &samplingRate) {
    CALL_FIRST1(getSamplingRate, samplingRate)
}

ErrorCodes_t getRealSamplingRates(
        std::vector <Measurement_t> &samplingRates) {
    CALL_FIRST1(getRealSamplingRates, samplingRates)
}

ErrorCodes_t getRealSamplingRate(
        Measurement_t &samplingRate) {
    CALL_FIRST1(getRealSamplingRate, samplingRate)
}

ErrorCodes_t getOversamplingRatios(
        std::vector <uint16_t> &oversamplingRatios) {
    CALL_FIRST1(getOversamplingRatios, oversamplingRatios)
}

ErrorCodes_t getOversamplingRatio(
        uint16_t &oversamplingRatio) {
    CALL_FIRST1(getOversamplingRatio, oversamplingRatio)
}

ErrorCodes_t getVoltageStimulusLpfs(
        std::vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    CALL_FIRST3(getVoltageStimulusLpfs, filterOptions, defaultOption, voltageRangeIdx)
}

ErrorCodes_t getVoltageReferenceLpfs(
        std::vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    CALL_FIRST3(getVoltageReferenceLpfs, filterOptions, defaultOption, voltageRangeIdx)
}

ErrorCodes_t hasSelectStimulusChannel(
        bool &selectStimulusChannelFlag,
        bool &singleChannelSSCFlag) {
    CALL_FIRST2(hasSelectStimulusChannel, selectStimulusChannelFlag, singleChannelSSCFlag)
}

ErrorCodes_t hasDigitalOffsetCompensation(
        bool &digitalOffsetCompensationFlag,
        bool &singleChannelDOCFlag,
        bool &selectableDOCAutostopFlag) {
    CALL_FIRST3(hasDigitalOffsetCompensation, digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag)
}

ErrorCodes_t hasZap(
        bool &zappableDeviceFlag,
        bool &singleChannelZapFlag) {
    CALL_FIRST2(hasZap, zappableDeviceFlag, singleChannelZapFlag)
}

ErrorCodes_t hasChannelOn(
        bool &channelOnFlag,
        bool &singleChannelOnFlag) {
    CALL_FIRST2(hasChannelOn, channelOnFlag, singleChannelOnFlag)
}

ErrorCodes_t getSwitchedOnChannels(
        uint32_t &channelsMask) {
    CALL_FIRST1(getSwitchedOnChannels, channelsMask)
}

ErrorCodes_t hasDigitalOffsetCompensationReset() {
    CALL_FIRST0(hasDigitalOffsetCompensationReset)
}

ErrorCodes_t hasDigitalOutput() {
    CALL_FIRST0(hasDigitalOutput)
}

ErrorCodes_t hasFrontEndResetDenoiser() {
    CALL_FIRST0(hasFrontEndResetDenoiser)
}

ErrorCodes_t getProtocolList(
        std::vector <std::string> &names,
        std::vector <std::string> &images,
        std::vector <std::vector <uint16_t>> &voltages,
        std::vector <std::vector <uint16_t>> &times,
        std::vector <std::vector <uint16_t>> &slopes,
        std::vector <std::vector <uint16_t>> &frequencies,
        std::vector <std::vector <uint16_t>> &adimensionals) {
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->getProtocolList(names, images, voltages, times, slopes, frequencies, adimensionals);
}

ErrorCodes_t getTriangularProtocolIdx(
        uint16_t &idx) {
    CALL_FIRST1(getTriangularProtocolIdx, idx)
}

ErrorCodes_t getSealTestProtocolIdx(
        uint16_t &idx) {
    CALL_FIRST1(getSealTestProtocolIdx, idx)
}

ErrorCodes_t getProtocolVoltage(
        std::vector <std::string> &voltageNames,
        std::vector <RangedMeasurement_t> &ranges,
        std::vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolVoltage, voltageNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolTime(
        std::vector <std::string> &timeNames,
        std::vector <RangedMeasurement_t> &ranges,
        std::vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolTime, timeNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolSlope(
        std::vector <std::string> &slopeNames,
        std::vector <RangedMeasurement_t> &ranges,
        std::vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolSlope, slopeNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolFrequency(
        std::vector <std::string> &frequencyNames,
        std::vector <RangedMeasurement_t> &ranges,
        std::vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolFrequency, frequencyNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolAdimensional(
        std::vector <std::string> &adimensionalNames,
        std::vector <RangedMeasurement_t> &ranges,
        std::vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolAdimensional, adimensionalNames, ranges, defaultValues)
}

ErrorCodes_t getVoltageOffsetControls(
        RangedMeasurement_t &voltageRange) {
    CALL_FIRST1(getVoltageOffsetControls, voltageRange)
}

ErrorCodes_t getInsertionPulseControls(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange) {
    CALL_FIRST2(getInsertionPulseControls, voltageRange, durationRange)
}

ErrorCodes_t hasReferencePulseControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    CALL_FIRST2(hasReferencePulseControls, referencePulseImplemented, overrideReferencePulseImplemented)
}

ErrorCodes_t getReferencePulseControls(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange) {
    CALL_FIRST2(getReferencePulseControls, voltageRange, durationRange)
}

ErrorCodes_t hasReferencePulseTrainControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    CALL_FIRST2(hasReferencePulseTrainControls, referencePulseImplemented, overrideReferencePulseImplemented)
}

ErrorCodes_t getReferencePulseTrainControls(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange,
        RangedMeasurement_t &periodRange,
        uint16_t &pulsesNumber) {
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->getReferencePulseTrainControls(voltageRange, durationRange, periodRange, pulsesNumber);
    }

ErrorCodes_t getEdhFormat(
        std::string &format) {
    CALL_FIRST1(getEdhFormat, format)
}

ErrorCodes_t getRawDataFilterCutoffFrequency(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    CALL_FIRST2(getRawDataFilterCutoffFrequency, range, defaultValue)
}

ErrorCodes_t getLedsNumber(
        uint16_t &ledsNum) {
    CALL_FIRST1(getLedsNumber, ledsNum)
}

ErrorCodes_t getLedsColors(
        std::vector <uint32_t> &ledsColors) {
    CALL_FIRST1(getLedsColors, ledsColors)
}

ErrorCodes_t getFastReferencePulseProtocolWave1Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        uint16_t &nPulse) {
    CALL_FIRST3(getFastReferencePulseProtocolWave1Range, voltageRange, timeRange, nPulse)
}

ErrorCodes_t getFastReferencePulseProtocolWave2Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        RangedMeasurement_t &durationRange,
        uint16_t &nPulse) {
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->getFastReferencePulseProtocolWave2Range(voltageRange, timeRange, durationRange, nPulse);
}

ErrorCodes_t getFastReferencePulseTrainProtocolWave2Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        RangedMeasurement_t &durationRange,
        RangedMeasurement_t &periodRange,
        uint16_t &pulsesPerTrain,
        uint16_t &nTrains) {
    return (msgDisps.empty()) ? ErrorDeviceNotConnected : msgDisps[0]->getFastReferencePulseTrainProtocolWave2Range(voltageRange, timeRange, durationRange, periodRange, pulsesPerTrain, nTrains);
}

/*************************\
 *  Calibration methods  *
\*************************/

ErrorCodes_t getCalibrationEepromSize(
        uint32_t &size) {
    ErrorCodes_t ret;
    if (!msgDisps.empty()) {
        ret = msgDisps[0]->getCalibrationEepromSize(size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t writeCalibrationEeprom(
        std::vector <uint32_t> value,
        std::vector <uint32_t> address,
        std::vector <uint32_t> size) {
    ErrorCodes_t ret;
    if (!msgDisps.empty()) {
        ret = msgDisps[0]->writeCalibrationEeprom(value, address, size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t readCalibrationEeprom(
        std::vector <uint32_t> &value,
        std::vector <uint32_t> address,
        std::vector <uint32_t> size) {
    ErrorCodes_t ret;
    if (!msgDisps.empty()) {
        ret = msgDisps[0]->readCalibrationEeprom(value, address, size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCustomFlags(
        std::vector <std::string> &customFlags,
        std::vector <bool> &customFlagsDefault) {
    CALL_FIRST2(getCustomFlags, customFlags, customFlagsDefault)
}

ErrorCodes_t getCustomDoubles(
        std::vector <std::string> &customDoubles,
        std::vector <RangedMeasurement_t> &customDoublesRanges,
        std::vector <double> &customDoublesDefault) {
    CALL_FIRST3(getCustomDoubles, customDoubles, customDoublesRanges, customDoublesDefault)
}

ErrorCodes_t hasNanionTemperatureController() {
    CALL_FIRST0(hasNanionTemperatureController)
}

ErrorCodes_t getTemperatureControllerRange(
        int &minTemperature,
        int &maxTemperature) {
    CALL_FIRST2(getTemperatureControllerRange, minTemperature, maxTemperature)
}

ErrorCodes_t hasWasherControls() {
    CALL_FIRST0(hasWasherControls)
}

ErrorCodes_t getWasherSpeedRange(
        RangedMeasurement_t &range) {
    CALL_FIRST1(getWasherSpeedRange, range)
}

ErrorCodes_t getWasherStatus(
        WasherStatus_t &status,
        WasherError_t &error) {
    CALL_FIRST2(getWasherStatus, status, error)
}

ErrorCodes_t getWasherPresetSpeeds(
        std::vector <int8_t> &speedValue) {
    CALL_FIRST1(getWasherPresetSpeeds, speedValue)
}


ErrorCodes_t hasCFastCompensation() {
    CALL_FIRST0(hasCFastCompensation)
}

ErrorCodes_t getCFastCompensationOptions(
        std::vector <std::string> &options) {
    CALL_FIRST1(getCFastCompensationOptions, options)
}

ErrorCodes_t getCFastCapacitanceControl(
        CompensationControl_t &control) {
    CALL_FIRST1(getCFastCapacitanceControl, control)
}

ErrorCodes_t hasTtlPulseTrain() {
    CALL_FIRST0(hasTtlPulseTrain)
}

ErrorCodes_t getVoltageOffsetCompensations(
        std::vector <Measurement_t> &offsets) {
    ErrorCodes_t ret = Success;
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (msgDispsNum == 1) {
        return msgDisps[0]->updateVoltageOffsetCompensations(offsets);
    }

    std::vector <Measurement_t> tempOffsets(currentChannelsNum);

    int c = 0;
    for (auto md : msgDisps) {
        ret = md->updateVoltageOffsetCompensations(tempOffsets);
        for (unsigned int idx = 0; idx < currentChannelsNum; idx++) {
            offsets[c+idx] = tempOffsets[idx];
        }
        c += currentChannelsNum;
    }

    return ret;
}

} // namespace er4CommLib

void allocateSampleDiscardVariables() {
    sdPidError.resize(totalCurrentChannelsNum);
    sdPidIntegralError.resize(totalCurrentChannelsNum);
    sdKfStateVariance.resize(totalCurrentChannelsNum);
    sdKfInnovation.resize(totalCurrentChannelsNum);
    sdKfStateEstimate.resize(totalCurrentChannelsNum);
    sdKfMeasurementVariance.resize(totalCurrentChannelsNum);
    sdTotalReadSamples.resize(totalCurrentChannelsNum);
    availableSamples.resize(totalCurrentChannelsNum);
    sdSamplesToDiscard.resize(totalCurrentChannelsNum);
}

void resetSampleDiscardVariables() {
    std::fill(sdPidError.begin(), sdPidError.end(), 0.0);
    std::fill(sdPidIntegralError.begin(), sdPidIntegralError.end(), 0.0);
    std::fill(sdKfStateVariance.begin(), sdKfStateVariance.end(), 0.0);
    std::fill(sdKfInnovation.begin(), sdKfInnovation.end(), 0.0);
    std::fill(sdKfStateEstimate.begin(), sdKfStateEstimate.end(), 0.0);
    std::fill(sdKfMeasurementVariance.begin(), sdKfMeasurementVariance.end(), 0.0);
    std::fill(sdTotalReadSamples.begin(), sdTotalReadSamples.end(), 0.0);
    std::fill(availableSamples.begin(), availableSamples.end(), 0);
    std::fill(sdSamplesToDiscard.begin(), sdSamplesToDiscard.end(), 0);
}
