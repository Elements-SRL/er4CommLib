//  Copyright (C) 2021-2024 Filippo Cona
//
//  This file is part of EDR4.
//
//  EDR4 is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  EDR4 is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with EDR4.  If not, see <http://www.gnu.org/licenses/>.

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

#define SAMPLES_DISCARD_THRESHOLD 100000

#include "er4commlib.h"

#include <algorithm>

#include "messagedispatcher.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include "ftd2xx_win.h"

static MessageDispatcher * messageDispatcher = nullptr;
static std::vector <MessageDispatcher *> msgDisps;
static unsigned int voltageChannelsNum = 0;
static unsigned int currentChannelsNum = 0;
static unsigned int totalChannelsNum = 0;
static unsigned int msgDispsNum = 0;
static unsigned int totalCurrentChannelsNum = 0;
static unsigned int totalTotalChannelsNum = 0;
static unsigned int msgDispCompensated = 0;
static uint16_t * bufferOut = nullptr;
static uint16_t * unfilteredBufferOut = nullptr;
static std::vector <uint32_t> totalSamples;
static std::vector <uint32_t> samplesToDiscard;
static std::vector <std::vector <uint32_t>> synchronizationMovingAverageQueue;
static std::vector <uint32_t> synchronizationMovingAverage;
static uint32_t synchronizationMovingAverageIndex;
static std::vector <uint32_t> availableSamples;
static std::vector <uint32_t> previousAvailableSamples;
static std::vector <uint32_t> previousDataRead;

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
            return ErrorUnknown; /*! \todo FCON sostituire */
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

    msgDisps[0]->getChannelsNumber(voltageChannelsNum, currentChannelsNum);
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;
    totalCurrentChannelsNum = currentChannelsNum*msgDispsNum;
    totalTotalChannelsNum = voltageChannelsNum+totalCurrentChannelsNum;

    for (auto md : msgDisps) {
        md->setMaxOutputPacketsNum(ER4CL_DATA_ARRAY_SIZE/(totalCurrentChannelsNum+voltageChannelsNum));
    }
    bufferOut = new (nothrow) uint16_t[ER4CL_DATA_ARRAY_SIZE];
    unfilteredBufferOut = new (nothrow) uint16_t[ER4CL_DATA_ARRAY_SIZE];

    /*! Synchronization stuff */
    totalSamples.resize(totalCurrentChannelsNum);
    std::fill(totalSamples.begin(), totalSamples.end(), 0);
    samplesToDiscard.resize(totalCurrentChannelsNum);
    std::fill(samplesToDiscard.begin(), samplesToDiscard.end(), 0);
    synchronizationMovingAverageQueue.resize(totalCurrentChannelsNum);
    for (auto&& q : synchronizationMovingAverageQueue) {
        q.resize(SAMPLES_DISCARD_THRESHOLD);
        std::fill(q.begin(), q.end(), 0);
    }
    synchronizationMovingAverage.resize(totalCurrentChannelsNum);
    std::fill(synchronizationMovingAverage.begin(), synchronizationMovingAverage.end(), 0);
    synchronizationMovingAverageIndex = 0;
    availableSamples.resize(totalCurrentChannelsNum);
    std::fill(availableSamples.begin(), availableSamples.end(), 0);
    previousAvailableSamples.resize(totalCurrentChannelsNum);
    std::fill(previousAvailableSamples.begin(), previousAvailableSamples.end(), 0);
    previousDataRead.resize(totalCurrentChannelsNum);
    std::fill(previousDataRead.begin(), previousDataRead.end(), 0);
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
        vector <int8_t> speedValues) {
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

//ErrorCodes_t CommLib::resetDigitalOffsetCompensation(bool reset) {
//    ErrorCodes_t ret;
//    if (messageDispatcher != nullptr) {
//        ret = messageDispatcher->resetDigitalOffsetCompensation(reset);

//    } else {
//        ret = ErrorDeviceNotConnected;
//    }
//    return ret;
//}

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
        string deviceId,
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
    status.availableDataPackets = std::numeric_limits <unsigned int> ::max();
    QueueStatus_t statusn;
    int c = 0;
    for (auto md : msgDisps) {
        ret = md->getQueueStatus(statusn);
        availableSamples[c++] = statusn.availableDataPackets;
        status.availableDataPackets = std::min(status.availableDataPackets, statusn.availableDataPackets);
        status.bufferOverflowFlag = status.bufferOverflowFlag || statusn.bufferOverflowFlag;
        status.lostDataFlag = status.lostDataFlag || statusn.lostDataFlag;
        status.saturationFlag = status.saturationFlag || statusn.saturationFlag;
        status.currentRangeIncreaseFlag = status.currentRangeIncreaseFlag || statusn.currentRangeIncreaseFlag;
        status.currentRangeDecreaseFlag = status.currentRangeDecreaseFlag || statusn.currentRangeDecreaseFlag;
        status.communicationErrorFlag = status.communicationErrorFlag || statusn.communicationErrorFlag;
        if (ret != Success && ret != WarningNoDataAvailable) {
            return ret;
        }
    }
    return ret;
}

ErrorCodes_t getChannelsNumber(
        uint32_t &voltageChannelsNum,
        uint32_t &currentChannelsNum) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = msgDisps[0]->getChannelsNumber(voltageChannelsNum, currentChannelsNum);
    currentChannelsNum *= msgDispsNum;
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

ErrorCodes_t readData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        uint16_t * &buffer) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = Success;
    if (msgDispsNum == 1) {
        return msgDisps[0]->getDataPackets(buffer, dataToRead, dataRead);
    }

    int c = 0;
    unsigned int bufferOutIdx;
    unsigned int bufferIdx;
    for (auto md : msgDisps) {
        ret = md->getDataPackets(buffer, dataToRead, dataRead);
        if (c == 0) {
            bufferOutIdx = 0;
            bufferIdx = 0;
            for (unsigned int idx = 0; idx < dataRead; idx++) {
                for (unsigned int chIdx = 0; chIdx < totalChannelsNum; chIdx++) {
                    bufferOut[bufferOutIdx++] = buffer[bufferIdx++];
                }

                bufferOutIdx += totalTotalChannelsNum-totalChannelsNum;
            }

        } else {
            bufferOutIdx = voltageChannelsNum+c*currentChannelsNum;
            bufferIdx = voltageChannelsNum;
            for (unsigned int idx = 0; idx < dataRead; idx++) {
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

    ErrorCodes_t ret = Success;
    if (msgDispsNum == 1) {
        return msgDisps[0]->getAllDataPackets(buffer, unfilteredBuffer, dataToRead, dataRead);
    }

    unsigned int c = 0;
    unsigned int bufferOutIdx;
    unsigned int bufferIdx;
    unsigned int dataToCopy;
    unsigned int minTotalSamples = std::numeric_limits <unsigned int>::max();
    unsigned int newSamples;
    for (auto md : msgDisps) {
        if (totalSamples[c] > SAMPLES_DISCARD_THRESHOLD) {
            samplesToDiscard[c]++;
        }

        ret = md->getAllDataPackets(buffer, unfilteredBuffer, dataToRead+samplesToDiscard[c], dataRead);
        if (dataRead > dataToRead) {
            dataToCopy = dataToRead;
            samplesToDiscard[c] -= dataRead-dataToRead;

        } else {
            dataToCopy = dataRead;
        }
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
        newSamples = availableSamples[c]-previousAvailableSamples[c]+previousDataRead[c];
        previousDataRead[c] = dataRead;
        previousAvailableSamples[c] = availableSamples[c];
        synchronizationMovingAverage[c] += newSamples-synchronizationMovingAverageQueue[c][synchronizationMovingAverageIndex];
        synchronizationMovingAverageQueue[c][synchronizationMovingAverageIndex] = newSamples;
        totalSamples[c] += synchronizationMovingAverage[c];
        if (totalSamples[c] < minTotalSamples) {
            minTotalSamples = totalSamples[c];
        }
        dataRead = dataToCopy;
        c++;
    }

    for (c = 0; c < totalCurrentChannelsNum; c++) {
        totalSamples[c] -= minTotalSamples;
    }

    synchronizationMovingAverageIndex++;
    if (synchronizationMovingAverageIndex >= SAMPLES_DISCARD_THRESHOLD) {
        synchronizationMovingAverageIndex = 0;
    }
    buffer = bufferOut;
    unfilteredBuffer = unfilteredBufferOut;
    return ret;
}

ErrorCodes_t purgeData() {
    MASS_CALL0(purgeData)
}

ErrorCodes_t getCurrentRanges(
        vector <RangedMeasurement_t> &currentRanges,
        vector <uint16_t> &defaultOptions) {
    CALL_FIRST2(getCurrentRanges, currentRanges, defaultOptions)
}

ErrorCodes_t getCurrentRange(
        RangedMeasurement_t &currentRange,
        uint16_t channelIdx) {
    if (msgDisps.empty()) {
        return ErrorDeviceNotConnected;
    }

    if (channelIdx > totalCurrentChannelsNum) {
        return ErrorValueOutOfRange;
    }

    return msgDisps[channelIdx/currentChannelsNum]->getCurrentRange(currentRange, channelIdx % currentChannelsNum);
}

ErrorCodes_t hasIndependentCurrentRanges() {
    CALL_FIRST0(hasIndependentCurrentRanges)
}

ErrorCodes_t getVoltageRanges(
        vector <RangedMeasurement_t> &voltageRanges,
        uint16_t &defaultOption,
        vector <string> &extensions) {
    CALL_FIRST3(getVoltageRanges, voltageRanges, defaultOption, extensions)
}

ErrorCodes_t getVoltageRange(
        RangedMeasurement_t &voltageRange) {
    CALL_FIRST1(getVoltageRange, voltageRange)
}

ErrorCodes_t getVoltageReferenceRanges(
        vector <RangedMeasurement_t> &ranges,
        uint16_t &defaultOption) {
    CALL_FIRST2(getVoltageReferenceRanges, ranges, defaultOption)
}

ErrorCodes_t getSamplingRates(
        vector <Measurement_t> &samplingRates,
        uint16_t &defaultOption) {
    CALL_FIRST2(getSamplingRates, samplingRates, defaultOption)
}

ErrorCodes_t getSamplingRate(
        Measurement_t &samplingRate) {
    CALL_FIRST1(getSamplingRate, samplingRate)
}

ErrorCodes_t getRealSamplingRates(
        vector <Measurement_t> &samplingRates) {
    CALL_FIRST1(getRealSamplingRates, samplingRates)
}

ErrorCodes_t getRealSamplingRate(
        Measurement_t &samplingRate) {
    CALL_FIRST1(getRealSamplingRate, samplingRate)
}

ErrorCodes_t getOversamplingRatios(
        vector <uint16_t> &oversamplingRatios) {
    CALL_FIRST1(getOversamplingRatios, oversamplingRatios)
}

ErrorCodes_t getOversamplingRatio(
        uint16_t &oversamplingRatio) {
    CALL_FIRST1(getOversamplingRatio, oversamplingRatio)
}

ErrorCodes_t getVoltageStimulusLpfs(
        vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    CALL_FIRST3(getVoltageStimulusLpfs, filterOptions, defaultOption, voltageRangeIdx)
}

ErrorCodes_t getVoltageReferenceLpfs(
        vector <Measurement_t> &filterOptions,
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
        vector <string> &names,
        vector <string> &images,
        vector <vector <uint16_t>> &voltages,
        vector <vector <uint16_t>> &times,
        vector <vector <uint16_t>> &slopes,
        vector <vector <uint16_t>> &frequencies,
        vector <vector <uint16_t>> &adimensionals) {
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
// DA QUI-------------------------------------
ErrorCodes_t getProtocolVoltage(
        vector <string> &voltageNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolVoltage, voltageNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolTime(
        vector <string> &timeNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolTime, timeNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolSlope(
        vector <string> &slopeNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolSlope, slopeNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolFrequency(
        vector <string> &frequencyNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    CALL_FIRST3(getProtocolFrequency, frequencyNames, ranges, defaultValues)
}

ErrorCodes_t getProtocolAdimensional(
        vector <string> &adimensionalNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
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
        string &format) {
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
        vector <uint32_t> &ledsColors) {
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
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCalibrationEepromSize(size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t writeCalibrationEeprom(
        vector <uint32_t> value,
        vector <uint32_t> address,
        vector <uint32_t> size) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->writeCalibrationEeprom(value, address, size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t readCalibrationEeprom(
        vector <uint32_t> &value,
        vector <uint32_t> address,
        vector <uint32_t> size) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->readCalibrationEeprom(value, address, size);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCustomFlags(
        vector <string> &customFlags,
        vector <bool> &customFlagsDefault) {
    CALL_FIRST2(getCustomFlags, customFlags, customFlagsDefault)
}

ErrorCodes_t getCustomDoubles(
        vector <string> &customDoubles,
        vector <RangedMeasurement_t> &customDoublesRanges,
        vector <double> &customDoublesDefault) {
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
        vector <int8_t> &speedValue) {
    CALL_FIRST1(getWasherPresetSpeeds, speedValue)
}


ErrorCodes_t hasCFastCompensation() {
    CALL_FIRST0(hasCFastCompensation)
}

ErrorCodes_t getCFastCompensationOptions(
        vector <string> &options) {
    CALL_FIRST1(getCFastCompensationOptions, options)
}

ErrorCodes_t getCFastCapacitanceControl(
        CompensationControl_t &control) {
    CALL_FIRST1(getCFastCapacitanceControl, control)
}

ErrorCodes_t getVoltageOffsetCompensations(
        std::vector <Measurement_t> &offsets) {
    ErrorCodes_t ret;
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

    return Success;
}

} // namespace er4CommLib
