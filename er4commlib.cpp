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

#include "er4commlib.h"

#include <algorithm>

#include "messagedispatcher.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include "ftd2xx_win.h"

static MessageDispatcher * messageDispatcher = nullptr;
static std::vector <MessageDispatcher *> msgDisps;

namespace er4CommLib {

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t detectDevices(
        vector <string> &deviceIds) {
    /*! Gets number of devices */
    return MessageDispatcher::detectDevices(deviceIds);
}

ErrorCodes_t connect(
        std::vector <std::string> deviceIds) {

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
            return ErrorUnknown; // definire un tipo di errore per questa situazione
        }
    }

    msgDisps.resize(deviceIds.size());
    ErrorCodes_t err = Success;
    int c = 0;
    for (auto&& deviceId : deviceIds) {
        err = MessageDispatcher::connectDevice(deviceId, msgDisps[c++]);
    }

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
        unsigned int idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageOffset(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkVoltageOffset(
        unsigned int idx,
        Measurement_t voltage,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkVoltageOffset(idx, voltage, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyInsertionPulse(
        Measurement_t voltage,
        Measurement_t duration) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->applyInsertionPulse(voltage, duration);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyReferencePulse(
        Measurement_t voltage,
        Measurement_t duration) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->applyReferencePulse(voltage, duration);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyReferencePulseTrain(
        Measurement_t voltage,
        Measurement_t duration,
        Measurement_t period,
        uint16_t number) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->applyReferencePulseTrain(voltage, duration, period, number);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t overrideReferencePulse(
      bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->overrideReferencePulse(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setRawDataFilter(
        Measurement_t cutoffFrequency,
        bool lowPassFlag,
        bool activeFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setRawDataFilter(cutoffFrequency, lowPassFlag, activeFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyDacExt(
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->applyDacExt(voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCustomFlag(
        uint16_t idx,
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCustomFlag(idx, flag, true);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCustomDouble(
        uint16_t idx,
        double value) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCustomDouble(idx, value, true);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t resetWasherError() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->resetWasherError();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setWasherPresetSpeeds(
        vector <int8_t> speedValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setWasherPresetSpeeds(speedValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t startWasher(
        uint16_t speedIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->startWasher(speedIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t updateWasherState() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->updateWasherState();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t updateWasherPresetSpeeds() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->updateWasherPresetSpeeds();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCurrentRange(
        uint16_t currentRangeIdx,
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCurrentRange(currentRangeIdx, channelIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVoltageRange(
        uint16_t voltageRangeIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageRange(voltageRangeIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVoltageReferenceRange(
        uint16_t voltageRangeIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageReferenceRange(voltageRangeIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setSamplingRate(
        uint16_t samplingRateIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setSamplingRate(samplingRateIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setOversamplingRatio(
        uint16_t oversamplingRatioIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setOversamplingRatio(oversamplingRatioIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVoltageStimulusLpf(
        uint16_t filterIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageStimulusLpf(filterIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVoltageReferenceLpf(
        uint16_t filterIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageReferenceLpf(filterIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t selectStimulusChannel(
        uint16_t channelIdx,
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->selectStimulusChannel(channelIdx, on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t digitalOffsetCompensation(
        uint16_t channelIdx,
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->digitalOffsetCompensation(channelIdx, on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t digitalOffsetCompensationAutostop(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->digitalOffsetCompensationAutostop(on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t zap(
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->zap(channelIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t switchChannelOn(
        uint16_t channelIdx,
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->switchChannelOn(channelIdx, on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(
        uint32_t idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave1Voltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave1Time(
        uint32_t idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave1Time(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(
        uint32_t idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Voltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Time(
        uint32_t idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Time(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Duration(
        uint32_t idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Duration(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Period(
        uint32_t idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Period(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2PulseNumber(
        uint32_t idx,
        uint16_t pulseNumber) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2PulseNumber(idx, pulseNumber);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t turnOnDigitalOutput(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->turnOnDigitalOutput(on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t turnLedOn(
        uint16_t ledIndex,
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->turnLedOn(ledIndex, on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t enableFrontEndResetDenoiser(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->enableFrontEndResetDenoiser(on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t resetDevice() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->resetDevice();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t resetDigitalOffsetCompensation() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->resetDigitalOffsetCompensation();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCompensationsChannel(
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCompensationsChannel(channelIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t turnCFastCompensationOn(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->turnCFastCompensationOn(on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCFastCompensationOptions(
        uint16_t optionIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCFastCompensationOptions(optionIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCFastCapacitance(
        Measurement_t value) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCFastCapacitance(value);

    } else {
        ret = ErrorDeviceNotConnected;
    }
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
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setDebugBit(byteOffset, bitOffset, status);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setDebugByte(
        uint16_t byteOffset,
        uint16_t byteValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setDebugByte(byteOffset, byteValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
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
    ErrorCodes_t ret = Success;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getDeviceInfo(deviceVersion, deviceSubversion, firmwareVersion);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
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
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getQueueStatus(status);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getChannelsNumber(
        uint32_t &voltageChannelsNum,
        uint32_t &currentChannelsNum) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getChannelsNumber(voltageChannelsNum, currentChannelsNum);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t convertVoltageValue(
        uint16_t intValue,
        double &fltValue) {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->convertVoltageValue(intValue, fltValue);

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t convertCurrentValue(
        uint16_t intValue,
        uint16_t channelIdx,
        double &fltValue) {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->convertCurrentValue(intValue, channelIdx, fltValue);

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t readData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        uint16_t * &buffer) {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->getDataPackets(buffer, dataToRead, dataRead);

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t readAllData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        uint16_t * &buffer,
        uint16_t * &unfilteredBuffer) {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->getAllDataPackets(buffer, unfilteredBuffer, dataToRead, dataRead);

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t purgeData() {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->purgeData();

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t getCurrentRanges(
        vector <RangedMeasurement_t> &currentRanges,
        vector <uint16_t> &defaultOptions) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCurrentRanges(currentRanges, defaultOptions);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCurrentRange(
        RangedMeasurement_t &currentRange,
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCurrentRange(currentRange, channelIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasIndependentCurrentRanges() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasIndependentCurrentRanges();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageRanges(
        vector <RangedMeasurement_t> &voltageRanges,
        uint16_t &defaultOption,
        vector <string> &extensions) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageRanges(voltageRanges, defaultOption, extensions);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageRange(
        RangedMeasurement_t &voltageRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageRange(voltageRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceRanges(
        vector <RangedMeasurement_t> &ranges,
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageReferenceRanges(ranges, defaultOption);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSamplingRates(
        vector <Measurement_t> &samplingRates,
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getSamplingRates(samplingRates, defaultOption);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSamplingRate(
        Measurement_t &samplingRate) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getSamplingRate(samplingRate);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getRealSamplingRates(
        vector <Measurement_t> &samplingRates) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getRealSamplingRates(samplingRates);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getRealSamplingRate(
        Measurement_t &samplingRate) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getRealSamplingRate(samplingRate);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getOversamplingRatios(
        vector <uint16_t> &oversamplingRatios) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getOversamplingRatios(oversamplingRatios);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getOversamplingRatio(
        uint16_t &oversamplingRatio) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getOversamplingRatio(oversamplingRatio);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageStimulusLpfs(
        vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageStimulusLpfs(filterOptions, defaultOption, voltageRangeIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceLpfs(
        vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageReferenceLpfs(filterOptions, defaultOption, voltageRangeIdx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasSelectStimulusChannel(
        bool &selectStimulusChannelFlag,
        bool &singleChannelSSCFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasSelectStimulusChannel(selectStimulusChannelFlag, singleChannelSSCFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasDigitalOffsetCompensation(
        bool &digitalOffsetCompensationFlag,
        bool &singleChannelDOCFlag,
        bool &selectableDOCAutostopFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasDigitalOffsetCompensation(digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasZap(
        bool &zappableDeviceFlag,
        bool &singleChannelZapFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasZap(zappableDeviceFlag, singleChannelZapFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasChannelOn(
        bool &channelOnFlag,
        bool &singleChannelOnFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasChannelOn(channelOnFlag, singleChannelOnFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSwitchedOnChannels(
        uint32_t &channelsMask) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getSwitchedOnChannels(channelsMask);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasDigitalOffsetCompensationReset() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasDigitalOffsetCompensationReset();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasDigitalOutput() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasDigitalOutput();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasFrontEndResetDenoiser() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasFrontEndResetDenoiser();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolList(
        vector <string> &names,
        vector <string> &images,
        vector <vector <uint16_t>> &voltages,
        vector <vector <uint16_t>> &times,
        vector <vector <uint16_t>> &slopes,
        vector <vector <uint16_t>> &frequencies,
        vector <vector <uint16_t>> &adimensionals) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolList(names, images, voltages, times, slopes, frequencies, adimensionals);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getTriangularProtocolIdx(
        uint16_t &idx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getTriangularProtocolIdx(idx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSealTestProtocolIdx(
        uint16_t &idx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getSealTestProtocolIdx(idx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolVoltage(
        vector <string> &voltageNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolVoltage(voltageNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolTime(
        vector <string> &timeNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolTime(timeNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolSlope(
        vector <string> &slopeNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolSlope(slopeNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolFrequency(
        vector <string> &frequencyNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolFrequency(frequencyNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolAdimensional(
        vector <string> &adimensionalNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolAdimensional(adimensionalNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageOffsetControls(
        RangedMeasurement_t &voltageRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageOffsetControls(voltageRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getInsertionPulseControls(RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getInsertionPulseControls(voltageRange, durationRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasReferencePulseControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasReferencePulseControls(referencePulseImplemented, overrideReferencePulseImplemented);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getReferencePulseControls(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getReferencePulseControls(voltageRange, durationRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasReferencePulseTrainControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasReferencePulseTrainControls(referencePulseImplemented, overrideReferencePulseImplemented);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getReferencePulseTrainControls(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange,
        RangedMeasurement_t &periodRange,
        uint16_t &pulsesNumber) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getReferencePulseTrainControls(voltageRange, durationRange, periodRange, pulsesNumber);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getEdhFormat(
        string &format) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getEdhFormat(format);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getRawDataFilterCutoffFrequency(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getRawDataFilterCutoffFrequency(range, defaultValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getLedsNumber(
        uint16_t &ledsNum) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getLedsNumber(ledsNum);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getLedsColors(
        vector <uint32_t> &ledsColors) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getLedsColors(ledsColors);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave1Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        uint16_t &nPulse) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFastReferencePulseProtocolWave1Range(voltageRange, timeRange, nPulse);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave2Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        RangedMeasurement_t &durationRange,
        uint16_t &nPulse) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFastReferencePulseProtocolWave2Range(voltageRange, timeRange, durationRange, nPulse);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseTrainProtocolWave2Range(
        RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &timeRange,
        RangedMeasurement_t &durationRange,
        RangedMeasurement_t &periodRange,
        uint16_t &pulsesPerTrain,
        uint16_t &nTrains) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFastReferencePulseTrainProtocolWave2Range(voltageRange, timeRange, durationRange, periodRange, pulsesPerTrain, nTrains);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
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
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCustomFlags(customFlags, customFlagsDefault);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCustomDoubles(
        vector <string> &customDoubles,
        vector <RangedMeasurement_t> &customDoublesRanges,
        vector <double> &customDoublesDefault) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCustomDoubles(customDoubles, customDoublesRanges,customDoublesDefault);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasNanionTemperatureController() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasNanionTemperatureController();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getTemperatureControllerRange(
        int &minTemperature,
        int &maxTemperature) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getTemperatureControllerRange(minTemperature, maxTemperature);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t hasWasherControls() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasWasherControls();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getWasherSpeedRange(
        RangedMeasurement_t &range) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getWasherSpeedRange(range);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getWasherStatus(
        WasherStatus_t &status,
        WasherError_t &error) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getWasherStatus(status, error);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getWasherPresetSpeeds(
        vector <int8_t> &speedValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getWasherPresetSpeeds(speedValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}


ErrorCodes_t hasCFastCompensation() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->hasCFastCompensation();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCFastCompensationOptions(
        vector <string> &options) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCFastCompensationOptions(options);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCFastCapacitanceControl(
        CompensationControl_t &control) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCFastCapacitanceControl(control);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageOffsetCompensations(
        vector <Measurement_t> &offsets) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->updateVoltageOffsetCompensations(offsets);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

} // namespace er4CommLib
