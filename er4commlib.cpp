//  Copyright (C) 2021 Filippo Cona
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

#include "er4commlib.h"

#include <algorithm>

#include "messagedispatcher.h"
#include "messagedispatcher_e1plus.h"
#include "messagedispatcher_e1light.h"
#include "messagedispatcher_e1hc.h"
#include "messagedispatcher_enpr.h"
#include "messagedispatcher_enpr_hc.h"
#include "messagedispatcher_e2hc.h"
#include "messagedispatcher_e4n.h"
#include "messagedispatcher_e4e.h"
#include "messagedispatcher_e16illumina.h"
#include "messagedispatcher_e16n.h"
#include "messagedispatcher_e16e.h"
#include "messagedispatcher_e16eth.h"
#include "messagedispatcher_el06b.h"
#include "messagedispatcher_el06c.h"
#include "messagedispatcher_el06d_el06e.h"
#include "messagedispatcher_fake_e16n.h"
#include "messagedispatcher_e16hc.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include "ftd2xx_win.h"

using namespace std;

static MessageDispatcher * messageDispatcher = nullptr;
static uint16_t * buffer_l;

/*! Private functions prototypes */
string getDeviceSerial(
        uint32_t index);

bool getDeviceCount(
        DWORD &numDevs);

namespace er4CommLib {

/*****************\
 *  Ctor / Dtor  *
\*****************/

ErrorCodes_t init() {
    buffer_l = new (nothrow) uint16_t [ER4CL_DATA_ARRAY_SIZE];
    if (buffer_l == nullptr) {
        //        delete [] deviceId_l;
        //        deviceId_l = nullptr;
        return ErrorInitializationFailed;
    }

    return Success;
}

ErrorCodes_t deinit() {
    if (buffer_l != nullptr) {
        delete [] buffer_l;
        buffer_l = nullptr;
    }

    return Success;
}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t detectDevices(
        vector <string> &deviceIds) {
    /*! Gets number of devices */
    DWORD numDevs;
    bool devCountOk = getDeviceCount(numDevs);
    if (!devCountOk) {
        return ErrorListDeviceFailed;

    } else if (numDevs < 2) {
        /*! Each device has 2 channels */
        deviceIds.clear();
        return ErrorNoDeviceFound;
    }

    vector <string> deviceIdsTemp;
    deviceIds.clear();
    deviceIdsTemp.clear();
    string deviceName;

    /*! Lists all serial numbers */
    for (uint32_t i = 0; i < numDevs; i++) {
        deviceName = getDeviceSerial(i);
        if (find(deviceIdsTemp.begin(), deviceIdsTemp.end(), deviceName) == deviceIdsTemp.end()) {
            /*! Devices with an open channel are detected wrongly and their name is an empty string */
            if (deviceName.size() > 0) {
                /*! If this device has been found for the first time put it in the temporary list */
                deviceIdsTemp.push_back(getDeviceSerial(i));
            }

        } else {
            /*! Devices with an open channel are detected wrongly and their name is an empty string */
            if (deviceName.size() > 0) {
                /*! If this device has been already been found both channels A and B are detected, so add it in the output list */
                deviceIds.push_back(getDeviceSerial(i));
            }
        }
    }

    return Success;
}

ErrorCodes_t connect(
        string deviceId) {

    ErrorCodes_t ret = Success;
    if (messageDispatcher == nullptr) {
        /*! Initializes eeprom */
        /*! \todo FCON questa info dovrà essere appresa dal device detector e condivisa qui dal metodo connect */
        FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
        if (deviceId == "e16 Demo") {
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

        DeviceTuple_t deviceTuple = ftdiEeprom->getDeviceTuple();
        DeviceTypes_t deviceType;

        ret = MessageDispatcher::getDeviceType(deviceTuple, deviceType);
        if (ret != Success) {
            if (ftdiEeprom != nullptr) {
                delete ftdiEeprom;
            }
            return ErrorDeviceTypeNotRecognized;
        }

        switch (deviceType) {
        case DeviceE1PlusEL03fEDR3:
            messageDispatcher = new MessageDispatcher_e1Plus_El03f_LegacyEdr3_V00(deviceId);
            break;

        case DeviceE1LightEL03fEDR3:
            messageDispatcher = new MessageDispatcher_e1Light_El03f_LegacyEdr3_V01(deviceId);
            break;

        case DeviceE16eEDR3:
            messageDispatcher = new MessageDispatcher_e16e_LegacyEdr3_V00(deviceId);
            break;

        case DeviceENPR:
            messageDispatcher = new MessageDispatcher_eNPR(deviceId);
            break;

        case DeviceENPRHC:
            messageDispatcher = new MessageDispatcher_eNPR_HC_V00(deviceId);
            break;

        case DeviceE4nV04EDR3:
            messageDispatcher = new MessageDispatcher_e4n_El03c_LegacyEdr3_V04(deviceId);
            break;

        case DeviceE4eEDR3:
            messageDispatcher = new MessageDispatcher_e4e_El03c_LegacyEdr3_V00(deviceId);
            break;

        case DeviceE4e:
            messageDispatcher = new MessageDispatcher_e4e(deviceId);
            break;

        case DeviceE16Illumina:
            messageDispatcher = new MessageDispatcher_e16Illumina_V01(deviceId);
            break;

        case DeviceE16IlluminaEDR3:
            messageDispatcher = new MessageDispatcher_e16Illumina_LegacyEdr3_V03(deviceId);
            break;

        case DeviceE16n:
            messageDispatcher = new MessageDispatcher_e16n(deviceId);
            break;

        case DeviceE16ETHEDR3:
            messageDispatcher = new MessageDispatcher_e16ETH_LegacyEdr3_V01(deviceId);
            break;

        case DeviceE16HC:
            messageDispatcher = new MessageDispatcher_e16HC(deviceId);
            break;

        case DeviceDlp:
            messageDispatcher = new MessageDispatcher_dlp(deviceId);
            break;

        case TestboardEL06b:
            messageDispatcher = new MessageDispatcher_EL06b(deviceId);
            break;

        case TestboardEL06c:
            messageDispatcher = new MessageDispatcher_EL06c(deviceId);
            break;

        case TestboardEL06dEL06e:
            messageDispatcher = new MessageDispatcher_EL06d_EL06e(deviceId);
            break;

        case DeviceE2HCExtAdc:
            messageDispatcher = new MessageDispatcher_e2HC_V00(deviceId);
            break;

        case DeviceE2HCIntAdc:
            messageDispatcher = new MessageDispatcher_e2HC_V01(deviceId);
            break;

        case DeviceENPRFairyLight:
            messageDispatcher = new MessageDispatcher_eNPR_FL(deviceId);
            break;

        case DeviceFakeE16n:
            messageDispatcher = new MessageDispatcher_fake_e16n(deviceId);
            break;

        default:
            if (ftdiEeprom != nullptr) {
                delete ftdiEeprom;
            }
            return ErrorDeviceTypeNotRecognized;
        }

        if (messageDispatcher != nullptr) {
            ret = messageDispatcher->init();
            if (ret != Success) {
                return ret;
            }

            ret = messageDispatcher->connect(ftdiEeprom);

            if (ret != Success) {
                messageDispatcher->disconnect();
                delete messageDispatcher;
                messageDispatcher = nullptr;
            }
        }

    } else {
        ret = ErrorDeviceAlreadyConnected;
    }
    return ret;
}

ErrorCodes_t disconnect() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->disconnect();
        if (ret == Success) {
            messageDispatcher->deinit();
            delete messageDispatcher;
            messageDispatcher = nullptr;
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

/****************\
 *  Tx methods  *
\****************/

ErrorCodes_t sendCommands() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->sendCommands();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t selectVoltageProtocol(
        unsigned int idx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->selectVoltageProtocol(idx);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyVoltageProtocol() {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->applyVoltageProtocol();

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setProtocolVoltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolTime(
        unsigned int idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setProtocolTime(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolSlope(
        unsigned int idx,
        Measurement_t slope) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setProtocolSlope(idx, slope);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolAdimensional(
        unsigned int idx,
        Measurement_t adimensional) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setProtocolAdimensional(idx, adimensional);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkSelectedProtocol(
        unsigned int idx,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkSelectedProtocol(idx, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkProtocolVoltage(idx, voltage, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolTime(
        unsigned int idx,
        Measurement_t time,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkProtocolTime(idx, time, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolSlope(
        unsigned int idx,
        Measurement_t slope,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkProtocolSlope(idx, slope, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolAdimensional(
        unsigned int idx,
        Measurement_t adimensional,
        std::string &message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->checkProtocolAdimensional(idx, adimensional, message);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
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


ErrorCodes_t setVoltageDacExt(
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVoltageDacExt(voltage);

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

ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(
        unsigned int idx,
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
        unsigned int idx,
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
        unsigned int idx,
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
        unsigned int idx,
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
        unsigned int idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Duration(idx, time);

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

ErrorCodes_t switchVcSel0(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->switchVcSel0(on);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t switchVcSel1(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->switchVcSel1(on);

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
        buffer = buffer_l;
        return messageDispatcher->getDataPackets(buffer, dataToRead, &dataRead);

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
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageRanges(voltageRanges, defaultOption);

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
        uint16_t &defaultOption) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageStimulusLpfs(filterOptions, defaultOption);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceLpfs(
        vector <Measurement_t> &filterOptions,
        uint16_t &defaultOption) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageReferenceLpfs(filterOptions, defaultOption);

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
        vector <vector <uint16_t>> &adimensionals) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolList(names, images, voltages, times, slopes, adimensionals);

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

ErrorCodes_t getReferencePulseControls(RangedMeasurement_t &voltageRange,
        RangedMeasurement_t &durationRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getReferencePulseControls(voltageRange, durationRange);

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

ErrorCodes_t getDacExtRange(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getDacExtRange(range, defaultValue);

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

/*! Private functions */
string getDeviceSerial(
        uint32_t index) {
    char buffer[64];
    string serial;
    FT_STATUS FT_Result = FT_ListDevices((PVOID)index, buffer, FT_LIST_BY_INDEX);
    if (FT_Result == FT_OK) {
        serial = buffer;
        return serial.substr(0, serial.size()-1); /*!< Removes channel character */

    } else {
        return "";
    }
}

bool getDeviceCount(
        DWORD &numDevs) {
    /*! Get the number of connected devices */
    numDevs = 0;
    FT_STATUS FT_Result = FT_ListDevices(&numDevs, nullptr, FT_LIST_NUMBER_ONLY);
    if (FT_Result == FT_OK) {
        return true;

    } else {
        return false;
    }
}
