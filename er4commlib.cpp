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
#include <vector>
#include <string>

#include "messagedispatcher.h"
#include "messagedispatcher_e1plus.h"
#include "messagedispatcher_e1light.h"
#include "messagedispatcher_e1hc.h"
#include "messagedispatcher_enpr.h"
#include "messagedispatcher_enpr_hc.h"
#include "messagedispatcher_e2hc.h"
#include "messagedispatcher_e4n.h"
#include "messagedispatcher_e4e.h"
#include "messagedispatcher_e16fastpulses.h"
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

/*! Private functions prototypes */
string getDeviceSerial(
        uint32_t index);

bool getDeviceCount(
        DWORD &numDevs);

/*****************\
 *  Ctor / Dtor  *
\*****************/

ErrorCodes_t init() {


    return Success;
}

ErrorCodes_t deinit() {

    return Success;
}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t detectDevices(
        char * deviceIds) {
    /*! Gets number of devices */
    DWORD numDevs;
    bool devCountOk = getDeviceCount(numDevs);
    if (!devCountOk) {
        return ErrorListDeviceFailed;

    } else if (numDevs < 2) {
        /*! Each device has 2 channels */

        return ErrorNoDeviceFound;
    }

    string deviceName;

    /*! Lists all serial numbers */
    for (uint32_t i = 0; i < numDevs; i++) {
        deviceName = getDeviceSerial(i);
        sprintf(deviceIds, "%s", deviceName.c_str());
    }

    return Success;
}

ErrorCodes_t connectDevice(
        char*  deviceId) {

    ErrorCodes_t ret = Success;
    if (messageDispatcher == nullptr) {
        string deviceIdStr(deviceId);


        /*! Initializes eeprom */
        /*! \todo FCON questa info dovrà essere appresa dal device detector e condivisa qui dal metodo connect */

        FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
        if ( deviceIdStr=="e16 Demo") {
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

        case DeviceE16FastPulses:
            messageDispatcher = new MessageDispatcher_e16FastPulses_V01(deviceId);
            break;

        case DeviceE16FastPulsesEDR3:
            messageDispatcher = new MessageDispatcher_e16FastPulses_LegacyEdr3_V03(deviceId);
            break;

        case DeviceE16n:
            messageDispatcher = new MessageDispatcher_e16n(deviceId);
            break;

        case DeviceE16ETHEDR3:
            messageDispatcher = new MessageDispatcher_e16ETH_LegacyEdr3_V01(deviceId);
            break;

        case DeviceE16HC_V01:
            messageDispatcher = new MessageDispatcher_e16HC_V01(deviceId);
            break;

        case DeviceE16HC_V02:
            messageDispatcher = new MessageDispatcher_e16HC_V02(deviceId);
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
        MeasurementReduced_t voltageRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        voltage=fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->setProtocolVoltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolTime(
        unsigned int idx,
        MeasurementReduced_t timeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t time;
        time=fromReduceMeasurement(timeRed);
        ret = messageDispatcher->setProtocolTime(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolSlope(
        unsigned int idx,
        MeasurementReduced_t slopeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t slope;
        slope=fromReduceMeasurement(slopeRed);
        ret = messageDispatcher->setProtocolSlope(idx, slope);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setProtocolAdimensional(
        unsigned int idx,
        MeasurementReduced_t adimensionalRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t adimensional;
        adimensional=fromReduceMeasurement(adimensionalRed);

        ret = messageDispatcher->setProtocolAdimensional(idx, adimensional);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkSelectedProtocol(
        unsigned int idx,
        char * message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        string messageStr;
        int messageOffset =0;
        ret=messageDispatcher->checkSelectedProtocol(idx, messageStr);

        for(uint32_t charIdx=0;charIdx<messageStr.length();charIdx++){
            message[messageOffset++]=messageStr[charIdx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolVoltage(
        unsigned int idx,
        MeasurementReduced_t voltageRed,
        char * message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        Measurement_t voltage;
        std::string messageStr;
        int messageOffset =0;
        voltage = fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->checkProtocolVoltage(idx, voltage, messageStr);

        for(uint32_t charIdx =0; charIdx<messageStr.length();charIdx++ ){
            message[messageOffset++] = messageStr[charIdx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolTime(
        unsigned int idx,
        MeasurementReduced_t timeRed,
        char * message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        Measurement_t time;
        string messageStr;
        int messageOffset = 0;
        time = fromReduceMeasurement(timeRed);
        ret = messageDispatcher->checkProtocolTime(idx, time, messageStr);

        for(uint32_t charIdx=0; charIdx<messageStr.length();charIdx++){
            message[messageOffset++]= messageStr[charIdx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolSlope(
        unsigned int idx,
        MeasurementReduced_t slopeRed,
        char * message) {

    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t slope;
        string messageStr;
        int messageOffset = 0;
        slope = fromReduceMeasurement(slopeRed);
        ret = messageDispatcher->checkProtocolSlope(idx, slope, messageStr);

        for(uint32_t charIdx=0; charIdx<messageStr.length(); charIdx++){
            message[messageOffset++]= messageStr[charIdx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkProtocolAdimensional(
        unsigned int idx,
        MeasurementReduced_t adimensionalRed,
        char * message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t adimensional;
        string messageStr;
        int messageOffset=0;
        adimensional = fromReduceMeasurement(adimensionalRed);
        ret = messageDispatcher->checkProtocolAdimensional(idx, adimensional, messageStr);

        for(uint32_t charIdx=0; charIdx<messageStr.length();charIdx++){
            message[messageOffset++]=messageStr[charIdx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVoltageOffset(
        unsigned int idx,
        MeasurementReduced_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltageOffset;
        voltageOffset =fromReduceMeasurement(voltage);
        ret = messageDispatcher->setVoltageOffset(idx, voltageOffset);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t checkVoltageOffset(
        unsigned int idx,
        MeasurementReduced_t voltageRed,
        char * message) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        Measurement_t voltage;
        string messageStr;
        int messageOffset = 0;
        voltage =fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->checkVoltageOffset(idx,voltage, messageStr);
        for(uint32_t charIdx=0; charIdx<messageStr.length();charIdx++){
            message[messageOffset++]=messageStr[charIdx];
        }
    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyInsertionPulse(
        MeasurementReduced_t voltageRed,
        MeasurementReduced_t durationRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        Measurement_t duration;
        voltage=fromReduceMeasurement(voltageRed);
        duration=fromReduceMeasurement(durationRed);
        ret = messageDispatcher->applyInsertionPulse(voltage, duration);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyReferencePulse(
        MeasurementReduced_t voltageRed,
        MeasurementReduced_t durationRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        Measurement_t duration;
        voltage=fromReduceMeasurement(voltageRed);
        duration=fromReduceMeasurement(durationRed);
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
        MeasurementReduced_t cutoffFrequencyRed,
        bool lowPassFlag,
        bool activeFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t cutoffFrequency;
        cutoffFrequency=fromReduceMeasurement(cutoffFrequencyRed);
        ret = messageDispatcher->setRawDataFilter(cutoffFrequency, lowPassFlag, activeFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t applyDacExt(
        MeasurementReduced_t voltageRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        voltage=fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->applyDacExt(voltage);

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
        int8_t * speedValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector<int8_t> speedValuesVec;
        uint32_t speedValuesOffset = 0;
        uint32_t protocolsNum= speedValuesVec.size();

        ret = messageDispatcher->setWasherPresetSpeeds(speedValuesVec);
        for(uint32_t speedValuesIdx =0; speedValuesIdx< protocolsNum; speedValuesIdx++){
                speedValues[speedValuesOffset++] = speedValuesVec[speedValuesIdx];

        }


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
        unsigned int idx,
        MeasurementReduced_t voltageRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        voltage=fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->setFastReferencePulseProtocolWave1Voltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave1Time(
        unsigned int idx,
        MeasurementReduced_t timeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t time;
        time=fromReduceMeasurement(timeRed);
        ret = messageDispatcher->setFastReferencePulseProtocolWave1Time(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(
        unsigned int idx,
        MeasurementReduced_t voltageRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t voltage;
        voltage=fromReduceMeasurement(voltageRed);
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Voltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Time(
        unsigned int idx,
        MeasurementReduced_t timeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t time;
        time=fromReduceMeasurement(timeRed);
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Time(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFastReferencePulseProtocolWave2Duration(
        unsigned int idx,
        MeasurementReduced_t timeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t time;
        time=fromReduceMeasurement(timeRed);
        ret = messageDispatcher->setFastReferencePulseProtocolWave2Duration(idx, time);

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
        MeasurementReduced_t valueRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t value;
        value=fromReduceMeasurement(valueRed);
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

ErrorCodes_t getDeviceInfoAlreadyConnect(
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
        char * deviceId,
        uint8_t &deviceVersion,
        uint8_t &deviceSubversion,
        uint32_t &firmwareVersion) {
    ErrorCodes_t ret = Success;
    string deviceIdStr(deviceId);
    /*! Initializes eeprom */
    /*! \todo FCON questa info dovrà essere appresa dal device detector e condivisa qui dal metodo connect */
    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
    if (deviceIdStr == "ePatch Demo") {
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
        int16_t buffer []) {
    if (messageDispatcher != nullptr) {
        uint16_t * tempBuffer = (uint16_t *)buffer;
        return messageDispatcher->getDataPackets(tempBuffer, dataToRead, &dataRead);

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
        RangedMeasurementReduced_t currentRangesRed[],
        uint16_t defaultOptions[]) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector <RangedMeasurement_t> currentRangesVec;
        vector <uint16_t> defaultOptionsVec;

        ret = messageDispatcher->getCurrentRanges(currentRangesVec, defaultOptionsVec);
        uint32_t size = currentRangesVec.size();
        for (uint32_t idx = 0; idx < size; idx++) {
            currentRangesRed[idx] = toReduceRangedMeasurement(currentRangesVec[idx]);
            defaultOptions[idx] = defaultOptionsVec[idx];
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCurrentRange(
        RangedMeasurementReduced_t &currentRangeRed,
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t currentRange;
        ret = messageDispatcher->getCurrentRange(currentRange, channelIdx);
        currentRangeRed = toReduceRangedMeasurement(currentRange);
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
        RangedMeasurementReduced_t voltageRangesRed[],
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector <RangedMeasurement_t> voltageRangesVec;

        ret = messageDispatcher->getVoltageRanges(voltageRangesVec, defaultOption);
        uint32_t size = voltageRangesVec.size();
        for (uint32_t idx = 0; idx < size; idx++) {
            voltageRangesRed[idx] = toReduceRangedMeasurement(voltageRangesVec[idx]);
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageRange(
        RangedMeasurementReduced_t &voltageRangeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t voltageRange;
        ret = messageDispatcher->getVoltageRange(voltageRange);
        voltageRangeRed=toReduceRangedMeasurement(voltageRange);
    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceRanges(
        vector <RangedMeasurementReduced_t> &rangesRed,
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector <RangedMeasurement_t> rangesVec;

        ret = messageDispatcher->getVoltageReferenceRanges(rangesVec, defaultOption);
        uint32_t size = rangesVec.size();
        for (uint32_t idx = 0; idx < size; idx++) {
            rangesRed[idx] = toReduceRangedMeasurement(rangesVec[idx]);
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSamplingRates(
        MeasurementReduced_t samplingRatesRed[],
        uint16_t &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t * samplingRates;
        vector <Measurement_t> samplingRatesVec;
        ret = messageDispatcher->getSamplingRates(samplingRatesVec, defaultOption);
        int size = samplingRatesVec.size();
        for (uint32_t idx = 0; idx < size; idx++ ) {
            samplingRates[idx] = samplingRatesVec[idx];
            samplingRatesRed[idx]= toReduceMeasurement(samplingRates[idx]);
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSamplingRate(
        MeasurementReduced_t &samplingRateRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t samplingRate;
        ret = messageDispatcher->getSamplingRate(samplingRate);
        samplingRateRed=toReduceMeasurement(samplingRate);
    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getRealSamplingRates(
        MeasurementReduced_t * samplingRatesRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        Measurement_t * samplingRates;
        vector<Measurement_t> samplingRatesVec;
        ret = messageDispatcher->getRealSamplingRates(samplingRatesVec);
        int size = samplingRatesVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            samplingRates[idx] = samplingRatesVec[idx];
            samplingRatesRed[idx]= toReduceMeasurement(samplingRates[idx]);
        }

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
        uint16_t * oversamplingRatios) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector<uint16_t> overSamplingRatiosVec;
        ret = messageDispatcher->getOversamplingRatios(overSamplingRatiosVec);

        int size = overSamplingRatiosVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            oversamplingRatios[idx] = overSamplingRatiosVec[idx];
        }

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
        MeasurementReduced_t * filterOptionsRed,
        uint16_t &defaultOption) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        Measurement_t * filterOptions;
        vector<Measurement_t> filterOptionsVec;
        ret = messageDispatcher->getVoltageStimulusLpfs(filterOptionsVec, defaultOption);
        int size = filterOptionsVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            filterOptions[idx] = filterOptionsVec[idx];
            filterOptionsRed[idx]= toReduceMeasurement(filterOptions[idx]);
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceLpfs(
        MeasurementReduced_t *voltageReferenceRed,
        uint16_t &defaultOption) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (messageDispatcher != nullptr) {
        Measurement_t * voltageReference;
        vector<Measurement_t> voltageReferenceVec;
        ret = messageDispatcher->getVoltageReferenceLpfs(voltageReferenceVec, defaultOption);
        int size= voltageReferenceVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            voltageReference[idx] = voltageReferenceVec[idx];
            voltageReferenceRed[idx]= toReduceMeasurement(voltageReference[idx]);
        }

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
        char names[],
        char images[],
        int16_t voltages[],
        int16_t times[],
        int16_t slopes[],
        int16_t adimensionals[]) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        vector<string> vectorNames;
        vector<string> vectorImages;
        vector <vector <uint16_t>> vectorVoltages;
        vector <vector <uint16_t>> vectorTimes;
        vector <vector <uint16_t>> vectorSlopes;
        vector <vector <uint16_t>> vectorAdimensional;
        uint32_t nameOffset = 0;
        uint32_t imagesOffset = 0;
        uint32_t voltagesOffset = 0;
        uint32_t timesOffset = 0;
        uint32_t slopesOffset = 0;
        uint32_t adimensionalOffset = 0;

        ret = messageDispatcher->getProtocolList(vectorNames, vectorImages, vectorVoltages, vectorTimes, vectorSlopes, vectorAdimensional);

        uint32_t protocolsNum=vectorNames.size();

        for(uint32_t namesIdx =0; namesIdx< protocolsNum; namesIdx++){
            for(uint32_t charIdx =0; charIdx<vectorNames[namesIdx].length();charIdx++ ){
                names[nameOffset++] = vectorNames[namesIdx][charIdx];
            }
            names[nameOffset++] = ',';
        }

        for(uint32_t imagesIdx =0; imagesIdx< protocolsNum; imagesIdx++){
            for(uint32_t charIdx =0; charIdx<vectorImages[imagesIdx].length();charIdx++ ){
                images[nameOffset++] = vectorImages[imagesIdx][charIdx];
            }
            images[imagesOffset++] = ',';
        }

        for(uint32_t voltagesIdx =0; voltagesIdx< protocolsNum; voltagesIdx++){
            for(uint32_t elementIdx =0; elementIdx< vectorVoltages[voltagesIdx].size(); elementIdx++ ){
                voltages[voltagesOffset++] = vectorVoltages[voltagesIdx][elementIdx];
            }
            voltages[voltagesOffset++] = -1;
        }

        for(uint32_t timesIdx =0; timesIdx< protocolsNum; timesIdx++){
            for(uint32_t elementIdx =0; elementIdx< vectorTimes[timesIdx].size(); elementIdx++ ){
                times[timesOffset++] = vectorTimes[timesIdx][elementIdx];
            }
            voltages[voltagesOffset++] = -1;
        }

        for(uint32_t slopeIdx =0; slopeIdx< protocolsNum; slopeIdx++){
            for(uint32_t elementIdx =0; elementIdx< vectorSlopes[slopeIdx].size(); elementIdx++ ){
                slopes[slopesOffset++] = vectorSlopes[slopeIdx][elementIdx];
            }
            slopes[slopesOffset++] = -1;
        }

        for(uint32_t adimensionalIdx =0; adimensionalIdx< protocolsNum; adimensionalIdx++){
            for(uint32_t elementIdx =0; elementIdx< vectorAdimensional[adimensionalIdx].size(); elementIdx++ ){
                adimensionals[adimensionalOffset++] = vectorAdimensional[adimensionalIdx][elementIdx];
            }
            adimensionals[adimensionalOffset++] = -1;
        }


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
        char voltageNames[],
        RangedMeasurementReduced_t rangesRed[],
        MeasurementReduced_t defaultValuesRed[]) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector <string> voltageNamesVec;
        vector <RangedMeasurement_t> rangesVec;
        vector <Measurement_t> defaultValuesVec;
        uint32_t voltageNameOffset = 0;

        ret = messageDispatcher->getProtocolVoltage(voltageNamesVec, rangesVec, defaultValuesVec);
        if (ret == Success) {
            int size = voltageNamesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                for (uint32_t charIdx = 0; charIdx < voltageNamesVec[idx].length(); charIdx++) {
                    voltageNames[voltageNameOffset++] = voltageNamesVec[idx][charIdx];
                }
                voltageNames[voltageNameOffset++] = ',';
            }

            size = rangesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                rangesRed[idx] = toReduceRangedMeasurement(rangesVec[idx]);
            }
            size = defaultValuesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                defaultValuesRed[idx] = toReduceMeasurement(defaultValuesVec[idx]);
            }

        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolTime(
        char timeNames[],
        RangedMeasurementReduced_t rangesRed[],
        MeasurementReduced_t defaultValuesRed[]) {
    ErrorCodes_t ret;
    vector <string> timeNamesVec;
    vector <RangedMeasurement_t> rangesVec;
    vector <Measurement_t> defaultValuesVec;
    uint32_t timeNamesOffset = 0;

    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolTime(timeNamesVec, rangesVec, defaultValuesVec);

        if (ret == Success) {
            int size = timeNamesVec.size();
            for (uint32_t idx = 0; idx< size; idx++){
                for (uint32_t charIdx = 0; charIdx < timeNamesVec[idx].length(); charIdx++) {
                    timeNamesVec[timeNamesOffset++] = timeNamesVec[idx][charIdx];
                }
                timeNames[timeNamesOffset++] = ',';
            }

            size = rangesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                rangesRed[idx] = toReduceRangedMeasurement(rangesVec[idx]);
            }
            size = defaultValuesVec.size();
            for (uint32_t idx = 0; idx < size; idx++ ) {
                defaultValuesRed[idx] = toReduceMeasurement(defaultValuesVec[idx]);
            }
        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolSlope(
        char slopeNames[],
        RangedMeasurementReduced_t rangesRed[],
        MeasurementReduced_t defaultValuesRed[]) {
    ErrorCodes_t ret;
    vector <string> slopeNamesVec;
    vector <RangedMeasurement_t> rangesVec;
    vector <Measurement_t> defaultValuesVec;
    uint32_t slopeNamesOffset = 0;

    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolSlope(slopeNamesVec, rangesVec, defaultValuesVec);

        if (ret == Success) {
            int size = slopeNamesVec.size();
            for (uint32_t idx = 0; idx< size; idx++){
                for (uint32_t charIdx = 0; charIdx < slopeNamesVec[idx].length(); charIdx++) {
                    slopeNames[slopeNamesOffset++] = slopeNamesVec[idx][charIdx];
                }
                slopeNames[slopeNamesOffset++] = ',';
            }

            size = rangesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                rangesRed[idx] = toReduceRangedMeasurement(rangesVec[idx]);
            }
            size = defaultValuesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                defaultValuesRed[idx] = toReduceMeasurement(defaultValuesVec[idx]);
            }

        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolAdimensional(
        char adimensionalNames[],
        RangedMeasurementReduced_t rangesRed[],
        MeasurementReduced_t defaultValuesRed[]) {
    ErrorCodes_t ret;
    vector <string> adimensionalNamesVec;
    vector <RangedMeasurement_t> rangesVec;
    vector <Measurement_t> defaultValuesVec;
    uint32_t adimensionalNamesOffset = 0;

    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolAdimensional(adimensionalNamesVec, rangesVec, defaultValuesVec);

        if (ret == Success) {
            int size = adimensionalNamesVec.size();
            for (uint32_t idx = 0; idx< size; idx++) {
                for (uint32_t charIdx =0; charIdx < adimensionalNamesVec[idx].length();charIdx++) {
                    adimensionalNames[adimensionalNamesOffset++] = adimensionalNamesVec[idx][charIdx];
                }
                adimensionalNames[adimensionalNamesOffset++] = ',';
            }

            size = rangesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                rangesRed[idx] = toReduceRangedMeasurement(rangesVec[idx]);
            }
            size = defaultValuesVec.size();
            for (uint32_t idx = 0; idx < size; idx++) {
                defaultValuesRed[idx] = toReduceMeasurement(defaultValuesVec[idx]);
            }

        }

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageOffsetControls(
        RangedMeasurementReduced_t &voltageRangeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t voltageRange;
        ret = messageDispatcher->getVoltageOffsetControls(voltageRange);
        voltageRangeRed=toReduceRangedMeasurement(voltageRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getInsertionPulseControls(RangedMeasurementReduced_t &voltageRangeRed,
                                       RangedMeasurementReduced_t &durationRangeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t voltageRange;
        RangedMeasurement_t durationRange;
        ret = messageDispatcher->getInsertionPulseControls(voltageRange, durationRange);

        voltageRangeRed=toReduceRangedMeasurement(voltageRange);
        durationRangeRed=toReduceRangedMeasurement(durationRange);

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

ErrorCodes_t getReferencePulseControls(RangedMeasurementReduced_t &voltageRangeRed,
                                       RangedMeasurementReduced_t &durationRangeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        RangedMeasurement_t voltageRange;
        RangedMeasurement_t durationRange;
        ret = messageDispatcher->getReferencePulseControls(voltageRange, durationRange);
        voltageRangeRed=toReduceRangedMeasurement(voltageRange);
        durationRangeRed=toReduceRangedMeasurement(durationRange);


    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}


ErrorCodes_t getEdhFormat(
        char * format) {
    ErrorCodes_t ret;
    string formatStr;
    int formatOffset=0;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getEdhFormat(formatStr);
        for(uint32_t charIdx=0; charIdx<formatStr.length();charIdx++){
            format[formatOffset++]=formatStr[charIdx];
        }


    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getRawDataFilterCutoffFrequency(
        RangedMeasurementReduced_t &rangeRed,
        MeasurementReduced_t &defaultValueRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t range;
        Measurement_t defaultValue;

        ret = messageDispatcher->getRawDataFilterCutoffFrequency(range, defaultValue);

        rangeRed=toReduceRangedMeasurement(range);
        defaultValueRed=toReduceMeasurement(defaultValue);

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
        uint32_t * ledsColors) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        //        ret = messageDispatcher->getLedsColors(ledsColors);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave1Range(
        RangedMeasurementReduced_t &voltageRangeRed,
        RangedMeasurementReduced_t &timeRangeRed,
        uint16_t &nPulse) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        RangedMeasurement_t voltageRange;
        RangedMeasurement_t timeRange;
        ret = messageDispatcher->getFastReferencePulseProtocolWave1Range(voltageRange, timeRange, nPulse);
        voltageRangeRed=toReduceRangedMeasurement(voltageRange);
        timeRangeRed=toReduceRangedMeasurement(timeRange);
    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave2Range(
        RangedMeasurementReduced_t &voltageRangeRed,
        RangedMeasurementReduced_t &timeRangeRed,
        RangedMeasurementReduced_t &durationRangeRed,
        uint16_t &nPulse) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t voltageRange;
        RangedMeasurement_t timeRange;
        RangedMeasurement_t durationRange;
        ret = messageDispatcher->getFastReferencePulseProtocolWave2Range(voltageRange, timeRange, durationRange, nPulse);

        voltageRangeRed=toReduceRangedMeasurement(voltageRange);
        timeRangeRed=toReduceRangedMeasurement(timeRange);
        durationRangeRed=toReduceRangedMeasurement(durationRange);
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
        RangedMeasurementReduced_t &rangeRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        RangedMeasurement_t range;
        ret = messageDispatcher->getWasherSpeedRange(range);
        rangeRed=toReduceRangedMeasurement(range);
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
        int8_t * speedValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        vector<int8_t> speedValueVec;
        ret = messageDispatcher->getWasherPresetSpeeds(speedValueVec);
        int size=speedValueVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            speedValue[idx] = speedValueVec[idx];
        }
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
        char *options) {
    ErrorCodes_t ret;
    vector<string> optionsVec;
    uint32_t optionOffset=0;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCFastCompensationOptions(optionsVec);

        if(ret==Success){
            int size= optionsVec.size();
            for(uint32_t idx=0;idx<size;idx++){
                for(uint32_t charIdx=0; charIdx<optionsVec[idx].length(); charIdx++){
                    options[optionOffset++]=optionsVec[idx][charIdx];
                }
            }
        }

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
        MeasurementReduced_t* offsetsRed) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {

        Measurement_t* offsets;
        vector<Measurement_t> offsetsVec;
        ret = messageDispatcher->updateVoltageOffsetCompensations(offsetsVec);

        int size= offsetsVec.size();
        for(uint32_t idx = 0; idx < size; idx++ ) {
            offsets[idx] = offsetsVec[idx];
            offsetsRed[idx]= toReduceMeasurement(offsets[idx]);
        }
    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}


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
