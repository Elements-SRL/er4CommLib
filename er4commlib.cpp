#include "e4ocommlib.h"

#include <algorithm>

#include "messagedispatcher.h"
#include "messagedispatcher_enpr_nooma.h"
#include "messagedispatcher_fake_nooma.h"

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

namespace e4oCommLib {

/*****************\
 *  Ctor / Dtor  *
\*****************/

ErrorCodes_t init() {
    buffer_l = new (nothrow) uint16_t [E4OCL_DATA_ARRAY_SIZE];
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

    } else if (numDevs == 0) {
        deviceIds.clear();
        return ErrorNoDeviceFound;
    }

    deviceIds.clear();
    string deviceName;

    /*! Lists all serial numbers */
    for (uint32_t i = 0; i < numDevs; i++) {
        deviceName = getDeviceSerial(i);
        if (find(deviceIds.begin(), deviceIds.end(), deviceName) == deviceIds.end()) {
            /*! Adds only new devices (no distinction between channels A and B creates duplicates) */
            if (deviceName.size() > 0) {
                /*! Devices with an open channel are detected wrongly and their name is an empty string */
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
        if (deviceId == "eNPR Demo") {
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
        case DeviceEnprNooma:
            messageDispatcher = new MessageDispatcher_eNPR_Nooma(deviceId);
            break;

        case DeviceFakeEnprNooma:
            messageDispatcher = new MessageDispatcher_fake_Nooma(deviceId);
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

ErrorCodes_t setInterposerInserted(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setInterposerInserted(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setConditioningCheck(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setConditioningCheck(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setConditioningChannel(
        uint16_t channelIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setConditioningChannel(channelIdx);

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

ErrorCodes_t setTriangularVoltage(
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setTriangularVoltage(voltage);

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

ErrorCodes_t setTriangularTime(
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setTriangularTime(time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFsmFlag(
        unsigned int idx,
        bool flag,
        bool applyFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFsmFlag(idx, flag, applyFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFsmVoltage(
        unsigned int idx,
        Measurement_t voltage,
        bool applyFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFsmVoltage(idx, voltage, applyFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFsmThresholdCurrent(
        unsigned int idx,
        Measurement_t current,
        bool applyFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFsmThresholdCurrent(idx, current, applyFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFsmTime(
        unsigned int idx,
        Measurement_t time,
        bool applyFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFsmTime(idx, time, applyFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setFsmInteger(
        unsigned int idx,
        unsigned int value,
        bool applyFlag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setFsmInteger(idx, value, applyFlag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setMovingAverageFilterDuration(
        Measurement_t duration) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setMovingAverageFilterDuration(duration);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t set4thOrderFilterCutoffFrequency(
        Measurement_t cFrequency) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->set4thOrderFilterCutoffFrequency(cFrequency);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setRawDataFilterCutoffFrequency(
        Measurement_t cFrequency) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setRawDataFilterCutoffFrequency(cFrequency);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setConditioningProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setConditioningProtocolVoltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setCheckingProtocolVoltage(
        unsigned int idx,
        Measurement_t voltage) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setCheckingProtocolVoltage(idx, voltage);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setConditioningProtocolTime(
        unsigned int idx,
        Measurement_t time) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setConditioningProtocolTime(idx, time);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t activateFEResetDenoiser(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->activateFEResetDenoiser(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t activateDacIntFilter(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->activateDacIntFilter(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t activateDacExtFilter(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->activateDacExtFilter(flag);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t setVcmForce(
        bool flag) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->setVcmForce(flag);

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

ErrorCodes_t digitalOffsetCompensation(
        bool on) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->digitalOffsetCompensation(on);

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

ErrorCodes_t resetDevice(
        bool reset) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->resetDevice(reset);

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

//ErrorCodes_t CommLib::getDeviceInfo(
//        uint8_t &deviceVersion,
//        uint8_t &deviceSubversion,
//        uint32_t &firmwareVersion) {
//    ErrorCodes_t ret = Success;
//    if (messageDispatcher != nullptr) {
//        ret = messageDispatcher->getDeviceInfo(deviceVersion, deviceSubversion, firmwareVersion);

//    } else {
//        ret = ErrorDeviceNotConnected;
//    }
//    return ret;
//}

//ErrorCodes_t CommLib::getDeviceInfo(
//        string deviceId,
//        uint8_t &deviceVersion,
//        uint8_t &deviceSubversion,
//        uint32_t &firmwareVersion) {
//    ErrorCodes_t ret = Success;
//    /*! Initializes eeprom */
//    /*! \todo FCON questa info dovrà essere appresa dal device detector e condivisa qui dal metodo connect */
//    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;
//    if (deviceId == "ePatch Demo") {
//        ftdiEepromId = FtdiEepromIdDemo;
//    }

//    /*! ftdiEeprom is deleted by the messageDispatcher if one is created successfully */
//    FtdiEeprom * ftdiEeprom = nullptr;
//    switch (ftdiEepromId) {
//    case FtdiEepromId56:
//        ftdiEeprom = new FtdiEeprom56(deviceId);
//        break;

//    case FtdiEepromIdDemo:
//        ftdiEeprom = new FtdiEepromDemo(deviceId);
//        break;
//    }

//    if (ftdiEeprom != nullptr) {
//        DeviceTuple_t deviceTuple = ftdiEeprom->getDeviceTuple();

//        deviceVersion = deviceTuple.version;
//        deviceSubversion = deviceTuple.subversion;
//        firmwareVersion = deviceTuple.fwVersion;

//    } else {
//        ret = ErrorEepromNotRecognized;
//    }
//    return ret;
//}

ErrorCodes_t getQueueStatus(
        QueueStatus_t &status) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getQueueStatus(&(status.availableDataPackets), &(status.bufferOverflowFlag), &(status.lostDataFlag));

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getChannelsNumber(
        uint32_t &fsmStatesChannelsNum,
        uint32_t &voltageChannelsNum,
        uint32_t &currentChannelsNum) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getChannelsNumber(fsmStatesChannelsNum, voltageChannelsNum, currentChannelsNum);

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
        double &fltValue) {
    if (messageDispatcher != nullptr) {
        return messageDispatcher->convertCurrentValue(intValue, fltValue);

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
        vector <RangedMeasurement_t> &currentRanges) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCurrentRanges(currentRanges);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCurrentRange(
        RangedMeasurement_t &currentRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCurrentRange(currentRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCheckingCurrentRange(
        RangedMeasurement_t &currentRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCheckingCurrentRange(currentRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getConditioningCurrentRange(
        RangedMeasurement_t &currentRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getConditioningCurrentRange(currentRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getVoltageRanges(
        vector <RangedMeasurement_t> &voltageRanges) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getVoltageRanges(voltageRanges);

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

ErrorCodes_t getCheckingVoltageRange(
        RangedMeasurement_t &voltageRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCheckingVoltageRange(voltageRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getConditioningVoltageRange(
        RangedMeasurement_t &voltageRange) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getConditioningVoltageRange(voltageRange);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getSamplingRates(
        vector <Measurement_t> &samplingRates,
        unsigned int &defaultOption) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getSamplingRates(samplingRates, defaultOption);

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

ErrorCodes_t getVoltageStimulusLpfs(
        vector <string> &filterOptions) {
    ErrorCodes_t ret;
//    if (messageDispatcher != nullptr) {
//        ret = messageDispatcher->getVoltageStimulusLpfs(filterOptions);

//    } else {
//        ret = ErrorDeviceNotConnected;
//    }
    return ret;
}

ErrorCodes_t getLiquidJunctionControl(
        CompensationControl_t &control) {
    ErrorCodes_t ret;
//    if (messageDispatcher != nullptr) {
//        ret = messageDispatcher->getLiquidJunctionControl(control);

//    } else {
//        ret = ErrorDeviceNotConnected;
//    }
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

ErrorCodes_t getProtocolList(
        vector <string> &protocolsNames,
        unsigned int &triangularIdx) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolList(protocolsNames, triangularIdx);

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

ErrorCodes_t getTriangularVoltage(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getTriangularVoltage(range, defaultValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getProtocolTime(
        vector <string> &timeNames,
        RangedMeasurement_t &range,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getProtocolTime(timeNames, range, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getTriangularTime(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getTriangularTime(range, defaultValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFsmFlag(
        vector <string> &flagNames,
        vector <bool> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFsmFlag(flagNames, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFsmVoltage(
        vector <string> &voltageNames,
        RangedMeasurement_t &range,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFsmVoltage(voltageNames, range, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFsmThresholdCurrent(
        vector <string> &currentNames,
        RangedMeasurement_t &range,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFsmThresholdCurrent(currentNames, range, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFsmTime(
        vector <string> &timeNames,
        RangedMeasurement_t &range,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFsmTime(timeNames, range, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getFsmInteger(
        vector <string> &integerNames,
        RangedMeasurement_t &range,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getFsmInteger(integerNames, range, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getMovingAverageFilterDuration(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getMovingAverageFilterDuration(range, defaultValue);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t get4thOrderFilterCutoffFrequency(
        RangedMeasurement_t &range,
        Measurement_t &defaultValue) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->get4thOrderFilterCutoffFrequency(range, defaultValue);

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

ErrorCodes_t getConditioningProtocolVoltage(
        vector <string> &voltageNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getConditioningProtocolVoltage(voltageNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getCheckingProtocolVoltage(
        vector <string> &voltageNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getCheckingProtocolVoltage(voltageNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

ErrorCodes_t getConditioningProtocolTime(
        vector <string> &timeNames,
        vector <RangedMeasurement_t> &ranges,
        vector <Measurement_t> &defaultValues) {
    ErrorCodes_t ret;
    if (messageDispatcher != nullptr) {
        ret = messageDispatcher->getConditioningProtocolTime(timeNames, ranges, defaultValues);

    } else {
        ret = ErrorDeviceNotConnected;
    }
    return ret;
}

} // namespace e4oCommLib

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
