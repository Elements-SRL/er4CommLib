#include "messagedispatcher.h"

#include <iostream>
#include <sstream>
#include <ctime>
#include <thread>
#include <math.h>
#include <random>
#include <algorithm>
#include <unistd.h>

static const vector <vector <uint32_t>> deviceTupleMapping = {
    {DeviceVersionE4, DeviceSubversionE4e, 129, DeviceE4e},             //    4,  8,129 : e4 Elements version
    {DeviceVersionE16, DeviceSubversionE16n, 4, DeviceE16n},            //    3,  5,  4 : e16 2020 release
    {DeviceVersionDlp, DeviceSubversionDlp, 4, DeviceDlp},              //    6,  3,  4 : debug dlp
    {DeviceVersionDemo, DeviceSubversionDemo, 1, DeviceFakeE16n}
};

/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcher                                        *
 *                                                                                          *
\********************************************************************************************/

/*****************\
 *  Ctor / Dtor  *
\*****************/

MessageDispatcher::MessageDispatcher(string deviceId) :
    deviceId(deviceId) {

    /*! Raw data filter */
    rawDataFilterCutoffFrequencyRange.step = 0.1;
    rawDataFilterCutoffFrequencyRange.min = 0.0;
    rawDataFilterCutoffFrequencyRange.max = 100.0;
    rawDataFilterCutoffFrequencyRange.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyRange.unit = "Hz";

    rawDataFilterCutoffFrequencyDefault.value = 30.0;
    rawDataFilterCutoffFrequencyDefault.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyDefault.unit = "Hz";
    rawDataFilterCutoffFrequency = rawDataFilterCutoffFrequencyDefault;
}

MessageDispatcher::~MessageDispatcher() {
    this->deinit();
}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t MessageDispatcher::init() {
    readDataBuffer = new (std::nothrow) unsigned char[FTD_RX_BUFFER_SIZE];
    if (readDataBuffer == nullptr) {
        return ErrorInitializationFailed;
    }

    outputDataBuffer = new (std::nothrow) uint16_t * [ER4CL_OUTPUT_BUFFER_SIZE];
    if (outputDataBuffer == nullptr) {
        return ErrorInitializationFailed;
    }

    outputDataBuffer[0] = new (std::nothrow) uint16_t[ER4CL_OUTPUT_BUFFER_SIZE*totalChannelsNum];
    if (outputDataBuffer == nullptr) {
        return ErrorInitializationFailed;
    }
    for (int packetIdx = 1; packetIdx < ER4CL_OUTPUT_BUFFER_SIZE; packetIdx++) {
        outputDataBuffer[packetIdx] = outputDataBuffer[0]+((int)totalChannelsNum)*packetIdx;
    }

    lsbNoiseArray = new (std::nothrow) double[LSB_NOISE_ARRAY_SIZE];
    if (lsbNoiseArray == nullptr) {
        return ErrorInitializationFailed;
    }

    txMsgBuffer = new (std::nothrow) vector <uint8_t>[FTD_TX_MSG_BUFFER_SIZE];
    if (txMsgBuffer == nullptr) {
        return ErrorInitializationFailed;
    }

    txRawBuffer = new (std::nothrow) uint8_t[txDataBytes];
    if (txRawBuffer == nullptr) {
        return ErrorInitializationFailed;
    }

#ifdef DEBUG_PRINT
    fid = fopen("temp.txt", "w+");
#endif

    this->computeMinimumPacketNumber();

    this->initializeRawDataFilterVariables();

    return Success;
}

ErrorCodes_t MessageDispatcher::deinit() {
    if (readDataBuffer != nullptr) {
        delete [] readDataBuffer;
        readDataBuffer = nullptr;
    }

    if (outputDataBuffer != nullptr) {
        if (outputDataBuffer[0] != nullptr) {
            delete [] outputDataBuffer[0];
            outputDataBuffer[0] = nullptr;
        }
        delete [] outputDataBuffer;
        outputDataBuffer = nullptr;
    }

    if (lsbNoiseArray != nullptr) {
        delete [] lsbNoiseArray;
        lsbNoiseArray = nullptr;
    }

    if (txMsgBuffer != nullptr) {
        delete [] txMsgBuffer;
        txMsgBuffer = nullptr;
    }

    if (txRawBuffer != nullptr) {
        delete [] txRawBuffer;
        txRawBuffer = nullptr;
    }

    for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        delete [] iirX[channelIdx];
        delete [] iirU[channelIdx];
        delete [] iirY[channelIdx];
    }

    delete [] iirX;
    delete [] iirU;
    delete [] iirY;

#ifdef DEBUG_PRINT
    fclose(fid);
#endif

    return Success;
}

ErrorCodes_t MessageDispatcher::connect(FtdiEeprom * ftdiEeprom) {
    if (connected) {
        return ErrorDeviceAlreadyConnected;
    }

    connected = true;
    ErrorCodes_t ret;

    this->ftdiEeprom = ftdiEeprom;

    /*! Initialize the ftdi Rx handle */
    ftdiRxHandle = new FT_HANDLE;

    ret = this->initFtdiChannel(ftdiRxHandle, rxChannel);
    if (ret != Success) {
        return ret;
    }

    if (rxChannel == txChannel) {
        ftdiTxHandle = ftdiRxHandle;

    } else {
        /*! Initialize the ftdi Tx handle */
        ftdiTxHandle = new FT_HANDLE;

        ret = this->initFtdiChannel(ftdiTxHandle, txChannel);
        if (ret != Success) {
            return ret;
        }
    }

    /*! Calculate the LSB noise vector */
    this->initializeLsbNoise();

    stopConnectionFlag = false;

    this->setRawDataFilter({30.0, UnitPfxKilo, "Hz"}, true, false);

    rxThread = thread(&MessageDispatcher::readDataFromDevice, this);

    txThread = thread(&MessageDispatcher::sendCommandsToDevice, this);

    threadsStarted = true;

    this->resetDevice();
    this_thread::sleep_for(chrono::milliseconds(10));

    /*! Initialize device */
    this->initializeDevice();
    this->stackOutgoingMessage(txStatus);

    this_thread::sleep_for(chrono::milliseconds(10));

    return ret;
}

ErrorCodes_t MessageDispatcher::disconnect() {
    if (!connected) {
        return ErrorDeviceNotConnected;
    }

    if (!stopConnectionFlag) {
        stopConnectionFlag = true;

        if (threadsStarted) {
            rxThread.join();
            txThread.join();
        }

        FT_STATUS ftRet;
        ftRet = FT_Close(* ftdiRxHandle);
        if (ftRet != FT_OK) {
            return ErrorDeviceDisconnectionFailed;
        }

        if (rxChannel != txChannel) {
            FT_STATUS ftRet;
            ftRet = FT_Close(* ftdiTxHandle);
            if (ftRet != FT_OK) {
                return ErrorDeviceDisconnectionFailed;
            }
        }

        if (ftdiEeprom != nullptr) {
            delete ftdiEeprom;
            ftdiEeprom = nullptr;
        }

        if (ftdiRxHandle != nullptr) {
            delete ftdiRxHandle;
            ftdiRxHandle = nullptr;
            if (txChannel == rxChannel) {
                ftdiTxHandle = nullptr;
            }
        }

        if (ftdiTxHandle != nullptr) {
            delete ftdiTxHandle;
            ftdiTxHandle = nullptr;
        }

        connected = false;

        return Success;

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t MessageDispatcher::getDeviceType(DeviceTuple_t tuple, DeviceTypes_t &type) {
    bool deviceFound = false;
    for (unsigned int mappingIdx = 0; mappingIdx < deviceTupleMapping.size(); mappingIdx++) {
        if (tuple.version == deviceTupleMapping[mappingIdx][0] &&
                tuple.subversion == deviceTupleMapping[mappingIdx][1] &&
                tuple.fwVersion == deviceTupleMapping[mappingIdx][2]) {
            type = (DeviceTypes_t)deviceTupleMapping[mappingIdx][3];
            deviceFound = true;
            break;
        }
    }

    if (deviceFound) {
        return Success;

    } else {
        return ErrorDeviceTypeNotRecognized;
    }
}

/****************\
 *  Tx methods  *
\****************/

ErrorCodes_t MessageDispatcher::turnOnLsbNoise(bool flag) {
    this->initializeLsbNoise(!flag);
    return Success;
}

ErrorCodes_t MessageDispatcher::convertVoltageValue(uint16_t uintValue, double &fltValue) {
    fltValue = (((double)((int16_t)uintValue))+lsbNoiseArray[lsbNoiseIdx])*voltageResolution;
    lsbNoiseIdx = (lsbNoiseIdx+1)&LSB_NOISE_ARRAY_MASK;

    return Success;
}

ErrorCodes_t MessageDispatcher::convertCurrentValue(uint16_t uintValue, double &fltValue) {
    fltValue = (((double)((int16_t)uintValue))+lsbNoiseArray[lsbNoiseIdx])*currentResolution;
    lsbNoiseIdx = (lsbNoiseIdx+1)&LSB_NOISE_ARRAY_MASK;

    return Success;
}

ErrorCodes_t MessageDispatcher::setVoltageRange(uint16_t voltageRangeIdx, bool applyFlag) {
    if (voltageRangeIdx < voltageRangesNum) {
        selectedVoltageRangeIdx = voltageRangeIdx;
        voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
        voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step;

        voltageRangeCoder->encode(selectedVoltageRangeIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setCurrentRange(uint16_t currentRangeIdx, bool applyFlag) {
    if (currentRangeIdx < currentRangesNum) {
        selectedCurrentRangeIdx = currentRangeIdx;
        currentRange = currentRangesArray[selectedCurrentRangeIdx];
        currentResolution = currentRangesArray[selectedCurrentRangeIdx].step;

        currentRangeCoder->encode(selectedCurrentRangeIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setSamplingRate(uint16_t samplingRateIdx, bool applyFlag) {
    if (samplingRateIdx < samplingRatesNum) {
        selectedSamplingRateIdx = samplingRateIdx;
        samplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
        integrationStep = integrationStepArray[selectedSamplingRateIdx];
        this->setRawDataFilter(rawDataFilterCutoffFrequency, rawDataFilterLowPassFlag, rawDataFilterActiveFlag);

        samplingRateCoder->encode(selectedSamplingRateIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::digitalOffsetCompensation(uint16_t channelIdx, bool on, bool applyFlag) {
    if ((channelIdx < currentChannelsNum) && digitalOffsetCompensationFlag) {
        digitalOffsetCompensationStates[channelIdx] = on;
        uint32_t digitalOffsetCompensationState = 0;
        for (unsigned int idx = 0; idx < currentChannelsNum; idx++) {
            digitalOffsetCompensationState |= (digitalOffsetCompensationStates[idx] ? (uint32_t)1 : 0) << idx;
        }
        digitalOffsetCompensationCoder->encode(digitalOffsetCompensationState, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else if ((channelIdx == currentChannelsNum) && singleChannelDOCFlag) {
        for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            this->digitalOffsetCompensation(channelIdx, on, false);
        }
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        if (((channelIdx < currentChannelsNum) && !digitalOffsetCompensationFlag) ||
                ((channelIdx == currentChannelsNum) && !singleChannelDOCFlag)) {
            return ErrorFeatureNotImplemented;

        } else {
            return ErrorValueOutOfRange;
        }
    }
}

ErrorCodes_t MessageDispatcher::zap(uint16_t channelIdx, bool applyFlag) {
    if ((channelIdx < currentChannelsNum) && singleChannelZapFlag) {
        zapStates[channelIdx] = !zapStates[channelIdx];
        uint32_t zapState = 0;
        for (unsigned int idx = 0; idx < currentChannelsNum; idx++) {
            zapState |= (zapStates[idx] ? (uint32_t)1 : 0) << idx;
        }
        zapCoder->encode(zapState, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else if ((channelIdx == currentChannelsNum) && zappableDeviceFlag) {
        for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            this->zap(channelIdx, false);
        }
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        if (((channelIdx < currentChannelsNum) && !singleChannelZapFlag) ||
                ((channelIdx == currentChannelsNum) && !zappableDeviceFlag)) {
            return ErrorFeatureNotImplemented;

        } else {
            return ErrorValueOutOfRange;
        }
    }
}

ErrorCodes_t MessageDispatcher::switchChannelOn(uint16_t channelIdx, bool on, bool applyFlag) {
    if ((channelIdx < currentChannelsNum) && singleChannelOnFlag) {
        channelOnStates[channelIdx] = on;
        uint32_t channelOnState = 0;
        for (unsigned int idx = 0; idx < currentChannelsNum; idx++) {
            channelOnState |= (channelOnStates[idx] ? (uint32_t)1 : 0) << idx;
        }
        channelOnCoder->encode(channelOnState, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else if ((channelIdx == currentChannelsNum) && channelOnFlag) {
        for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            this->switchChannelOn(channelIdx, on, false);
        }
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        if (((channelIdx < currentChannelsNum) && !singleChannelOnFlag) ||
                ((channelIdx == currentChannelsNum) && !channelOnFlag)) {
            return ErrorFeatureNotImplemented;

        } else {
            return ErrorValueOutOfRange;
        }
    }
}

ErrorCodes_t MessageDispatcher::switchVcSel0(bool on) {
    VcSel0Coder->encode(on ? 1 : 0, txStatus);
    this->stackOutgoingMessage(txStatus);

    return Success;
}

ErrorCodes_t MessageDispatcher::switchVcSel1(bool on) {
    VcSel1Coder->encode(on ? 1 : 0, txStatus);
    this->stackOutgoingMessage(txStatus);

    return Success;
}

ErrorCodes_t MessageDispatcher::resetDevice() {
    deviceResetCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    deviceResetCoder->encode(0, txStatus);

    return Success;
}

ErrorCodes_t MessageDispatcher::resetDigitalOffsetCompensation(bool) {
    ErrorCodes_t ret = Success;
//    uint16_t dataLength = switchesStatusLength;
//    vector <uint16_t> txDataMessage(dataLength);
//    this->switches2DataMessage(txDataMessage);

//    unsigned int resetIdx = ResetIndexDigitalOffsetCompensation;

//    if (reset) {
//        txDataMessage[resetWord[resetIdx]] |= resetByte[resetIdx];

//    } else {
//        txDataMessage[resetWord[resetIdx]] &= ~resetByte[resetIdx];
//    }

//    ret = this->manageOutgoingMessageLife(MsgDirectionEdrToDevice+MsgTypeIdSwitchCtrl, txDataMessage, dataLength);
//    if (ret == Success) {
//        this->dataMessage2Switches(txDataMessage);
//    }
    return ret;
}

ErrorCodes_t MessageDispatcher::sendCommands() {
    this->stackOutgoingMessage(txStatus);

    return Success;
}

ErrorCodes_t MessageDispatcher::selectVoltageProtocol(unsigned int idx, bool applyFlag) {
    protocolsSelectCoder->encode(idx, txStatus);
    if (applyFlag) {
        this->stackOutgoingMessage(txStatus);
    }

    return Success;
}

ErrorCodes_t MessageDispatcher::applyVoltageProtocol() {
    protocolStartCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    protocolStartCoder->encode(0, txStatus);

    return Success;
}

ErrorCodes_t MessageDispatcher::setProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag) {
    if (idx < protocolVoltagesNum) {
        voltage.convertValue(protocolVoltageRanges[idx].prefix);
        protocolVoltageCoders[idx]->encode(voltage.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setProtocolTime(unsigned int idx, Measurement_t time, bool applyFlag) {
    if (idx < protocolTimesNum) {
        time.convertValue(protocolTimeRanges[idx].prefix);
        protocolTimeCoders[idx]->encode(time.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setProtocolSlope(unsigned int idx, Measurement_t slope, bool applyFlag) {
    if (idx < protocolSlopesNum) {
        slope.convertValue(protocolSlopeRanges[idx].prefix);
        protocolSlopeCoders[idx]->encode(slope.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setProtocolAdimensional(unsigned int idx, Measurement_t adimensional, bool applyFlag) {
    if (idx < protocolAdimensionalsNum) {
        adimensional.convertValue(protocolAdimensionalRanges[idx].prefix);
        protocolAdimensionalCoders[idx]->encode(adimensional.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::checkSelectedProtocol(unsigned int idx, string &message) {
    selectedProtocol = idx;
    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::checkProtocolVoltage(unsigned int idx, Measurement_t voltage, string &message) {
    selectedProtocolVoltage[idx] = voltage;
    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::checkProtocolTime(unsigned int idx, Measurement_t time, string &message) {
    selectedProtocolTime[idx] = time;
    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::checkProtocolSlope(unsigned int idx, Measurement_t slope, string &message) {
    selectedProtocolSlope[idx] = slope;
    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::checkProtocolAdimensional(unsigned int idx, Measurement_t adimensional, string &message) {
    selectedProtocolAdimensional[idx] = adimensional;
    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::applyInsertionPulse(Measurement_t voltage, Measurement_t duration) {
    if (insertionPulseImplemented) {
        voltage.convertValue(insertionPulseVoltageRange.prefix);
        duration.convertValue(insertionPulseDurationRange.prefix);

        insertionPulseVoltageCoder->encode(voltage.value, txStatus);
        insertionPulseDurationCoder->encode(duration.value, txStatus);
        insertionPulseApplyCoder->encode(1, txStatus);
        this->stackOutgoingMessage(txStatus);
        insertionPulseApplyCoder->encode(0, txStatus);

        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::setRawDataFilter(Measurement_t cutoffFrequency, bool lowPassFlag, bool activeFlag) {
    ErrorCodes_t ret;

    if ((cutoffFrequency.value > 0.0) || (cutoffFrequency < samplingRate*0.5)) {
        rawDataFilterCutoffFrequency = cutoffFrequency;
        rawDataFilterLowPassFlag = lowPassFlag;
        rawDataFilterActiveFlag = activeFlag;
        this->computeFilterCoefficients();

        ret = Success;

    } else {
        ret = ErrorValueOutOfRange;
    }

    return ret;
}

ErrorCodes_t MessageDispatcher::activateFEResetDenoiser(bool flag, bool applyFlag) {
//    fEResetDenoiserCoder->encode(flag ? 1 : 0, txStatus);
//    if (applyFlag) {
//        this->stackOutgoingMessage(txStatus);
//    }

    return Success;
}

ErrorCodes_t MessageDispatcher::activateDacIntFilter(bool flag, bool applyFlag) {
    dacIntFilterCoder->encode(flag ? 1 : 0, txStatus);
    if (applyFlag) {
        this->stackOutgoingMessage(txStatus);
    }

    return Success;
}

ErrorCodes_t MessageDispatcher::resetWasherError() {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::setWasherPresetSpeeds(vector <int8_t>) {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::startWasher(uint16_t) {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::updateWasherState() {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::updateWasherPresetSpeeds() {
    return ErrorFeatureNotImplemented;
}

/****************\
 *  Rx methods  *
\****************/

ErrorCodes_t MessageDispatcher::isDeviceUpgradable(string &upgradeNotes, string &notificationTag) {
    ErrorCodes_t ret = Success;

    upgradeNotes = this->upgradeNotes;
    notificationTag = this->notificationTag;
    if (upgradeNotes == "NONE") {
        ret = ErrorUpgradesNotAvailable;
    }

    return ret;
}

ErrorCodes_t MessageDispatcher::getDeviceInfo(uint8_t &deviceVersion, uint8_t &deviceSubversion, uint32_t &firmwareVersion) {
    ErrorCodes_t ret = Success;

    DeviceTuple_t tuple = ftdiEeprom->getDeviceTuple();
    deviceVersion = tuple.version;
    deviceSubversion = tuple.subversion;
    firmwareVersion = tuple.fwVersion;

    return ret;
}

ErrorCodes_t MessageDispatcher::getQueueStatus(unsigned int * availableDataPackets, bool * bufferOverflowFlag, bool * dataLossFlag) {
    unique_lock <mutex> readDataMtxLock (readDataMtx);
    if (outputBufferAvailablePackets > maxOutputPacketsNum) {
        * availableDataPackets = maxOutputPacketsNum;

    } else {
        * availableDataPackets = outputBufferAvailablePackets;
    }
    * bufferOverflowFlag = outputBufferOverflowFlag;
    * dataLossFlag = bufferDataLossFlag;

    outputBufferOverflowFlag = false;
    bufferDataLossFlag = false;
    return Success;
}

ErrorCodes_t MessageDispatcher::getDataPackets(uint16_t * &data, unsigned int packetsNumber, unsigned int * packetsRead) {
    unique_lock <mutex> readDataMtxLock (readDataMtx);
    ErrorCodes_t ret = Success;

    if (packetsNumber > outputBufferAvailablePackets) {
        ret = WarningNotEnoughAvailable;
        packetsNumber = outputBufferAvailablePackets;
    }

    if (packetsNumber > maxOutputPacketsNum) {
        ret = WarningNotEnoughAvailable;
        packetsNumber = maxOutputPacketsNum;
    }

    unsigned int channelIdx;
    for (unsigned int packetIdx = 0; packetIdx < packetsNumber; packetIdx++) {
        for (channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
            data[packetIdx*totalChannelsNum+channelIdx] = outputDataBuffer[(outputBufferReadOffset+packetIdx)&ER4CL_OUTPUT_BUFFER_MASK][channelIdx];
        }
    }
    outputBufferReadOffset = (outputBufferReadOffset+packetsNumber)&ER4CL_OUTPUT_BUFFER_MASK;
    outputBufferAvailablePackets -= packetsNumber;
    * packetsRead = packetsNumber;

    return ret;
}

ErrorCodes_t MessageDispatcher::purgeData() {
    unique_lock <mutex> readDataMtxLock (readDataMtx);
    /*! Performs a fake read of all available data samples. */
    outputBufferReadOffset = outputBufferWriteOffset;
    outputBufferAvailablePackets = 0;
    outputBufferOverflowFlag = false;
    bufferDataLossFlag = false;
    return Success;
}

ErrorCodes_t MessageDispatcher::getChannelsNumber(uint32_t &voltageChannelsNumber, uint32_t &currentChannelsNumber) {
    voltageChannelsNumber = voltageChannelsNum;
    currentChannelsNumber = currentChannelsNum;
    return Success;
}

ErrorCodes_t MessageDispatcher::getCurrentRanges(vector <RangedMeasurement_t> &currentRanges, uint16_t &defaultOption) {
    currentRanges = currentRangesArray;
    defaultOption = defaultCurrentRangeIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::getCurrentRange(RangedMeasurement_t &currentRange) {
    currentRange = currentRangesArray[selectedCurrentRangeIdx];
    return Success;
}

ErrorCodes_t MessageDispatcher::getVoltageRanges(vector <RangedMeasurement_t> &voltageRanges, uint16_t &defaultOption) {
    voltageRanges = voltageRangesArray;
    defaultOption = defaultVoltageRangeIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::getVoltageRange(RangedMeasurement_t &voltageRange) {
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    return Success;
}

ErrorCodes_t MessageDispatcher::getSamplingRates(vector <Measurement_t> &samplingRates, uint16_t &defaultOption) {
    samplingRates = samplingRatesArray;
    defaultOption = defaultSamplingRateIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::getSamplingRate(Measurement_t &samplingRate) {
    samplingRate = samplingRatesArray[selectedSamplingRateIdx];
    return Success;
}

ErrorCodes_t MessageDispatcher::getRealSamplingRates(vector <Measurement_t> &samplingRates) {
    samplingRates = realSamplingRatesArray;
    return Success;
}

ErrorCodes_t MessageDispatcher::hasDigitalOffsetCompensation(bool &digitalOffsetCompensationFlag, bool &singleChannelDOCFlag) {
    digitalOffsetCompensationFlag = this->digitalOffsetCompensationFlag;
    singleChannelDOCFlag = this->singleChannelDOCFlag;
    return Success;
}

ErrorCodes_t MessageDispatcher::hasZap(bool &zappableDeviceFlag, bool &singleChannelZapFlag) {
    zappableDeviceFlag = this->zappableDeviceFlag;
    singleChannelZapFlag = this->singleChannelZapFlag;
    return Success;
}

ErrorCodes_t MessageDispatcher::hasChannelOn(bool &channelOnFlag, bool &singleChannelOnFlag) {
    channelOnFlag = this->channelOnFlag;
    singleChannelOnFlag = this->singleChannelOnFlag;
    return Success;
}

ErrorCodes_t MessageDispatcher::getLiquidJunctionControl(CompensationControl_t &control) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (liquidJunctionControl.implemented) {
        control = liquidJunctionControl;
        ret = Success;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::getProtocolList(vector <string> &names, vector <string> &images, vector <vector <uint16_t>> &voltages, vector <vector <uint16_t>> &times, vector <vector <uint16_t>> &slopes, vector <vector <uint16_t>> &adimensionals) {
    names = protocolsNames;
    images = protocolsImages;
    voltages = protocolsAvailableVoltages;
    times = protocolsAvailableTimes;
    slopes = protocolsAvailableSlopes;
    adimensionals = protocolsAvailableAdimensionals;
    return Success;
}

ErrorCodes_t MessageDispatcher::getTriangularProtocolIdx(uint16_t &idx) {
    if (triangularProtocolIdx > 0) {
        /*! Protocol 0 is always the VHold */
        idx = triangularProtocolIdx;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getSealTestProtocolIdx(uint16_t &idx) {
    if (sealTestProtocolIdx > 0) {
        /*! Protocol 0 is always the VHold */
        idx = sealTestProtocolIdx;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getProtocolVoltage(vector <string> &voltageNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues) {
    voltageNames = protocolVoltageNames;
    ranges = protocolVoltageRanges;
    defaultValues = protocolVoltageDefault;
    return Success;
}

ErrorCodes_t MessageDispatcher::getProtocolTime(vector <string> &timeNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues) {
    timeNames = protocolTimeNames;
    ranges = protocolTimeRanges;
    defaultValues = protocolTimeDefault;
    return Success;
}

ErrorCodes_t MessageDispatcher::getProtocolSlope(vector <string> &slopeNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues) {
    slopeNames = protocolSlopeNames;
    ranges = protocolSlopeRanges;
    defaultValues = protocolSlopeDefault;
    return Success;
}

ErrorCodes_t MessageDispatcher::getProtocolAdimensional(vector <string> &adimensionalNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues) {
    adimensionalNames = protocolAdimensionalNames;
    ranges = protocolAdimensionalRanges;
    defaultValues = protocolAdimensionalDefault;
    return Success;
}

ErrorCodes_t MessageDispatcher::getInsertionPulseControls(RangedMeasurement_t &voltageRange, RangedMeasurement_t &durationRange) {
    if (insertionPulseImplemented) {
        voltageRange = insertionPulseVoltageRange;
        durationRange = insertionPulseDurationRange;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getEdhFormat(string &format) {
    format = edhFormat;
    return Success;
}

ErrorCodes_t MessageDispatcher::getRawDataFilterCutoffFrequency(RangedMeasurement_t &range, Measurement_t &defaultValue) {
    range = rawDataFilterCutoffFrequencyRange;
    defaultValue = rawDataFilterCutoffFrequencyDefault;
    return Success;
}

ErrorCodes_t MessageDispatcher::hasWasherControls() {
    if (washerControlFlag) {
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getWasherSpeedRange(RangedMeasurement_t &) {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::getWasherStatus(WasherStatus_t &, WasherError_t &) {
    return ErrorFeatureNotImplemented;
}

ErrorCodes_t MessageDispatcher::getWasherPresetSpeeds(vector <int8_t> &) {
    return ErrorFeatureNotImplemented;
}

/*********************\
 *  Private methods  *
\*********************/

ErrorCodes_t MessageDispatcher::initFtdiChannel(FT_HANDLE * handle, char channel) {
    FT_STATUS ftRet;

    string communicationSerialNumber = deviceId+channel;

    /*! Opens the device */
    ftRet = FT_OpenEx((PVOID)communicationSerialNumber.c_str(), FT_OPEN_BY_SERIAL_NUMBER, handle);
    if (ftRet != FT_OK) {
        return ErrorDeviceConnectionFailed;
    }

    /*! Sets latency */
    ftRet = FT_SetLatencyTimer(* handle, 2); /*!< ms */
    if (ftRet != FT_OK) {
        FT_Close(* handle);
        return ErrorFtdiConfigurationFailed;
    }

    /*! Sets transfers size to */
    ftRet = FT_SetUSBParameters(* handle, 4096, 4096);
    if (ftRet != FT_OK) {
        FT_Close(* handle);
        return ErrorFtdiConfigurationFailed;
    }

    /*! Purges buffers */
    ftRet = FT_Purge(* handle, FT_PURGE_RX | FT_PURGE_TX);
    if (ftRet != FT_OK) {
        FT_Close(* handle);
        return ErrorFtdiConfigurationFailed;
    }

    if (channel == rxChannel) {
        if (ftRet != FT_OK) {
            FT_Close(* handle);
            return ErrorFtdiConfigurationFailed;
        }
    }

    return Success;
}

void MessageDispatcher::initializeLsbNoise(bool nullValues) {
    if (nullValues) {
        /*! By default there is no added noise  */
        for (int32_t i = 0; i < LSB_NOISE_ARRAY_SIZE; i++) {
            lsbNoiseArray[i] = 0.0;
        }

    } else {
        mt19937 mtRng((uint32_t)time(nullptr));
        double den = (double)0xFFFFFFFF;
        for (int32_t i = 0; i < LSB_NOISE_ARRAY_SIZE; i++) {
            lsbNoiseArray[i] = ((double)mtRng())/den-0.5;
        }
    }
}

void MessageDispatcher::readDataFromDevice() {
    stopConnectionFlag = false;

    /*! Declare variables to store FTDI functions results */
    FT_STATUS result;
    unsigned long availableFrames; /*!< Number of packets available in RX queue */
    DWORD readResult;
    DWORD ftdiQueuedBytes;

    /*! Declare variables to manage buffers indexing */
    unsigned int syncOffset; /*!< Index used to check the sync word */
    unsigned int lastSyncOffset; /*! Last position in which a sync word was found: used to track back when synchronization is lost */
    uint32_t candidateSync;
    int availableBufferIdx; /*! Index used to search sync words: relative to last write offset */
    int lastSyncIdx; /*!< Last index for which a sync word was found */
    unsigned int okFrames; /*! Number of frames correctly identified within 2 syncwords that can be analyzed */
    int availableBytes; /*! Number of bytes available for analysis */

    unique_lock <mutex> readDataMtxLock (readDataMtx);
    readDataMtxLock.unlock();

#ifdef DEBUG_RAW_BIT_RATE_PRINT
    std::chrono::steady_clock::time_point startPrintfTime;
    std::chrono::steady_clock::time_point currentPrintfTime;
    startPrintfTime = std::chrono::steady_clock::now();
    long long int acc = 0;
#endif

    while (!stopConnectionFlag) {

        /******************\
         *  Reading part  *
        \******************/

        /*! Read queue status to check the number of available bytes */
        result = FT_GetQueueStatus(* ftdiRxHandle, &ftdiQueuedBytes);
        if (result != FT_OK) {
            continue; /*! \todo FCON Notify to user? */
        }

        /*! Computes the number of available packets */
        availableFrames = ftdiQueuedBytes/(unsigned long)readFrameLength;

        /*! If there are not enough frames wait for a minimum frame number,
         *  the ftdi driver will wait for that to decrease overhead */
        if (availableFrames < minFrameNumber) {
            usleep(fewFramesSleep);
            continue;
        }

        if (ftdiQueuedBytes+bytesLeftFromPreviousRead >= FTD_RX_BUFFER_SIZE) {
            ftdiQueuedBytes = FTD_RX_BUFFER_SIZE-bytesLeftFromPreviousRead;
        }

        /*! Reads the data */
        unsigned int bytesToEnd = FTD_RX_BUFFER_SIZE-bufferWriteOffset;
        if (ftdiQueuedBytes > bytesToEnd) {
            result = FT_Read(* ftdiRxHandle, readDataBuffer+bufferWriteOffset, bytesToEnd, &readResult);
            result |= FT_Read(* ftdiRxHandle, readDataBuffer, ftdiQueuedBytes-bytesToEnd, &readResult);

        } else {
            result = FT_Read(* ftdiRxHandle, readDataBuffer+bufferWriteOffset, ftdiQueuedBytes, &readResult);
        }

        if (result != FT_OK) {
            continue; /*! \todo FCON Notify to user? */
        }

        /******************\
         *  Parsing part  *
        \******************/

#ifdef DEBUG_RAW_BIT_RATE_PRINT
        currentPrintfTime = std::chrono::steady_clock::now();
        acc += ftdiQueuedBytes;
        if ((double)(std::chrono::duration_cast <std::chrono::microseconds> (currentPrintfTime-startPrintfTime).count()) > 1.0e6) {
            printf("%f byte/s\n", 1.0e6*((double)acc)/(double)(std::chrono::duration_cast <std::chrono::microseconds> (currentPrintfTime-startPrintfTime).count()));
            fflush(stdout);
            startPrintfTime = currentPrintfTime;
            acc = 0;
        }
#endif

        /*! Extracts a pointer to the buffer */
        bufferWriteOffset = (bufferWriteOffset+ftdiQueuedBytes)&FTD_RX_BUFFER_MASK;

        candidateSync = 0;
        lastSyncIdx = 0;
        okFrames = 0;
        availableBytes = (int)(ftdiQueuedBytes+bytesLeftFromPreviousRead);
        FtdBufferAnalaysisStatus_t status;

        if (exitOnSyncWord) {
            availableBufferIdx = 0;
            syncOffset = (bufferReadOffset+(unsigned int)FTD_RX_SYNC_WORD_SIZE)&FTD_RX_BUFFER_MASK;
            status = FtdStatusHeaderFound;

        } else {
            availableBufferIdx = -1;
            syncOffset = bufferReadOffset;
            status = FtdStatusLookingForHeader;
        }

        lastSyncOffset = syncOffset;

        /*! This loop alternates 2 loops:
         *  the first looks for a sync word one byte at a time when synchronization is lost (hopefully rarely executed)
         *  the second looks for sync words one packet at a time while synchronization is kept */
        while (status != FtdStatusBufferFinished) {
            if (status == FtdStatusLookingForHeader) {
                /*! Resynchronization loop */

                /*! Preloads FTD_RX_SYNC_WORD_SIZE-1 bytes in candidateSync */
                for (int syncByte = 0; syncByte < FTD_RX_SYNC_WORD_SIZE-1; syncByte++) {
                    candidateSync = ((candidateSync << 8) & 0xFFFFFF00)+(readDataBuffer[syncOffset] & 0xFF);
                    syncOffset = (syncOffset+1)&FTD_RX_BUFFER_MASK;
                }

                while ((candidateSync != rxSyncWord) && (availableBufferIdx+FTD_RX_SYNC_WORD_SIZE <= availableBytes)) {
                    /*! Shifts the candidate syncword and adds a new byte */
                    candidateSync = ((candidateSync << 8) & 0xFFFFFF00)+(readDataBuffer[syncOffset] & 0xFF);
                    syncOffset = (syncOffset+1)&FTD_RX_BUFFER_MASK;
                    availableBufferIdx++;
                }

                if (candidateSync == rxSyncWord) {
                    /*! If loop exited on sync word found move to second loop */
                    lastSyncIdx = availableBufferIdx;
                    lastSyncOffset = syncOffset;
                    bufferReadOffset = syncOffset-(unsigned int)FTD_RX_SYNC_WORD_SIZE;
                    status = FtdStatusHeaderFound;

                } else {
                    /*! If loop did not exit on sync word the buffer has finished... */

                    /*! Lets the next call to this function know this did not return on a sync word found,... */
                    exitOnSyncWord = false;

                    /*! Updates bufferReadOffset to exclude bytes analyzed without finding a sync word... */
                    bufferReadOffset = ((unsigned int)(((int)bufferReadOffset)+availableBufferIdx-lastSyncIdx))&FTD_RX_BUFFER_MASK;

                    /*! And returns */
                    status = FtdStatusBufferFinished;
                }
            }

            while (status == FtdStatusHeaderFound) {
                /*! Synchronized loop */

                if (availableBufferIdx+readFrameLength+FTD_RX_SYNC_WORD_SIZE <= availableBytes) {
                    /*! If there are enough bytes to check for another syncword... */

                    /*! Moves to the expected offset of the next syncword... */
                    syncOffset = (syncOffset+(unsigned int)(readFrameLength-FTD_RX_SYNC_WORD_SIZE))&FTD_RX_BUFFER_MASK;
                    availableBufferIdx += readFrameLength;

                    /*! Loads FTD_RX_SYNC_WORD_SIZE bytes in candidateSync... */
                    candidateSync = 0;
                    for (int syncByte = 0; syncByte < FTD_RX_SYNC_WORD_SIZE; syncByte++) {
                        candidateSync = ((candidateSync << 8) & 0xFFFFFF00)+(readDataBuffer[syncOffset] & 0xFF);
                        syncOffset = (syncOffset+1)&FTD_RX_BUFFER_MASK;
                    }

                    /*! And checks the sync word */
                    if (candidateSync == rxSyncWord) {
                        /*! If sync word is found */
                        lastSyncIdx = availableBufferIdx;
                        lastSyncOffset = syncOffset;
                        okFrames++;

                    } else {
                        /*! If sync word is not found... */

                        /*! Tracks back to last found sync word to restore synchronization... */
                        syncOffset = (lastSyncOffset+1)&FTD_RX_BUFFER_MASK;
                        availableBufferIdx = lastSyncIdx+FTD_RX_SYNC_WORD_SIZE;

                        readDataMtxLock.lock(); /*!< Protects data modified in storeDataPackets and bufferDataLossFlag */
                        if (okFrames > 0) { /*!< If at least one correct packet was found process it */
                            this->storeDataFrames(okFrames); /*!< bufferReadOffset is updated by storeDataPackets */
                            okFrames = 0;
                        }

                        bufferDataLossFlag = true;
                        readDataMtxLock.unlock();

                        /*! And switches to the first loop */
                        status = FtdStatusLookingForHeader;
                    }

                } else {
                    /*! If there are not enough bytes to check for another syncword... */

                    /*! Lets the next call to this function know this returned on a syn cword found,... */
                    exitOnSyncWord = true;

                    /*! Sends eventually found frames... */
                    if (okFrames > 0) {
                        readDataMtxLock.lock(); /*!< Protects data modified in storeDataPackets */
                        this->storeDataFrames(okFrames); /*!< bufferReadOffset is updated by storeDataPackets */
                        readDataMtxLock.unlock();
                    }

                    /*! And returns */
                    status = FtdStatusBufferFinished;
                }
            }
        }

        bytesLeftFromPreviousRead = (unsigned int)(availableBytes-availableBufferIdx);
    }
}

void MessageDispatcher::sendCommandsToDevice() {
    FT_STATUS ftRet;
    DWORD bytesToWrite;
    DWORD ftdiWrittenBytes;
    int writeTries = 0;

    /*! Variables used to access the tx raw buffer */
    uint32_t txRawBufferReadIdx = 0; /*!< Index being processed wrt buffer  */

    /*! Variables used to access the tx msg buffer */
    uint32_t txMsgBufferReadOffset = 0; /*!< Offset of the part of buffer to be processed  */

    /*! Variables used to access the tx data buffer */
    uint32_t txDataBufferReadIdx;

    bool notSentTxData;

    unique_lock <mutex> txMutexLock (txMutex);
    txMutexLock.unlock();

    while (!stopConnectionFlag) {

        /***********************\
         *  Data copying part  *
        \***********************/

        txMutexLock.lock();
        while (txMsgBufferReadLength <= 0) {
            txMsgBufferNotEmpty.wait_for(txMutexLock, chrono::milliseconds(100));
            if (stopConnectionFlag) {
                break;
            }
        }
        txMutexLock.unlock();
        if (stopConnectionFlag) {
            continue;
        }

        txRawBufferReadIdx = 0;
        for (txDataBufferReadIdx = 0; txDataBufferReadIdx < txDataBytes; txDataBufferReadIdx++) {
            * ((uint8_t *)(txRawBuffer+txRawBufferReadIdx)) = txMsgBuffer[txMsgBufferReadOffset][txDataBufferReadIdx];
            txRawBufferReadIdx += FTD_TX_WORD_SIZE;
        }

        txMsgBufferReadOffset = (txMsgBufferReadOffset+1)&FTD_TX_MSG_BUFFER_MASK;

        /******************\
         *  Sending part  *
        \******************/

        notSentTxData = true;
        bytesToWrite = (DWORD)txDataBytes;
        while (notSentTxData && (writeTries++ < FTD_MAX_WRITE_TRIES)) { /*! \todo FCON prevedere un modo per notificare ad alto livello e all'utente */
            ftRet = FT_Write(* ftdiTxHandle, txRawBuffer, bytesToWrite, &ftdiWrittenBytes);

            if (ftRet != FT_OK) {
                continue;
            }

#ifdef DEBUG_PRINT
            fprintf(fid, "\n%d %d %d\n", txDataBytes, bytesToWrite, ftdiWrittenBytes);
            fflush(fid);

            for (int i = 0; i < txDataBytes; i++) {
                fprintf(fid, "%03d:%02x ", i, txRawBuffer[i]);
                if (i % 16 == 15) {
                    fprintf(fid, "\n");
                }
            }
            fprintf(fid, "\n");
            fflush(fid);
#endif

            /*! If less bytes than need are sent purge the buffer and retry */
            if (ftdiWrittenBytes < bytesToWrite) {
                /*! Cleans TX buffer */
                ftRet = FT_Purge(* ftdiTxHandle, FT_PURGE_TX);

            } else {
                notSentTxData = false;
                writeTries = 0;
            }
        }

        txMutexLock.lock();
        txMsgBufferReadLength--;
        txMsgBufferNotFull.notify_all();
        txMutexLock.unlock();
    }
}

void MessageDispatcher::storeDataFrames(unsigned int framesNum) {
    uint16_t value;
    uint8_t infoIndex;
    uint8_t infoValue;

    for (unsigned int frameIdx = 0; frameIdx < framesNum; frameIdx++) {
        /*! Skips the sync word at the beginning of each packet */
        bufferReadOffset = (bufferReadOffset+(unsigned long)FTD_RX_SYNC_WORD_SIZE)&FTD_RX_BUFFER_MASK;

        /*! Extract info struct */
        infoIndex = *(readDataBuffer+bufferReadOffset);
        bufferReadOffset = (bufferReadOffset+1)&FTD_RX_BUFFER_MASK;

        infoValue = *(readDataBuffer+bufferReadOffset);
        bufferReadOffset = (bufferReadOffset+1)&FTD_RX_BUFFER_MASK;

        infoStructPtr[infoIndex] = infoValue;

        /*! Get current and voltage data */
        for (int packetIdx = 0; packetIdx < packetsPerFrame; packetIdx++) {
            for (unsigned short channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
                value = 0;

                for (unsigned int byteIdx = 0; byteIdx < FTD_RX_WORD_SIZE; byteIdx++) {
                    value <<= 8;
                    value += *(readDataBuffer+bufferReadOffset);
                    bufferReadOffset = (bufferReadOffset+1)&FTD_RX_BUFFER_MASK;
                }

                /*! Correct offset of voltage channels */
                if (channelIdx < voltageChannelsNum) {

                } else {
                    this->applyFilter(channelIdx-voltageChannelsNum, value);
                }

                outputDataBuffer[outputBufferWriteOffset][channelIdx] = value;
            }

            if (iirOff < 1) {
                iirOff = IIR_ORD;

            } else {
                iirOff--;
            }

            outputBufferWriteOffset = (outputBufferWriteOffset+1)&ER4CL_OUTPUT_BUFFER_MASK;
            outputBufferAvailablePackets++;
        }
    }

    /*! If too many packets are written but not read from the user the buffer saturates */
    if (outputBufferAvailablePackets > ER4CL_OUTPUT_BUFFER_SIZE/totalChannelsNum) {
        outputBufferAvailablePackets = ER4CL_OUTPUT_BUFFER_SIZE/totalChannelsNum; /*!< Saturates available packets */
        outputBufferReadOffset = outputBufferWriteOffset; /*! Move read offset just on top of the write offset so that it can read up to 1 position before after a full buffer read */
        outputBufferOverflowFlag = true;
    }
}

void MessageDispatcher::stackOutgoingMessage(vector <uint8_t> &txDataMessage) {
    unique_lock <mutex> txMutexLock (txMutex);
    while (txMsgBufferReadLength >= FTD_TX_MSG_BUFFER_SIZE) {
        txMsgBufferNotFull.wait(txMutexLock);
    }

    txMsgBuffer[txMsgBufferWriteOffset] = txDataMessage;
    txMsgBufferWriteOffset = (txMsgBufferWriteOffset+1)&FTD_TX_MSG_BUFFER_MASK;
    txMsgBufferReadLength++;
    txMsgBufferNotEmpty.notify_all();
}

void MessageDispatcher::int322uint16(int32_t from, vector <uint16_t> &to, size_t offset) {
    to[offset] = from & 0xFFFF;
    to[offset+1] = (from >> 16) & 0xFFFF;
}

void MessageDispatcher::uint322uint16(uint32_t from, vector <uint16_t> &to, size_t offset) {
    to[offset] = from & 0xFFFF;
    to[offset+1] = (from >> 16) & 0xFFFF;
}

void MessageDispatcher::computeMinimumPacketNumber() {
    Measurement_t samplingRateInHz = samplingRate;
    samplingRateInHz.convertValue(UnitPfxNone);
    minFrameNumber = (unsigned long)ceil(FTD_FEW_PACKET_COEFF*samplingRateInHz.value/((double)packetsPerFrame));
    minFrameNumber = (unsigned long)min(minFrameNumber, (unsigned long)ceil(((double)FTD_MAX_BYTES_TO_WAIT_FOR)/(double)readFrameLength));
    fewFramesSleep = (unsigned int)ceil(((double)((minFrameNumber*(unsigned long)packetsPerFrame)/2))/samplingRateInHz.value*1.0e6);
}

void MessageDispatcher::initializeRawDataFilterVariables() {
    iirX = new double * [totalChannelsNum];
    iirU = new double * [totalChannelsNum];
    iirY = new double * [totalChannelsNum];
    for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        iirX[channelIdx] = new double[IIR_ORD+1];
        iirU[channelIdx] = new double[IIR_ORD+1];
        iirY[channelIdx] = new double[IIR_ORD+1];
        for (int tapIdx = 0; tapIdx < IIR_ORD+1; tapIdx++) {
            iirX[channelIdx][tapIdx] = 0.0;
            iirU[channelIdx][tapIdx] = 0.0;
            iirY[channelIdx][tapIdx] = 0.0;
        }
    }
}

void MessageDispatcher::computeFilterCoefficients() {
    if (rawDataFilterActiveFlag && (rawDataFilterCutoffFrequency < samplingRate*0.5)) {
        rawDataFilterCutoffFrequency.convertValue(1.0/integrationStep.multiplier());

        double k1 = tan(M_PI*rawDataFilterCutoffFrequency.value*integrationStep.value);
        double k12 = k1*k1;
        double k2 = -2+2*k12; /*!< frequently used expression */
        double d1 = 1.0/(1.0+k1*IIR_2_SIN_3_PI_8+k12); /*! denominator of first biquad */
        double d2 = 1.0/(1.0+k1*IIR_2_SIN_PI_8+k12); /*! denominator of second biquad */

        /*! Denominators */
        iir1Den[0] = 1.0;
        iir2Den[0] = 1.0;
        iir1Den[1] = k2*d1;
        iir2Den[1] = k2*d2;
        iir1Den[2] = (-1.0-k2+k12*k12+k12*IIR_2_COS_3_PI_8_2)*d1*d1;
        iir2Den[2] = (-1.0-k2+k12*k12+k12*IIR_2_COS_PI_8_2)*d2*d2;

        /*! Gains and numerators */
        double iir1G;
        double iir2G;
        if (rawDataFilterLowPassFlag) {
            iir1G = (1.0+iir1Den[1]+iir1Den[2])*0.25;
            iir2G = (1.0+iir2Den[1]+iir2Den[2])*0.25;

            iir1Num[1] = 2.0*iir1G;
            iir2Num[1] = 2.0*iir2G;

        } else {
            iir1G = (1.0-iir1Den[1]+iir1Den[2])*0.25;
            iir2G = (1.0-iir2Den[1]+iir2Den[2])*0.25;

            iir1Num[1] = -2.0*iir1G;
            iir2Num[1] = -2.0*iir2G;
        }

        iir1Num[0] = iir1G;
        iir2Num[0] = iir2G;
        iir1Num[2] = iir1G;
        iir2Num[2] = iir2G;

    } else {
        /*! Delta impulse response with no autoregressive part */
        iir1Num[0] = 1.0;
        iir2Num[0] = 1.0;
        iir1Num[1] = 0.0;
        iir2Num[1] = 0.0;
        iir1Num[2] = 0.0;
        iir2Num[2] = 0.0;
        iir1Den[0] = 1.0;
        iir2Den[0] = 1.0;
        iir1Den[1] = 0.0;
        iir2Den[1] = 0.0;
        iir1Den[2] = 0.0;
        iir2Den[2] = 0.0;
    }

    /*! reset FIFOs */
    for (uint16_t channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        for (int tapIdx = 0; tapIdx < IIR_ORD+1; tapIdx++) {
            iirX[channelIdx][tapIdx] = 0.0;
            iirU[channelIdx][tapIdx] = 0.0;
            iirY[channelIdx][tapIdx] = 0.0;
        }
    }
}

void MessageDispatcher::applyFilter(uint16_t channelIdx, uint16_t &x) {
    /*! 4th order Butterworth filter with cascaded biquad structure */
    int tapIdx;
    double xFlt = (double)((int16_t)x);

    /*! 1st biquad section */
    int coeffIdx = 0;
    iirX[channelIdx][iirOff] = xFlt;
    double u = xFlt*iir1Num[coeffIdx++];

    for (tapIdx = iirOff+1; tapIdx <= IIR_ORD; tapIdx++) {
        u += iirX[channelIdx][tapIdx]*iir1Num[coeffIdx]-iirU[channelIdx][tapIdx]*iir1Den[coeffIdx];
        coeffIdx++;
    }

    for (tapIdx = 0; tapIdx < iirOff; tapIdx++) {
        u += iirX[channelIdx][tapIdx]*iir1Num[coeffIdx]-iirU[channelIdx][tapIdx]*iir1Den[coeffIdx];
        coeffIdx++;
    }

    /*! 2nd biquad section */
    coeffIdx = 0;
    iirU[channelIdx][iirOff] = u;
    double y = u*iir2Num[coeffIdx++];

    for (tapIdx = iirOff+1; tapIdx <= IIR_ORD; tapIdx++) {
        y += iirU[channelIdx][tapIdx]*iir2Num[coeffIdx]-iirY[channelIdx][tapIdx]*iir2Den[coeffIdx];
        coeffIdx++;
    }

    for (tapIdx = 0; tapIdx < iirOff; tapIdx++) {
        y += iirU[channelIdx][tapIdx]*iir2Num[coeffIdx]-iirY[channelIdx][tapIdx]*iir2Den[coeffIdx];
        coeffIdx++;
    }

    iirY[channelIdx][iirOff] = y;
    y = round(y > SHORT_MAX ? SHORT_MAX : (y < SHORT_MIN ? SHORT_MIN : y));
    x = (uint16_t)((int16_t)y);
}
