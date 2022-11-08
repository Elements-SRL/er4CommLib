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
    {DeviceVersionE1, DeviceSubversionE1PlusEL03F, 1, DeviceE1PlusEL03fEDR3},               //    9,  8,  1 : e1+ EL03f chip (Legacy version for EDR3)
    {DeviceVersionE1, DeviceSubversionE1LightEL03F, 1, DeviceE1LightEL03fEDR3},             //    9,  7,  1 : e1Light EL03f chip (Legacy version for EDR3)
    {DeviceVersionE1, DeviceSubversionE1LightEL03F, 2, DeviceE1LightEL03fEDR3},             //    9,  7,  2 : e1Light EL03f chip (Legacy version for EDR3)
    {DeviceVersionE1, DeviceSubversionE1bEL03C, 4, DeviceE1bEL03cEDR3},                     //    9,  1,  4 : e1b EL03f chip (Legacy version for EDR3)
    {DeviceVersionE1, DeviceSubversionE1HcEL03F, 1, DeviceE1HcEL03fEDR3},                   //    9,  9,  1 : e1HC EL03f chip (Legacy version for EDR3)
    {DeviceVersionE16, DeviceSubversionE16e, 11, DeviceE16eEDR3},                           //    3,  8, 11 : e16e (Legacy version for EDR3)
    {DeviceVersionENPR, DeviceSubversionENPR, 129, DeviceENPR},                             //    8,  2,129 : eNPR
    {DeviceVersionENPR, DeviceSubversionENPRHC, 129, DeviceENPRHC},                         //    8,  8,129 : eNPR-HC
    {DeviceVersionE4, DeviceSubversionE4n, 10, DeviceE4nV04EDR3},                           //    4,  3, 10 : e4 Orbit mini with old ramp protocols (Legacy version for EDR3)
    {DeviceVersionE4, DeviceSubversionE4n, 11, DeviceE4nV04EDR3},                           //    4,  3, 11 : e4 Orbit mini with old ramp protocols (Legacy version for EDR3)
    {DeviceVersionE4, DeviceSubversionE4e, 15, DeviceE4eEDR3},                              //    4,  8, 15 : e4 Elements (Legacy version for EDR3)
    {DeviceVersionE4, DeviceSubversionE4e, 129, DeviceE4e},                                 //    4,  8,129 : e4 Elements version
    {DeviceVersionE16, DeviceSubversionE16Illumina, 129, DeviceE16Illumina},                //    3,  4,129 : e16 Orbit customized for Illumina
    {DeviceVersionE16, DeviceSubversionE16Illumina, 4, DeviceE16IlluminaEDR3},              //    3,  4,  4 : e16 Orbit customized for Illumina (Legacy version for EDR3)
    {DeviceVersionE16, DeviceSubversionE16n, 135, DeviceE16n},                              //    3,  5,135 : e16 2020 release
    {DeviceVersionE16, DeviceSubversionE16n, 136, DeviceE16n},                              //    3,  5,136 : e16 2020 release
    {DeviceVersionE16, DeviceSubversionE16eth, 4, DeviceE16ETHEDR3},                        //    3,  9,  4 : e16eth (Legacy Version for EDR3)
    {DeviceVersionE16, DeviceSubversionE16HC, 4, DeviceE16HC_V01},                          //    3, 10,  4 : e16HC No voltage amplifier
    {DeviceVersionE16, DeviceSubversionE16HC, 5, DeviceE16HC_V02},                          //    3, 10,  5 : e16HC
    {DeviceVersionDlp, DeviceSubversionDlp, 4, DeviceDlp},                                  //    6,  3,  4 : debug dlp
    {DeviceVersionDlp, DeviceSubversionEL06b, 129, TestboardEL06b},                         //    6,  5,129 : testboard EL06b
    {DeviceVersionDlp, DeviceSubversionEL06c, 129, TestboardEL06c},                         //    6,  6,129 : testboard EL06c
    {DeviceVersionDlp, DeviceSubversionEL06d, 129, TestboardEL06dEL06e},                    //    6,  7,129 : testboard EL06d
    {DeviceVersionDlp, DeviceSubversionEL06e, 129, TestboardEL06dEL06e},                    //    6,  8,129 : testboard EL06e
    {DeviceVersionPrototype, DeviceSubversionE2HCExtAdc, 1, DeviceE2HCExtAdc},              //  254, 14,  1 : e2HC with external ADC
    {DeviceVersionPrototype, DeviceSubversionE2HCExtAdc, 129, DeviceE2HCExtAdc},            //  254, 14,129 : e2HC with external ADC
    {DeviceVersionPrototype, DeviceSubversionE2HCIntAdc, 1, DeviceE2HCIntAdc},              //  254, 15,  1 : e2HC with internal (delta-sigma) ADC
    {DeviceVersionPrototype, DeviceSubversionE2HCIntAdc, 129, DeviceE2HCIntAdc},            //  254, 15,129 : e2HC with internal (delta-sigma) ADC
    {DeviceVersionPrototype, DeviceSubversionENPRFairyLight, 129, DeviceENPRFairyLight},    //  254, 16,129 : eNPR prototype for Fairy Light project
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

    cFastCompensationOptions.clear();
    cFastCompensationControl.implemented = false;
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
#ifdef _WIN32
    string path = string(getenv("HOMEDRIVE"))+string(getenv("HOMEPATH"));
#else
    string path = string(getenv("HOME"));
#endif
    stringstream ss;

    for (size_t i = 0; i < path.length(); ++i) {
        if (path[i] == '\\') {
            ss << "\\\\";

        } else {
            ss << path[i];
        }
    }
#ifdef _WIN32
    ss << "\\\\temp.txt";
#else
    ss << "/temp.txt";
#endif

    fid = fopen(ss.str().c_str(), "wb");
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

    if (iirX != nullptr) {
        for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
            delete [] iirX[channelIdx];
        }
        delete [] iirX;
        iirX = nullptr;
    }

    if (iirY != nullptr) {
        for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
            delete [] iirY[channelIdx];
        }
        delete [] iirY;
        iirY = nullptr;
    }

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
    connectionPaused = false;
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
    deviceCommunicationErrorFlag = false;

    /*! Calculate the LSB noise vector */
    this->initializeLsbNoise();

    this->initializeCompensations();

    this->initializeFerdMemory();

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

        if (!connectionPaused) {
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
        connectionPaused = false;

        return Success;

    } else {
        return ErrorDeviceNotConnected;
    }
}

ErrorCodes_t MessageDispatcher::pauseConnection(bool pauseFlag) {
    if (!connected) {
        return ErrorDeviceNotConnected;
    }

    ErrorCodes_t ret = Success;
    if (pauseFlag) {
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
        connectionPaused = true;

    } else {
        /*! Initialize the ftdi Rx handle */
        ret = this->initFtdiChannel(ftdiRxHandle, rxChannel);
        if (ret != Success) {
            return ret;
        }

        if (rxChannel == txChannel) {
            ftdiTxHandle = ftdiRxHandle;

        } else {
            /*! Initialize the ftdi Tx handle */
            ret = this->initFtdiChannel(ftdiTxHandle, txChannel);
            if (ret != Success) {
                return ret;
            }
        }
        deviceCommunicationErrorFlag = false;
        connectionPaused = false;
    }
    return ret;
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

ErrorCodes_t MessageDispatcher::convertCurrentValue(uint16_t uintValue, uint16_t channelIdx, double &fltValue) {
    fltValue = (((double)((int16_t)uintValue))+lsbNoiseArray[lsbNoiseIdx])*currentResolutions[channelIdx];
    lsbNoiseIdx = (lsbNoiseIdx+1)&LSB_NOISE_ARRAY_MASK;

    return Success;
}

ErrorCodes_t MessageDispatcher::setVoltageRange(uint16_t voltageRangeIdx, bool applyFlag) {
    if (voltageRangeIdx < voltageRangesNum) {
        selectedVoltageRangeIdx = voltageRangeIdx;
        voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
        voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step*(double)voltageRangeDivider;

        this->setFerdParameters();

        voltageRangeCoder->encode(selectedVoltageRangeIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setVoltageReferenceRange(uint16_t voltageRangeIdx, bool applyFlag) {
    if (voltageRangeIdx < voltageReferenceRangesNum) {
        selectedVoltageReferenceRangeIdx = voltageRangeIdx;
        voltageReferenceRange = voltageReferenceRangesArray[selectedVoltageReferenceRangeIdx];

        voltageReferenceRangeCoder->encode(selectedVoltageReferenceRangeIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        this->manageVoltageReference();

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool applyFlag) {
    if (currentRangeIdx < currentRangesNum) {
        selectedCurrentRangeIdx = currentRangeIdx; /*! \todo FCON funziona solo con un range */
        if (channelIdx < currentChannelsNum) {
            selectedCurrentRangesIdx[channelIdx] = currentRangeIdx;
            currentRanges[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]];
            currentResolutions[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]].step;

            this->setFerdParameters();

            currentRangeCoders[channelIdx]->encode(selectedCurrentRangesIdx[channelIdx], txStatus);
            if (applyFlag) {
                this->stackOutgoingMessage(txStatus);
            }

            return Success;

        } else if (channelIdx == currentChannelsNum) {
            for (channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
                selectedCurrentRangesIdx[channelIdx] = currentRangeIdx;
                currentRanges[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]];
                currentResolutions[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]].step;
            }

            this->setFerdParameters();

            currentRangeCoders[0]->encode(selectedCurrentRangesIdx[0], txStatus);
            if (applyFlag) {
                this->stackOutgoingMessage(txStatus);
            }

            return Success;

        } else {
            return ErrorValueOutOfRange;
        }

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setSamplingRate(uint16_t samplingRateIdx, bool applyFlag) {
    if (samplingRateIdx < samplingRatesNum) {
        selectedSamplingRateIdx = samplingRateIdx;
        baseSamplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
        samplingRate = baseSamplingRate*(double)oversamplingRatio;
        integrationStep = integrationStepArray[selectedSamplingRateIdx]/(double)oversamplingRatio;

        this->setRawDataFilter(rawDataFilterCutoffFrequency, rawDataFilterLowPassFlag, rawDataFilterActiveFlag);
        this->computeMinimumPacketNumber();
        this->setFerdParameters();

        samplingRateCoder->encode(selectedSamplingRateIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setOversamplingRatio(uint16_t oversamplingRatioIdx, bool applyFlag) {
    if (oversamplingRatioIdx < oversamplingRatiosNum) {
        selectedOversamplingRatioIdx = oversamplingRatioIdx;
        oversamplingRatio = oversamplingRatiosArray[selectedOversamplingRatioIdx];

        samplingRate = baseSamplingRate*(double)oversamplingRatio;
        integrationStep = integrationStepArray[selectedSamplingRateIdx]/(double)oversamplingRatio;

        this->setRawDataFilter(rawDataFilterCutoffFrequency, rawDataFilterLowPassFlag, rawDataFilterActiveFlag);
        this->computeMinimumPacketNumber();
        this->setFerdParameters();

        oversamplingRatioCoder->encode(selectedOversamplingRatioIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setVoltageStimulusLpf(uint16_t filterIdx, bool applyFlag) {
    if (filterIdx < voltageStimulusLpfOptionsNum) {
        dacIntFilterCoder->encode(filterIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setVoltageReferenceLpf(uint16_t filterIdx, bool applyFlag) {
    if (filterIdx < voltageReferenceLpfOptionsNum) {
        dacExtFilterCoder->encode(filterIdx, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::selectStimulusChannel(uint16_t channelIdx, bool on, bool applyFlag) {
    if ((channelIdx < currentChannelsNum) && selectStimulusChannelFlag) {
        selectStimulusChannelStates[channelIdx] = on;
        uint32_t selectStimulusChannelState = 0;
        for (unsigned int idx = 0; idx < currentChannelsNum; idx++) {
            selectStimulusChannelState |= (selectStimulusChannelStates[idx] ? (uint32_t)1 : 0) << idx;
        }
        selectStimulusChannelCoder->encode(selectStimulusChannelState, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else if ((channelIdx == currentChannelsNum) && singleChannelSSCFlag) {
        for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            this->selectStimulusChannel(channelIdx, on, false);
        }
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        if (((channelIdx < currentChannelsNum) && !selectStimulusChannelFlag) ||
                ((channelIdx == currentChannelsNum) && !singleChannelSSCFlag)) {
            return ErrorFeatureNotImplemented;

        } else {
            return ErrorValueOutOfRange;
        }
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

ErrorCodes_t MessageDispatcher::digitalOffsetCompensationAutostop(bool on, bool applyFlag) {
    if (selectableDOCAutostopFlag) {
        digitalOffsetCompensationAutostopCoder->encode(on, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorFeatureNotImplemented;
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

ErrorCodes_t MessageDispatcher::turnOnDigitalOutput(bool on) {
    if (!digOutImplementedFlag) {
        return ErrorFeatureNotImplemented;
    }

    digOutCoder->encode(on ? 1 : 0, txStatus);
    this->stackOutgoingMessage(txStatus);
    return Success;
}

ErrorCodes_t MessageDispatcher::turnLedOn(uint16_t ledIndex, bool on) {
    if (ledIndex >= ledsNum) {
        return ErrorValueOutOfRange;
    }

    ledsCoders[ledIndex]->encode(on ? 1 : 0, txStatus);
    this->stackOutgoingMessage(txStatus);
    return Success;
}

ErrorCodes_t MessageDispatcher::enableFrontEndResetDenoiser(bool on) {
    if (!ferdImplementedFlag) {
        return ErrorFeatureNotImplemented;
    }

    ferdFlag = on;
    return Success;
}

ErrorCodes_t MessageDispatcher::resetDevice() {
    deviceResetCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    deviceResetCoder->encode(0, txStatus);
    this->stackOutgoingMessage(txStatus);

    this->resetCalib();

    return Success;
}

ErrorCodes_t MessageDispatcher::resetCalib() {
    if (resetCalibrationFlag) {
        calibResetCoder->encode(1, txStatus);
        this->stackOutgoingMessage(txStatus);
        calibResetCoder->encode(0, txStatus);
        this->stackOutgoingMessage(txStatus);
    }

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

ErrorCodes_t MessageDispatcher::setVoltageOffset(unsigned int idx, Measurement_t voltage, bool applyFlag) {
    if (idx == currentChannelsNum) {
        for (idx = 0; idx < currentChannelsNum; idx++) {
            voltage.convertValue(selectedVoltageOffset[idx].prefix);
            voltageOffsetCoders[idx]->encode(voltage.value, txStatus);
        }

        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }
        return Success;

    } else if (idx < currentChannelsNum) {
        voltage.convertValue(selectedVoltageOffset[idx].prefix);
        voltageOffsetCoders[idx]->encode(voltage.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::checkVoltageOffset(unsigned int idx, Measurement_t voltage, string &message) {
    voltage.convertValue(selectedVoltageOffset[idx].prefix);
    selectedVoltageOffset[idx] = voltage;
    minSelectedVoltageOffset = voltage;
    maxSelectedVoltageOffset = voltage;

    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        if (selectedVoltageOffset[channelIdx].value < minSelectedVoltageOffset.value) {
            minSelectedVoltageOffset.value = selectedVoltageOffset[channelIdx].value;

        } else if (selectedVoltageOffset[channelIdx].value > maxSelectedVoltageOffset.value) {
            maxSelectedVoltageOffset.value = selectedVoltageOffset[channelIdx].value;
        }
    }

    if (this->checkProtocolValidity(message)) {
        return Success;

    } else {
        return ErrorInvalidProtocolParameters;
    }
}

ErrorCodes_t MessageDispatcher::applyInsertionPulse(Measurement_t voltage, Measurement_t duration) {
    if (insertionPulseImplemented) {
        this->selectVoltageProtocol(0, false);
        this->applyVoltageProtocol();

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

ErrorCodes_t MessageDispatcher::applyReferencePulse(Measurement_t voltage, Measurement_t duration) {
    if (referencePulseImplemented) {
        this->selectVoltageProtocol(0, false);
        this->applyVoltageProtocol();

        voltage.convertValue(referencePulseVoltageRange.prefix);
        duration.convertValue(referencePulseDurationRange.prefix);

        referencePulseVoltageCoder->encode(voltage.value, txStatus);
        referencePulseDurationCoder->encode(duration.value, txStatus);
        referencePulseApplyCoder->encode(1, txStatus);
        this->stackOutgoingMessage(txStatus);
        referencePulseApplyCoder->encode(0, txStatus);

        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::overrideReferencePulse(bool flag, bool applyFlag) {
    if (overrideReferencePulseImplemented) {
        if (flag) {
            overrideReferencePulseApplyCoder->encode(1, txStatus);

        } else {
            overrideReferencePulseApplyCoder->encode(0, txStatus);
        }

        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

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


ErrorCodes_t MessageDispatcher::applyDacExt(Measurement_t voltage, bool applyFlag) {
    if (!dacExtControllableFlag) {
        return ErrorFeatureNotImplemented;
    }

    voltage.convertValue(voltageReferenceRange.prefix);
    voltageReference = voltage;
    if (invertedDacExtFlag) {
        dacExtCoders[selectedVoltageReferenceRangeIdx]->encode(-voltage.value, txStatus);

    } else {
        dacExtCoders[selectedVoltageReferenceRangeIdx]->encode(voltage.value, txStatus);
        voltageReference.value = -voltageReference.value;
    }

    if (applyFlag) {
        this->stackOutgoingMessage(txStatus);
    }

    this->manageVoltageReference();

    return Success;
}

ErrorCodes_t MessageDispatcher::setFastReferencePulseProtocolWave1Voltage(unsigned int idx, Measurement_t voltage, bool applyFlag) {
    if (idx < 8) {
        voltage.convertValue(fastPulseW1VoltageRange.prefix);
        fastPulseW1VoltageCoder[idx]->encode(voltage.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setFastReferencePulseProtocolWave1Time(unsigned int idx, Measurement_t time, bool applyFlag) {
    if (idx < protocolTimesNum) {
        time.convertValue(protocolTimeRanges[idx].prefix);
        fastPulseW1TimeCoder[idx]->encode(time.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setFastReferencePulseProtocolWave2Voltage(unsigned int idx, Measurement_t voltage, bool applyFlag) {
    if (idx < 20) {
        voltage.convertValue(fastPulseW2VoltageRange.prefix);
        fastPulseW2VoltageCoder[idx]->encode(voltage.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setFastReferencePulseProtocolWave2Time(unsigned int idx, Measurement_t time, bool applyFlag) {
    if (idx < 20) {
        time.convertValue(fastPulseW2TimeRange.prefix);
        fastPulseW2TimeCoder[idx]->encode(time.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher::setFastReferencePulseProtocolWave2Duration(unsigned int idx, Measurement_t time, bool applyFlag) {
    if (idx < 20) {
        time.convertValue(fastPulseW2DurationRange.prefix);
        fastPulseW2DurationCoder[idx]->encode(time.value, txStatus);
        if (applyFlag) {
            this->stackOutgoingMessage(txStatus);
        }

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
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


ErrorCodes_t MessageDispatcher::setCompensationsChannel(uint16_t channelIdx) {
    compensationsSettingChannel = channelIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::turnCFastCompensationOn(bool on) {
    ErrorCodes_t ret;
    if (cFastCompensationControl.implemented) {
        if (compensationsSettingChannel < currentChannelsNum) {
            compensationsEnabledArray[CompensationCFast][compensationsSettingChannel] = on;
            cFastOnCoders[compensationsSettingChannel]->encode(on ? 1 : 0, txStatus);

            this->stackOutgoingMessage(txStatus);

            ret = Success;

        } else if (compensationsSettingChannel == currentChannelsNum) {
            vector <bool> flags = compensationsEnabledArray[CompensationCFast];
            for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
                compensationsEnabledArray[CompensationCFast][channelIdx] = on;
                cFastOnCoders[channelIdx]->encode(on ? 1 : 0, txStatus);
            }

            this->stackOutgoingMessage(txStatus);

            ret = Success;

        } else {
            ret = ErrorValueOutOfRange;
        }

    } else {
        ret = ErrorFeatureNotImplemented;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::setCFastCompensationOptions(uint16_t optionIdx) {
    ErrorCodes_t ret;
    if (cFastCompensationControl.implemented) {
        if (optionIdx < cFastCompensationOptions.size()) {
            //        uint16_t originalIdx = cFastCompensationOptionIdx[compensationsSettingChannel];
            //        cFastCompensationOptionIdx[compensationsSettingChannel] = optionIdx;
            //        ErrorCodes_t ret = this->setCompensationsOptions();
            //        if (ret != Success) {
            //            cFastCompensationOptionIdx[compensationsSettingChannel] = originalIdx;
            //        }
            //        return ret;

            //    } else if (optionIdx == 0) {
            ret = ErrorCommandNotImplemented;

        } else {
            ret = ErrorValueOutOfRange;
        }

    } else {
        ret = ErrorCommandNotImplemented;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::setCFastCapacitance(Measurement_t capacitance) {
    ErrorCodes_t ret;
    if (cFastCompensationControl.implemented) {
        capacitance.convertValue(cFastCompensationControl.prefix);

        if (compensationsSettingChannel < currentChannelsNum) {
            cFastCapacitance[compensationsSettingChannel] = capacitance.value;
            cFastControlCoders[compensationsSettingChannel]->encode(capacitance.value, txStatus);

            this->stackOutgoingMessage(txStatus);

            ret = Success;

        } else if (compensationsSettingChannel == currentChannelsNum) {
            for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
                cFastCapacitance[channelIdx] = capacitance.value;
                cFastControlCoders[channelIdx]->encode(capacitance.value, txStatus);
            }

            this->stackOutgoingMessage(txStatus);

            ret = Success;

        } else {
            ret = ErrorValueOutOfRange;
        }

    } else {
        ret = ErrorFeatureNotImplemented;
    }

    return ret;
}

ErrorCodes_t MessageDispatcher::setDebugBit(uint16_t byteOffset, uint16_t bitOffset, bool status) {
    BoolCoder::CoderConfig_t boolConfig;
    boolConfig.initialByte = byteOffset;
    boolConfig.initialBit = bitOffset;
    boolConfig.bitsNum = 1;
    bitDebugCoder = new BoolArrayCoder(boolConfig);
    bitDebugCoder->encode(status ? 1 : 0, txStatus);
    this->stackOutgoingMessage(txStatus);
    delete bitDebugCoder;
    return Success;
}

ErrorCodes_t MessageDispatcher::setDebugByte(uint16_t byteOffset, uint16_t byteValue) {
    BoolCoder::CoderConfig_t boolConfig;
    boolConfig.initialByte = byteOffset;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 7;
    byteDebugCoder = new BoolArrayCoder(boolConfig);
    byteDebugCoder->encode(byteValue, txStatus);
    this->stackOutgoingMessage(txStatus);
    delete byteDebugCoder;
    return Success;
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

ErrorCodes_t MessageDispatcher::getQueueStatus(QueueStatus_t &status) {
    unique_lock <mutex> readDataMtxLock (readDataMtx);
    if (outputBufferAvailablePackets > maxOutputPacketsNum) {
        status.availableDataPackets = maxOutputPacketsNum;

    } else {
        status.availableDataPackets = outputBufferAvailablePackets;
    }
    status.bufferOverflowFlag = outputBufferOverflowFlag;
    status.lostDataFlag = bufferDataLossFlag;
    status.saturationFlag = bufferSaturationFlag;
    if (selectedCurrentRangeIdx < currentRangesNum+1) {
        status.currentRangeIncreaseFlag = bufferIncreaseCurrentRangeFlag;

    } else {
        status.currentRangeIncreaseFlag = false;
    }

    if (selectedCurrentRangeIdx > 0) {
        status.currentRangeDecreaseFlag = bufferDecreaseCurrentRangeFlag;

    } else {
        status.currentRangeDecreaseFlag = false;
    }
    status.communicationErrorFlag = deviceCommunicationErrorFlag;

    outputBufferOverflowFlag = false;
    bufferDataLossFlag = false;
    bufferSaturationFlag = false;
    bufferIncreaseCurrentRangeFlag = false;
    bufferDecreaseCurrentRangeFlag = false;

    if (deviceCommunicationErrorFlag) {
        return ErrorDeviceCommunicationFailed;

    } else if (status.availableDataPackets == 0) {
        return WarningNoDataAvailable;

    } else {
        return Success;
    }
}

ErrorCodes_t MessageDispatcher::getDataPackets(uint16_t * &data, unsigned int packetsNumber, unsigned int * packetsRead) {
    unique_lock <mutex> readDataMtxLock (readDataMtx);
    ErrorCodes_t ret = Success;

    if (packetsNumber > outputBufferAvailablePackets) {
        ret = WarningNotEnoughDataAvailable;
        packetsNumber = outputBufferAvailablePackets;
    }

    if (packetsNumber > maxOutputPacketsNum) {
        ret = WarningNotEnoughDataAvailable;
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
    bufferSaturationFlag = false;
    return Success;
}

ErrorCodes_t MessageDispatcher::getChannelsNumber(uint32_t &voltageChannelsNumber, uint32_t &currentChannelsNumber) {
    voltageChannelsNumber = voltageChannelsNum;
    currentChannelsNumber = currentChannelsNum;
    return Success;
}

ErrorCodes_t MessageDispatcher::getCurrentRanges(vector <RangedMeasurement_t> &currentRanges, vector <uint16_t> &defaultOptions) {
    currentRanges = currentRangesArray;
    defaultOptions = defaultCurrentRangesIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::getCurrentRange(RangedMeasurement_t &currentRange, uint16_t channelIdx) {
    currentRange = currentRangesArray[selectedCurrentRangesIdx[channelIdx]];
    return Success;
}

ErrorCodes_t MessageDispatcher::hasIndependentCurrentRanges() {
    ErrorCodes_t ret = Success;
    if (!independentCurrentRangesFlag) {
        ret = ErrorFeatureNotImplemented;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::getVoltageRanges(vector <RangedMeasurement_t> &voltageRanges, uint16_t &defaultOption) {
    voltageRanges = voltageRangesArray;
    for (uint16_t idx = 0; idx < voltageRanges.size(); idx++) {
        voltageRanges[idx].step *= (double)voltageRangeDivider;
    }
    defaultOption = defaultVoltageRangeIdx;
    return Success;
}

ErrorCodes_t MessageDispatcher::getVoltageRange(RangedMeasurement_t &voltageRange) {
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    voltageRange.step *= (double)voltageRangeDivider;
    return Success;
}

ErrorCodes_t MessageDispatcher::getVoltageReferenceRanges(vector <RangedMeasurement_t> &ranges, uint16_t &defaultOption) {
    if (!dacExtControllableFlag) {
        return ErrorFeatureNotImplemented;
    }

    ranges = voltageReferenceRangesArray;
    defaultOption = defaultVoltageReferenceRangeIdx;
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

ErrorCodes_t MessageDispatcher::getOversamplingRatios(vector <uint16_t> &oversamplingRatios) {
    if (oversamplingImplemented) {
        oversamplingRatios = oversamplingRatiosArray;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getOversamplingRatio(uint16_t &oversamplingRatio) {
    if (oversamplingImplemented) {
        oversamplingRatio = oversamplingRatiosArray[selectedOversamplingRatioIdx];
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getVoltageStimulusLpfs(vector <Measurement_t> &filterOptions, uint16_t &defaultOption) {
    if (dacIntFilterAvailable) {
        filterOptions = voltageStimulusLpfOptions;
        defaultOption = voltageStimulusLpfDefaultOption;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getVoltageReferenceLpfs(vector <Measurement_t> &filterOptions, uint16_t &defaultOption) {
    if (dacExtFilterAvailable) {
        filterOptions = voltageReferenceLpfOptions;
        defaultOption = voltageReferenceLpfDefaultOption;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::hasSelectStimulusChannel(bool &selectStimulusChannelFlag, bool &singleChannelSSCFlag) {
    selectStimulusChannelFlag = this->selectStimulusChannelFlag;
    singleChannelSSCFlag = this->singleChannelSSCFlag;
    return Success;
}

ErrorCodes_t MessageDispatcher::hasDigitalOffsetCompensation(bool &digitalOffsetCompensationFlag, bool &singleChannelDOCFlag, bool &selectableDOCAutostopFlag) {
    digitalOffsetCompensationFlag = this->digitalOffsetCompensationFlag;
    singleChannelDOCFlag = this->singleChannelDOCFlag;
    selectableDOCAutostopFlag = this->selectableDOCAutostopFlag;
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

ErrorCodes_t MessageDispatcher::hasDigitalOutput() {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (digOutImplementedFlag) {
        ret = Success;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::hasFrontEndResetDenoiser() {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (ferdImplementedFlag) {
        ret = Success;
    }
    return ret;
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

ErrorCodes_t MessageDispatcher::getVoltageOffsetControls(RangedMeasurement_t &voltageRange) {
    if (voltageOffsetControlImplemented) {
        voltageRange = voltageOffsetRange;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
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

ErrorCodes_t MessageDispatcher::hasReferencePulseControls(bool &referencePulseImplemented, bool &overrideReferencePulseImplemented){
    referencePulseImplemented = this->referencePulseImplemented;
    overrideReferencePulseImplemented = this->overrideReferencePulseImplemented;
    return Success;

}

ErrorCodes_t MessageDispatcher::getReferencePulseControls(RangedMeasurement_t &voltageRange, RangedMeasurement_t &durationRange) {
    if (referencePulseImplemented) {
        voltageRange = referencePulseVoltageRange;
        durationRange = referencePulseDurationRange;
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

ErrorCodes_t MessageDispatcher::getLedsNumber(uint16_t &ledsNumber) {
    ErrorCodes_t ret = (ledsNum > 0 ? Success : ErrorFeatureNotImplemented);
    ledsNumber = ledsNum;
    return ret;
}

ErrorCodes_t MessageDispatcher::getLedsColors(vector <uint32_t> &ledsColors) {
    ledsColors = ledsColorsArray;
    return Success;
}

ErrorCodes_t MessageDispatcher::getFastReferencePulseProtocolWave1Range(RangedMeasurement_t &voltageRange, RangedMeasurement_t &timeRange, uint16_t &nPulse) {
    if (referencePulseImplemented) {
        voltageRange = fastPulseW1VoltageRange;
        timeRange = fastPulseW1TimeRange;
        nPulse = fastPulseW1num;
        return Success;

    } else {
        return ErrorCommandNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getFastReferencePulseProtocolWave2Range(RangedMeasurement_t &voltageRange, RangedMeasurement_t &timeRange, RangedMeasurement_t &durationRange, uint16_t &nPulse) {
    if (referencePulseImplemented) {
        voltageRange = fastPulseW2VoltageRange;
        timeRange = fastPulseW2TimeRange;
        durationRange = fastPulseW2DurationRange;
        nPulse = fastPulseW2num;
        return Success;

    } else {
        return ErrorCommandNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::hasNanionTemperatureController() {
    if (nanionTemperatureControllerFlag) {
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getTemperatureControllerRange(int &, int &) {
    return ErrorFeatureNotImplemented;
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

ErrorCodes_t MessageDispatcher::hasCFastCompensation() {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (cFastCompensationControl.implemented) {
        ret = Success;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::getCFastCompensationOptions(vector <string> &options) {
    if (cFastCompensationOptions.size() > 0) {
        options = cFastCompensationOptions;
        return Success;

    } else {
        return ErrorFeatureNotImplemented;
    }
}

ErrorCodes_t MessageDispatcher::getCFastCapacitanceControl(CompensationControl_t &control) {
    ErrorCodes_t ret = ErrorFeatureNotImplemented;
    if (cFastCompensationControl.implemented) {
        control = cFastCompensationControl;
        ret = Success;
    }
    return ret;
}

ErrorCodes_t MessageDispatcher::updateVoltageOffsetCompensations(vector <Measurement_t> &) {
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

void MessageDispatcher::initializeCompensations() {
    for (uint16_t compensationIdx = 0; compensationIdx < CompensationsNum; compensationIdx++) {
        /*! Assuming patch devices always have the same number of voltage and current channels */
        compensationsEnabledArray[compensationIdx].resize(currentChannelsNum);
        for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            compensationsEnabledArray[compensationIdx][channelIdx] = false;
        }
    }

    cFastCapacitance.resize(currentChannelsNum);
    cFastCompensationFlag.resize(currentChannelsNum);
    //    cFastCompensationOptionIdx.resize(currentChannelsNum);

    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        cFastCapacitance[channelIdx] = 0.0;
        cFastCompensationFlag[channelIdx] = false;

        //        cFastCompensationOptionIdx[channelIdx] = 0;
    }
}

void MessageDispatcher::processCurrentData(uint16_t channelIdx, uint16_t &x) {
    double xFlt = (double)((int16_t)x);
    xFlt = this->frontEndResetDenoise(channelIdx, xFlt);
    xFlt = this->applyFilter(channelIdx, xFlt);
    xFlt = round(xFlt > SHORT_MAX ? SHORT_MAX : (xFlt < SHORT_MIN ? SHORT_MIN : xFlt));
    x = (uint16_t)((int16_t)xFlt);
}

void MessageDispatcher::initializeFerdMemory() {
    ferdY.resize(totalChannelsNum);
    ferdY0.resize(totalChannelsNum);
    ferdM.resize(totalChannelsNum);

    for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        ferdY[channelIdx].resize(maxFerdSize);
    }
}

void MessageDispatcher::setFerdParameters() {
    /*! This are just the steps common to all devices, this method should be overriden for the front en reset denoiser to work properly */
    ferdIdx = 0;
    ferdD = 1.0/(double)ferdL;

    for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        ferdM[channelIdx] = 0.0;
        for (unsigned int idx = 0; idx < ferdL; idx++) {
            ferdY[channelIdx][idx] = 0.0;
        }
    }
}

double MessageDispatcher::frontEndResetDenoise(uint16_t channelIdx, double x) {
    if (ferdFlag & !ferdInhibition) {
        /*! Store previous noise estimate value */
        ferdY0[channelIdx] = ferdY[channelIdx][ferdIdx];

        /*! Update noise estimate */
        ferdY[channelIdx][ferdIdx] += (x-ferdY[channelIdx][ferdIdx])*ferdK;

        /*! Update noise estimate baseline */
        ferdM[channelIdx] += (ferdY[channelIdx][ferdIdx]-ferdY0[channelIdx])*ferdD;

        /*! Remove noise estimate and add baseline back */
        return x-ferdY[channelIdx][ferdIdx]+ferdM[channelIdx];

    } else {
        return x;
    }
}

void MessageDispatcher::readDataFromDevice() {
    stopConnectionFlag = false;

    /*! Declare variables to store FTDI functions results */
    FT_STATUS result;
    unsigned long availableFrames; /*!< Number of packets available in RX queue */
    DWORD readResult;
    DWORD ftdiQueuedBytes;
    unsigned int bytesToEnd;

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
        if (connectionPaused) {
            if (this->pauseConnection(false) != Success) {
                usleep(100000);
                continue;
            }
        }

        /******************\
         *  Reading part  *
        \******************/

        /*! Read queue status to check the number of available bytes */
        result = FT_GetQueueStatus(* ftdiRxHandle, &ftdiQueuedBytes);
        if (result != FT_OK) {
            deviceCommunicationErrorFlag = true;
            this->pauseConnection(true);
            usleep(100000);
            continue;
        }

        /*! Computes the number of available packets */
        availableFrames = ftdiQueuedBytes/(unsigned long)readFrameLength;

        /*! If there are not enough frames wait for a minimum frame number,
         *  the ftdi driver will wait for that to decrease overhead */
        if (availableFrames < minReadFrameNumber) {
            usleep(fewFramesSleep);
            continue;
        }

        /*! Cap bytes to read so that we do not try to read more than is available on the internal buffer */
        if (ftdiQueuedBytes+bytesReadFromDriver >= FTD_RX_BUFFER_SIZE) {
            ftdiQueuedBytes = FTD_RX_BUFFER_SIZE-bytesReadFromDriver;
        }

        /*! Reads the data */
        bytesToEnd = FTD_RX_BUFFER_SIZE-bufferWriteOffset;
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

        /*! Before storing the data to the output buffer wait for a least minStoreFrameNumber frames */
        bytesReadFromDriver += ftdiQueuedBytes;
        if (bytesReadFromDriver/(unsigned long)readFrameLength < minStoreFrameNumber) {
            continue;
        }

        candidateSync = 0;
        lastSyncIdx = 0;
        okFrames = 0;
        availableBytes = (int)bytesReadFromDriver;
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

        /*! Left over bytes that made up a partial frame */
        bytesReadFromDriver = (unsigned int)(availableBytes-availableBufferIdx);
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
    bool increaseRangeFlag;
    bool decreaseRangeFlag = true;
    bufferIncreaseCurrentRangeFlag = true;

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

                if (channelIdx < voltageChannelsNum) {
                    /*! Value is an uint, so before dividing cast to int, so that negative numbers are divided correctly */
                    value = (uint16_t)(((int16_t)value)/voltageRangeDivider+voltageReferenceOffset);
                    increaseRangeFlag = false;

                } else {
                    if ((value > UINT16_CURRENT_RANGE_INCREASE_MINIMUM_THRESHOLD) && (value < UINT16_CURRENT_RANGE_INCREASE_MAXIMUM_THRESHOLD)) {
                        /*! Suggest to increase range if any of the channels is above the threshold */
                        increaseRangeFlag = true;
                        if ((value == UINT16_POSITIVE_SATURATION) || (value == UINT16_NEGATIVE_SATURATION)) {
                            bufferSaturationFlag = true;
                        }

                    } else if ((value < UINT16_CURRENT_RANGE_DECREASE_MINIMUM_THRESHOLD) && (value > UINT16_CURRENT_RANGE_DECREASE_MAXIMUM_THRESHOLD)) {
                        /*! Suggest to decrease range only if all the channels are below the threshold in the whole frame (otherwise noise could bring all values within threshold and trigger spurious changes) */
                        decreaseRangeFlag = false;
                    }

                    this->processCurrentData(channelIdx, value);
                }

                outputDataBuffer[outputBufferWriteOffset][channelIdx] = value;
            }

            /*! Check that in each packet at least one channel is above threshold for the whole frame */
            bufferIncreaseCurrentRangeFlag &= increaseRangeFlag;

            if (++ferdIdx >= ferdL) {
                /*! At the moment the front end reset denoiser is only available for devices that apply the same current range on all channels */
                ferdIdx = 0;
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

    if (decreaseRangeFlag) {
        bufferDecreaseCurrentRangeFlag = true;
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
    minStoreFrameNumber = (unsigned long)ceil(FTD_FEW_PACKET_COEFF*samplingRateInHz.value/((double)packetsPerFrame));
    minReadFrameNumber = (unsigned long)min(minStoreFrameNumber, (unsigned long)ceil(((double)FTD_MAX_BYTES_TO_WAIT_FOR)/(double)readFrameLength));
    fewFramesSleep = (unsigned int)ceil(((double)(minReadFrameNumber*(unsigned long)packetsPerFrame))/samplingRateInHz.value*1.0e6);
}

void MessageDispatcher::initializeRawDataFilterVariables() {
    iirX = new double * [totalChannelsNum];
    iirY = new double * [totalChannelsNum];
    for (unsigned int channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        iirX[channelIdx] = new double[IIR_ORD+1];
        iirY[channelIdx] = new double[IIR_ORD+1];
        for (int tapIdx = 0; tapIdx < IIR_ORD+1; tapIdx++) {
            iirX[channelIdx][tapIdx] = 0.0;
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
        double d = 1.0/(1.0+k1*IIR_2_SIN_PI_4+k12); /*!< denominator */

        /*! Denominators */
        iirDen[0] = 1.0;
        iirDen[1] = k2*d;
        iirDen[2] = (-1.0-k2+k12*k12+k12*IIR_2_COS_PI_4_2)*d*d;

        /*! Gains and numerators */
        double iirG;
        if (rawDataFilterLowPassFlag) {
            iirG = (1.0+iirDen[1]+iirDen[2])*0.25;

            iirNum[1] = 2.0*iirG;

        } else {
            iirG = (1.0-iirDen[1]+iirDen[2])*0.25;

            iirNum[1] = -2.0*iirG;
        }

        iirNum[0] = iirG;
        iirNum[2] = iirG;

    } else {
        /*! Delta impulse response with no autoregressive part */
        iirNum[0] = 1.0;
        iirNum[1] = 0.0;
        iirNum[2] = 0.0;
        iirDen[0] = 1.0;
        iirDen[1] = 0.0;
        iirDen[2] = 0.0;
    }

    /*! reset FIFOs */
    for (uint16_t channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
        for (int tapIdx = 0; tapIdx < IIR_ORD+1; tapIdx++) {
            iirX[channelIdx][tapIdx] = 0.0;
            iirY[channelIdx][tapIdx] = 0.0;
        }
    }
}

double MessageDispatcher::applyFilter(uint16_t channelIdx, double x) {
    /*! 2nd order Butterworth filter */
    int tapIdx;

    int coeffIdx = 0;
    iirX[channelIdx][iirOff] = x;
    double y = x*iirNum[coeffIdx++];

    for (tapIdx = iirOff+1; tapIdx <= IIR_ORD; tapIdx++) {
        y += iirX[channelIdx][tapIdx]*iirNum[coeffIdx]-iirY[channelIdx][tapIdx]*iirDen[coeffIdx];
        coeffIdx++;
    }

    for (tapIdx = 0; tapIdx < iirOff; tapIdx++) {
        y += iirX[channelIdx][tapIdx]*iirNum[coeffIdx]-iirY[channelIdx][tapIdx]*iirDen[coeffIdx];
        coeffIdx++;
    }

    iirY[channelIdx][iirOff] = y;
    return y;
}

void MessageDispatcher::manageVoltageReference() {
    voltageReference.convertValue(voltageRange.prefix);
    voltageReferenceOffset = (int16_t)(voltageReference.value/voltageResolution);
}

MessageDispatcherLegacyEdr3::MessageDispatcherLegacyEdr3(string deviceId) :
    MessageDispatcher(deviceId) {

}

MessageDispatcherLegacyEdr3::~MessageDispatcherLegacyEdr3() {

}

void MessageDispatcherLegacyEdr3::storeDataFrames(unsigned int framesNum) {
    uint16_t value;
    bool increaseRangeFlag;
    bool decreaseRangeFlag = true;
    bufferIncreaseCurrentRangeFlag = true;

    for (unsigned int frameIdx = 0; frameIdx < framesNum; frameIdx++) {
        /*! Skips the sync word at the beginning of each packet */
        bufferReadOffset = (bufferReadOffset+(unsigned long)FTD_RX_SYNC_WORD_SIZE)&FTD_RX_BUFFER_MASK;

        /*! Get current and voltage data */
        for (int packetIdx = 0; packetIdx < packetsPerFrame; packetIdx++) {
            for (unsigned short channelIdx = 0; channelIdx < totalChannelsNum; channelIdx++) {
                value = 0;

                for (unsigned int byteIdx = 0; byteIdx < FTD_RX_WORD_SIZE; byteIdx++) {
                    value <<= 8;
                    value += *(readDataBuffer+bufferReadOffset);
                    bufferReadOffset = (bufferReadOffset+1)&FTD_RX_BUFFER_MASK;
                }

                if (channelIdx < voltageChannelsNum) {
                    value -= rawVoltageZero; /*! Offset binary conversion to 2's complement */
                    value = (uint16_t)(((int16_t)value)/voltageRangeDivider+voltageReferenceOffset);

                    increaseRangeFlag = false;

                } else {
                    value -= 0x8000; /*! Offset binary conversion to 2's complement */

                    if ((value > UINT16_CURRENT_RANGE_INCREASE_MINIMUM_THRESHOLD) && (value < UINT16_CURRENT_RANGE_INCREASE_MAXIMUM_THRESHOLD)) {
                        /*! Suggest to increase range if any of the channels is above the threshold */
                        increaseRangeFlag = true;
                        if ((value == UINT16_POSITIVE_SATURATION) || (value == UINT16_NEGATIVE_SATURATION)) {
                            bufferSaturationFlag = true;
                        }

                    } else if ((value < UINT16_CURRENT_RANGE_DECREASE_MINIMUM_THRESHOLD) && (value > UINT16_CURRENT_RANGE_DECREASE_MAXIMUM_THRESHOLD)) {
                        /*! Suggest to decrease range only if all the channels are below the threshold in the whole frame (otherwise noise could bring all values within threshold and trigger spurious changes) */
                        decreaseRangeFlag = false;
                    }

                    this->processCurrentData(channelIdx, value);
                }

                outputDataBuffer[outputBufferWriteOffset][channelIdx] = value;
            }

            /*! Check that in each packet at least one channel is above threshold for the whole frame */
            bufferIncreaseCurrentRangeFlag &= increaseRangeFlag;

            if (++ferdIdx >= ferdL) {
                /*! At the moment the front end reset denoiser is only available for devices that apply the same current range on all channels */
                ferdIdx = 0;
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

    if (decreaseRangeFlag) {
        bufferDecreaseCurrentRangeFlag = true;
    }

    /*! If too many packets are written but not read from the user the buffer saturates */
    if (outputBufferAvailablePackets > ER4CL_OUTPUT_BUFFER_SIZE/totalChannelsNum) {
        outputBufferAvailablePackets = ER4CL_OUTPUT_BUFFER_SIZE/totalChannelsNum; /*!< Saturates available packets */
        outputBufferReadOffset = outputBufferWriteOffset; /*! Move read offset just on top of the write offset so that it can read up to 1 position before after a full buffer read */
        outputBufferOverflowFlag = true;
    }
}
