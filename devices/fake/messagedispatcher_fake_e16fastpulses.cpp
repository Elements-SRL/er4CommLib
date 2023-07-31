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

#include "messagedispatcher_fake_e16fastpulses.h"

#include <thread>
#include <ctime>
#include <cmath>
#include <sstream>

/*****************\
 *  Ctor / Dtor  *
\*****************/

MessageDispatcher_fake_e16FastPulses::MessageDispatcher_fake_e16FastPulses(string di) :
    MessageDispatcher_e16FastPulses_V02(di) {

    /*! Sampling rates */
    samplingRatesNum = 4;
    samplingRatesArray.resize(samplingRatesNum);

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate1_25kHz].value = 1.25;
    realSamplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate5kHz].value = 5.0;
    realSamplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate10kHz].value = 10.0;
    realSamplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate10kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate20kHz].value = 20.0;
    realSamplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate20kHz].unit = "Hz";
}

MessageDispatcher_fake_e16FastPulses::~MessageDispatcher_fake_e16FastPulses() {

}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t MessageDispatcher_fake_e16FastPulses::connect(FtdiEeprom * ftdiEeprom) {
    if (connected) {
        return ErrorDeviceAlreadyConnected;
    }

    connected = true;

    this->ftdiEeprom = ftdiEeprom;

    /*! Calculate the LSB noise vector */
    this->initializeLsbNoise();

    this->initializeCompensations();

    this->initializeFerdMemory();

    stopConnectionFlag = false;

//    rxThread = thread(&MessageDispatcher_fake_e16FastPulses::readDataFromGenerator, this);

    txThread = thread(&MessageDispatcher_fake_e16FastPulses::sendCommandsToGenerator, this);

//    gnThread = thread(&MessageDispatcher_fake_e16FastPulses::generateData, this);

//    satThread = thread(&MessageDispatcher_fake_e16FastPulses::saturationFromGenerator, this);

    return Success;
}

ErrorCodes_t MessageDispatcher_fake_e16FastPulses::disconnectDevice() {
    if (!connected) {
        return ErrorDeviceNotConnected;
    }

    this->deinit();

    if (!stopConnectionFlag) {
        stopConnectionFlag = true;

//        rxThread.join();
        txThread.join();
//        gnThread.join();
//        satThread.join();

        if (ftdiEeprom != nullptr) {
            delete ftdiEeprom;
            ftdiEeprom = nullptr;
        }

        connected = false;

        return Success;

    } else {
        return ErrorDeviceNotConnected;
    }
}

/******************************\
 *  Tx methods for generator  *
\******************************/

ErrorCodes_t MessageDispatcher_fake_e16FastPulses::setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool) {
    ErrorCodes_t ret;
    ret = MessageDispatcher::setCurrentRange(currentRangeIdx, channelIdx);
    if (ret == Success) {
        genVcCurrentResolution = currentRangesArray[currentRangeIdx];
        genVcCurrentResolution.convertValues(UnitPfxNone);
    }
    return ret;
}

ErrorCodes_t MessageDispatcher_fake_e16FastPulses::setSamplingRate(uint16_t samplingRateIdx, bool) {
    ErrorCodes_t ret;
    ret = MessageDispatcher::setSamplingRate(samplingRateIdx);
    if (ret == Success) {
        genSamplingRate = samplingRate;
        genSamplingRate.convertValue(UnitPfxNone);
        integrationItemStepsNum = (int)round(genSamplingRate.value*integrationStep);
        samplingTime = 1.0/genSamplingRate.value;
    }
    return ret;
}

void MessageDispatcher_fake_e16FastPulses::sendCommandsToGenerator() {
    DWORD bytesToWrite;

    /*! Variables used to access the tx raw buffer */
    uint32_t txRawBufferReadIdx = 0; /*!< Index being processed wrt buffer  */

    /*! Variables used to access the tx msg buffer */
    uint32_t txMsgBufferReadOffset = 0; /*!< Offset of the part of buffer to be processed  */

    /*! Variables used to access the tx data buffer */
    uint32_t txDataBufferReadIdx;

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

        bytesToWrite = (DWORD)txDataBytes;

#ifdef DEBUG_PRINT
            fprintf(fid, "\n%d %d\n", txDataBytes, bytesToWrite);
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

        txMutexLock.lock();
        txMsgBufferReadLength--;
        txMsgBufferNotFull.notify_all();
        txMutexLock.unlock();
    }
}
