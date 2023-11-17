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

#include "messagedispatcher_fake_e16n.h"

#include <thread>
#include <ctime>
#include <cmath>
#include <sstream>

/*****************\
 *  Ctor / Dtor  *
\*****************/

MessageDispatcher_fake_e16n::MessageDispatcher_fake_e16n(string di) :
    MessageDispatcher_e16n_V01(di) {

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

MessageDispatcher_fake_e16n::~MessageDispatcher_fake_e16n() {

}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t MessageDispatcher_fake_e16n::connect(FtdiEeprom * ftdiEeprom) {
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

    return Success;
}

ErrorCodes_t MessageDispatcher_fake_e16n::disconnectDevice() {
    if (!connected) {
        return ErrorDeviceNotConnected;
    }

    this->deinit();

    if (!stopConnectionFlag) {
        stopConnectionFlag = true;

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

ErrorCodes_t MessageDispatcher_fake_e16n::setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool) {
    ErrorCodes_t ret;
    ret = MessageDispatcher::setCurrentRange(currentRangeIdx, channelIdx);
    if (ret == Success) {
        genVcCurrentResolution = currentRangesArray[currentRangeIdx];
        genVcCurrentResolution.convertValues(UnitPfxNone);
    }
    return ret;
}

ErrorCodes_t MessageDispatcher_fake_e16n::setSamplingRate(uint16_t samplingRateIdx, bool) {
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
