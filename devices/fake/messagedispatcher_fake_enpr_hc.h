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

#ifndef MESSAGEDISPATCHER_FAKE_ENPR_HC_H
#define MESSAGEDISPATCHER_FAKE_ENPR_HC_H

#include "messagedispatcher_enpr_hc.h"

/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcherFake                                    *
 *                                                                                          *
\********************************************************************************************/

class MessageDispatcher_fake_eNPR_HC : public MessageDispatcher_eNPR_HC_V00 {
public:
    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher_fake_eNPR_HC(string di);
    ~MessageDispatcher_fake_eNPR_HC();

    /************************\
     *  Connection methods  *
    \************************/

    ErrorCodes_t connect(FtdiEeprom * ftdiEeprom) override;
    ErrorCodes_t disconnectDevice() override;

    /******************************\
     *  Tx methods for generator  *
    \******************************/

    ErrorCodes_t setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool applyFlag) override;
    ErrorCodes_t setSamplingRate(uint16_t samplingRateIdx, bool applyFlag) override;

protected:
    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRatesNum
    };

    void initializeDevice() override {};

    /***********************\
     *  Signals variables  *
    \***********************/

    double genResistance = 50.0e6;

    RangedMeasurement_t genVcVoltageResolution;
    double genVoltage = 0.0;
    double genVoltageStep = 0.0;
    double genVoltageAmp = 0.0;
    double genVoltageNorm;
    uint16_t genVoltageInt;

    RangedMeasurement_t genVcCurrentResolution;
    double genCurrent = 0.0;
    double genCurrentStep = 0.0;
    double genCurrentAmp = 0.0;
    double genCurrentNorm;
    uint16_t genCurrentInt;

    Measurement_t genSamplingRate;
    double samplingTime = 0.0001;
    double integrationStep = 0.01;
    int integrationItemStepsNum;
};

#endif // MESSAGEDISPATCHER_FAKE_ENPR_HC_H
