#ifndef MESSAGEDISPATCHER_FAKE_E16FASTPULSES_H
#define MESSAGEDISPATCHER_FAKE_E16FASTPULSES_H

#include "messagedispatcher_e16fastpulses.h"

using namespace std;
using namespace er4CommLib;

/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcherFake                                    *
 *                                                                                          *
\********************************************************************************************/

class MessageDispatcher_fake_e16FastPulses : public MessageDispatcher_e16FastPulses_V02 {
public:
    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher_fake_e16FastPulses(string di);
    ~MessageDispatcher_fake_e16FastPulses();

    /************************\
     *  Connection methods  *
    \************************/

    ErrorCodes_t connect(FtdiEeprom * ftdiEeprom) override;
    ErrorCodes_t disconnectDevice() override;
//    void readDataFromGenerator();
    void sendCommandsToGenerator();

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

    double wFHN = 0.0;
    double dwFHN = 0.0;
    double vFHN = 0.0;
    double dvFHN = 0.0;
    const double aFHN = 0.0007;
    const double bFHN = 0.8;
    const double tFHN = 5.0;

    Measurement_t genSamplingRate;
    double samplingTime = 0.0001;
    double integrationStep = 0.01;
    int integrationItemStepsNum;
};

#endif // MESSAGEDISPATCHER_FAKE_E16FASTPULSES_H
