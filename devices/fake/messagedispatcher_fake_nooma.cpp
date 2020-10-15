#include "messagedispatcher_fake_nooma.h"

#include <thread>
#include <ctime>
#include <cmath>
#include <sstream>

/*****************\
 *  Ctor / Dtor  *
\*****************/

MessageDispatcher_fake_Nooma::MessageDispatcher_fake_Nooma(string di) :
    MessageDispatcher_eNPR_Nooma(di) {

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

MessageDispatcher_fake_Nooma::~MessageDispatcher_fake_Nooma() {

}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t MessageDispatcher_fake_Nooma::connect(FtdiEeprom * ftdiEeprom) {
    if (connected) {
        return ErrorDeviceAlreadyConnected;
    }

    connected = true;

    this->ftdiEeprom = ftdiEeprom;

    /*! Calculate the LSB noise vector */
    this->initializeLsbNoise();

    stopConnectionFlag = false;

//    rxThread = thread(&MessageDispatcher_fake_Nooma::readAndParseMessagesForGenerator, this);

//    txThread = thread(&MessageDispatcher_fake_Nooma::unwrapAndSendMessagesForGenerator, this);

//    gnThread = thread(&MessageDispatcher_fake_Nooma::generateData, this);

//    satThread = thread(&MessageDispatcher_fake_Nooma::saturationFromGenerator, this);

    return Success;
}

ErrorCodes_t MessageDispatcher_fake_Nooma::disconnect() {
    if (!connected) {
        return ErrorDeviceNotConnected;
    }

    if (!stopConnectionFlag) {
        stopConnectionFlag = true;

//        rxThread.join();
//        txThread.join();
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

ErrorCodes_t MessageDispatcher_fake_Nooma::setCurrentRange(uint16_t currentRangeIdx) {
    ErrorCodes_t ret;
    ret = MessageDispatcher::setCurrentRange(currentRangeIdx);
    if (ret == Success) {
        genVcCurrentResolution = currentRangesArray[currentRangeIdx];
        genVcCurrentResolution.convertValues(UnitPfxNone);
    }
    return ret;
}

ErrorCodes_t MessageDispatcher_fake_Nooma::setSamplingRate(uint16_t samplingRateIdx, bool) {
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
