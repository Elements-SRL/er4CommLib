#include "messagedispatcher_fake_enpr_hc.h"

MessageDispatcher_fake_eNPR_HC::MessageDispatcher_fake_eNPR_HC(string di) :
    MessageDispatcher_eNPR_HC_V02(di) {

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

MessageDispatcher_fake_eNPR_HC::~MessageDispatcher_fake_eNPR_HC() {

}

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t MessageDispatcher_fake_eNPR_HC::connect(FtdiEeprom * ftdiEeprom) {
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

ErrorCodes_t MessageDispatcher_fake_eNPR_HC::disconnectDevice() {
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

ErrorCodes_t MessageDispatcher_fake_eNPR_HC::setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool) {
    ErrorCodes_t ret;
    ret = MessageDispatcher::setCurrentRange(currentRangeIdx, channelIdx);
    if (ret == Success) {
        genVcCurrentResolution = currentRangesArray[currentRangeIdx];
        genVcCurrentResolution.convertValues(UnitPfxNone);
    }
    return ret;
}

ErrorCodes_t MessageDispatcher_fake_eNPR_HC::setSamplingRate(uint16_t samplingRateIdx, bool) {
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
