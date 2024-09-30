#include "messagedispatcher_el06f.h"

using namespace std;
using namespace er4CommLib;

MessageDispatcher_EL06f::MessageDispatcher_EL06f(string id) :
    MessageDispatcher_EL06d_EL06e(id) {

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate5kHz].value = 5.0;
    samplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate10kHz].value = 10.0;
    samplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate10kHz].unit = "Hz";
    samplingRatesArray[SamplingRate20kHz].value = 20.0;
    samplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate20kHz].unit = "Hz";
    samplingRatesArray[SamplingRate40kHz].value = 40.0;
    samplingRatesArray[SamplingRate40kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate40kHz].unit = "Hz";
    samplingRatesArray[SamplingRate80kHz].value = 80.0;
    samplingRatesArray[SamplingRate80kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate80kHz].unit = "Hz";
    samplingRatesArray[SamplingRate160kHz].value = 160.0;
    samplingRatesArray[SamplingRate160kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate160kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate5kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate5kHz].value = 5.0;
    realSamplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate10kHz].value = 10.0;
    realSamplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate10kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate20kHz].value = 20.0;
    realSamplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate20kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate40kHz].value = 40.0;
    realSamplingRatesArray[SamplingRate40kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate40kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate80kHz].value = 80.0;
    realSamplingRatesArray[SamplingRate80kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate80kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate160kHz].value = 160.0;
    realSamplingRatesArray[SamplingRate160kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate160kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate5kHz].value = 200.0;
    integrationStepArray[SamplingRate5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate5kHz].unit = "s";
    integrationStepArray[SamplingRate10kHz].value = 100.0;
    integrationStepArray[SamplingRate10kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate10kHz].unit = "s";
    integrationStepArray[SamplingRate20kHz].value = 50.0;
    integrationStepArray[SamplingRate20kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate20kHz].unit = "s";
    integrationStepArray[SamplingRate40kHz].value = 25.0;
    integrationStepArray[SamplingRate40kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate40kHz].unit = "s";
    integrationStepArray[SamplingRate80kHz].value = 12.5;
    integrationStepArray[SamplingRate80kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate80kHz].unit = "s";
    integrationStepArray[SamplingRate160kHz].value = 6.25;
    integrationStepArray[SamplingRate160kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate160kHz].unit = "s";

    selectedSamplingRateIdx = defaultSamplingRateIdx;
    baseSamplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
    samplingRate = baseSamplingRate;
    integrationStep = integrationStepArray[selectedSamplingRateIdx];

    customOptionsNum = CustomOptionsNum;
    customOptionsNames.resize(customOptionsNum);
    customOptionsNames[CustomOptionClockDivider] = "Clock Divider";
    customOptionsDescriptions.resize(customOptionsNum);
    customOptionsDescriptions[CustomOptionClockDivider].resize(4);
    customOptionsDescriptions[CustomOptionClockDivider][0] = "/ 1";
    customOptionsDescriptions[CustomOptionClockDivider][1] = "/ 2";
    customOptionsDescriptions[CustomOptionClockDivider][2] = "/ 4";
    customOptionsDescriptions[CustomOptionClockDivider][3] = "/ 8";
    customOptionsDefault.resize(customOptionsNum);
    customOptionsDefault[CustomOptionClockDivider] = 0;

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;

    /*! Sampling rate */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 5;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    samplingRateCoder->addMapItem(16); /*!< 5kHz    -> 0b10000 */
    samplingRateCoder->addMapItem(17); /*!< 10kHz   -> 0b10001 */
    samplingRateCoder->addMapItem(18); /*!< 20kHz   -> 0b10010 */
    samplingRateCoder->addMapItem(19); /*!< 40kHz   -> 0b10011 */
    samplingRateCoder->addMapItem(4); /*!< 80kHz    -> 0b00100 */
    samplingRateCoder->addMapItem(5); /*!< 160kHz   -> 0b00101 */

    boolConfig.initialByte = 2;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 2;
    customOptionsCoders.resize(customOptionsNum);
    customOptionsCoders[CustomOptionClockDivider] = new BoolArrayCoder(boolConfig);
}
