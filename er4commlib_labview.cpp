#include "er4commlib_labview.h"

#include <algorithm>

#include "messagedispatcher.h"

#ifdef _WIN32
#include <windows.h>
#endif

namespace er4cl = er4CommLib;

static void input2Measurement(LVMeasurement_t i, Measurement_t &m);
static void measurement2Output(Measurement_t m, LVMeasurement_t &o);
static void rangedMeasurement2Output(RangedMeasurement_t r, LVRangedMeasurement_t &o);
static void compensationControl2Output(CompensationControl_t c, LVCompensationControl_t &o);

/************************\
 *  Connection methods  *
\************************/

ErrorCodes_t connectDevices() {
    std::vector <std::string> deviceIds;

    ErrorCodes_t ret;
    if ((ret = er4cl::detectDevices(deviceIds)) != Success) {
        return ret;
    }
    return er4cl::connect(deviceIds);
}

ErrorCodes_t disconnectDevices() {
    return er4cl::disconnect();
}

/****************\
 *  Tx methods  *
\****************/

ErrorCodes_t sendCommands() {
    return er4cl::sendCommands();
}

ErrorCodes_t selectVoltageProtocol(
        unsigned int idx) {
    return er4cl::selectVoltageProtocol(idx);
}

ErrorCodes_t applyVoltageProtocol() {
    return er4cl::applyVoltageProtocol();
}

ErrorCodes_t setProtocolVoltage(
        unsigned int idx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    return er4cl::setProtocolVoltage(idx, voltage);
}

ErrorCodes_t setProtocolTime(
        unsigned int idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    return er4cl::setProtocolTime(idx, time);
}

ErrorCodes_t setProtocolSlope(
        unsigned int idx,
        LVMeasurement_t lvSlope) {
    Measurement_t slope;
    input2Measurement(lvSlope, slope);
    return er4cl::setProtocolSlope(idx, slope);
}

ErrorCodes_t setProtocolFrequency(
        unsigned int idx,
        LVMeasurement_t lvFrequency) {
    Measurement_t frequency;
    input2Measurement(lvFrequency, frequency);
    return er4cl::setProtocolFrequency(idx, frequency);
}

ErrorCodes_t setProtocolAdimensional(
        unsigned int idx,
        LVMeasurement_t lvAdimensional) {
    Measurement_t adimensional;
    input2Measurement(lvAdimensional, adimensional);
    return er4cl::setProtocolAdimensional(idx, adimensional);
}

ErrorCodes_t checkSelectedProtocol(
        unsigned int idx) {
    std::string message;
    return er4cl::checkSelectedProtocol(idx, message);
}

ErrorCodes_t checkProtocolVoltage(
        unsigned int idx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    std::string message;
    return er4cl::checkProtocolVoltage(idx, voltage, message);
}

ErrorCodes_t checkProtocolTime(
        unsigned int idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    std::string message;
    return er4cl::checkProtocolTime(idx, time, message);
}

ErrorCodes_t checkProtocolSlope(
        unsigned int idx,
        LVMeasurement_t lvSlope) {
    Measurement_t slope;
    input2Measurement(lvSlope, slope);
    std::string message;
    return er4cl::checkProtocolSlope(idx, slope, message);
}

ErrorCodes_t checkProtocolFrequency(
        unsigned int idx,
        LVMeasurement_t lvFrequency) {
    Measurement_t frequency;
    input2Measurement(lvFrequency, frequency);
    std::string message;
    return er4cl::checkProtocolFrequency(idx, frequency, message);
}

ErrorCodes_t checkProtocolAdimensional(
        unsigned int idx,
        LVMeasurement_t lvAdimensional) {
    Measurement_t adimensional;
    input2Measurement(lvAdimensional, adimensional);
    std::string message;
    return er4cl::checkProtocolAdimensional(idx, adimensional, message);
}

ErrorCodes_t setVoltageOffset(
        unsigned int channelIdx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    return er4cl::setVoltageOffset(channelIdx, voltage);
}

ErrorCodes_t checkVoltageOffset(
        unsigned int channelIdx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    std::string message;
    return er4cl::checkVoltageOffset(channelIdx, voltage, message);
}

ErrorCodes_t applyInsertionPulse(
        LVMeasurement_t lvVoltage,
        LVMeasurement_t lvDuration) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    Measurement_t duration;
    input2Measurement(lvDuration, duration);
    return er4cl::applyInsertionPulse(voltage, duration);
}

ErrorCodes_t applyReferencePulse(
        LVMeasurement_t lvVoltage,
        LVMeasurement_t lvDuration) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    Measurement_t duration;
    input2Measurement(lvDuration, duration);
    return er4cl::applyReferencePulse(voltage, duration);
}

ErrorCodes_t applyReferencePulseTrain(
        LVMeasurement_t lvVoltage,
        LVMeasurement_t lvDuration,
        LVMeasurement_t lvPeriod,
        uint16_t number) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    Measurement_t duration;
    input2Measurement(lvDuration, duration);
    Measurement_t period;
    input2Measurement(lvPeriod, period);
    return er4cl::applyReferencePulseTrain(voltage, duration, period, number);
}

ErrorCodes_t overrideReferencePulse(
        bool flag) {
    return er4cl::overrideReferencePulse(flag);
}

ErrorCodes_t setRawDataFilter(
        LVMeasurement_t lvCutoffFrequency,
        bool lowPassFlag,
        bool activeFlag) {
    Measurement_t cutoffFrequency;
    input2Measurement(lvCutoffFrequency, cutoffFrequency);
    return er4cl::setRawDataFilter(cutoffFrequency, lowPassFlag, activeFlag);
}

ErrorCodes_t applyDacExt(
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    return er4cl::applyDacExt(voltage);
}

ErrorCodes_t setCustomFlag(
        uint16_t idx,
        bool flag) {
    return er4cl::setCustomFlag(idx, flag);
}

ErrorCodes_t setCustomDouble(
        uint16_t idx,
        double value) {
    return er4cl::setCustomDouble(idx, value);
}

ErrorCodes_t resetWasherError() {
    return er4cl::resetWasherError();
}

ErrorCodes_t setWasherPresetSpeeds(
        int8_t * lvSpeedValues) {
    std::vector <int8_t> speedValues;
    er4cl::getWasherPresetSpeeds(speedValues);
    for (unsigned int idx = 0; idx < speedValues.size(); idx++) {
        speedValues[idx] = lvSpeedValues[idx];
    }
    return er4cl::setWasherPresetSpeeds(speedValues);
}

ErrorCodes_t startWasher(
        uint16_t speedIdx) {
    return er4cl::startWasher(speedIdx);
}

ErrorCodes_t updateWasherState() {
    return er4cl::updateWasherState();
}

ErrorCodes_t updateWasherPresetSpeeds() {
    return er4cl::updateWasherPresetSpeeds();
}

ErrorCodes_t setCurrentRange(
        uint16_t currentRangeIdx,
        uint16_t channelIdx) {
    return er4cl::setCurrentRange(currentRangeIdx, channelIdx);
}

ErrorCodes_t setVoltageRange(
        uint16_t voltageRangeIdx) {
    return er4cl::setVoltageRange(voltageRangeIdx);
}

ErrorCodes_t setVoltageReferenceRange(
        uint16_t voltageRangeIdx) {
    return er4cl::setVoltageReferenceRange(voltageRangeIdx);
}

ErrorCodes_t setGpRange(
        uint16_t gpRangeIdx,
        uint16_t channelIdx) {
    return er4cl::setGpRange(gpRangeIdx, channelIdx);
}

ErrorCodes_t setSamplingRate(
        uint16_t samplingRateIdx) {
    return er4cl::setSamplingRate(samplingRateIdx);
}

ErrorCodes_t setOversamplingRatio(
        uint16_t oversamplingRatioIdx) {
    return er4cl::setOversamplingRatio(oversamplingRatioIdx);
}

ErrorCodes_t setVoltageStimulusLpf(
        uint16_t filterIdx) {
    return er4cl::setVoltageStimulusLpf(filterIdx);
}

ErrorCodes_t setVoltageReferenceLpf(
        uint16_t filterIdx) {
    return er4cl::setVoltageReferenceLpf(filterIdx);
}

ErrorCodes_t selectStimulusChannel(
        uint16_t channelIdx,
        bool on) {
    return er4cl::selectStimulusChannel(channelIdx, on);
}

ErrorCodes_t digitalOffsetCompensation(
        uint16_t channelIdx,
        bool on) {
    return er4cl::digitalOffsetCompensation(channelIdx, on);
}

ErrorCodes_t digitalOffsetCompensationAutostop(
        bool on) {
    return er4cl::digitalOffsetCompensationAutostop(on);
}

ErrorCodes_t zap(
        uint16_t channelIdx) {
    return er4cl::zap(channelIdx);
}

ErrorCodes_t switchChannelOn(
        uint16_t channelIdx,
        bool on) {
    return er4cl::switchChannelOn(channelIdx, on);
}

ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(
        uint32_t idx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    return er4cl::setFastReferencePulseProtocolWave1Voltage(idx, voltage);
}

ErrorCodes_t setFastReferencePulseProtocolWave1Time(
        uint32_t idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    return er4cl::setFastReferencePulseProtocolWave1Time(idx, time);
}

ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(
        uint32_t idx,
        LVMeasurement_t lvVoltage) {
    Measurement_t voltage;
    input2Measurement(lvVoltage, voltage);
    return er4cl::setFastReferencePulseProtocolWave2Voltage(idx, voltage);
}

ErrorCodes_t setFastReferencePulseProtocolWave2Time(
        uint32_t idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    return er4cl::setFastReferencePulseProtocolWave2Time(idx, time);
}

ErrorCodes_t setFastReferencePulseProtocolWave2Duration(
        uint32_t idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    return er4cl::setFastReferencePulseProtocolWave2Duration(idx, time);
}

ErrorCodes_t setFastReferencePulseProtocolWave2Period(
        uint32_t idx,
        LVMeasurement_t lvTime) {
    Measurement_t time;
    input2Measurement(lvTime, time);
    return er4cl::setFastReferencePulseProtocolWave2Period(idx, time);
}

ErrorCodes_t setFastReferencePulseProtocolWave2PulseNumber(
        uint32_t idx,
        uint16_t pulseNumber) {
    return er4cl::setFastReferencePulseProtocolWave2PulseNumber(idx, pulseNumber);
}

ErrorCodes_t turnOnDigitalOutput(
        bool on) {
    return er4cl::turnOnDigitalOutput(on);
}

ErrorCodes_t turnLedOn(
        uint16_t ledIndex,
        bool on) {
    return er4cl::turnLedOn(ledIndex, on);
}

ErrorCodes_t enableFrontEndResetDenoiser(
        bool on) {
    return er4cl::enableFrontEndResetDenoiser(on);
}

ErrorCodes_t resetDevice() {
    return er4cl::resetDevice();
}

ErrorCodes_t resetSynchronizationVariables() {
    er4cl::resetSynchronizationVariables();
    return Success;
}

ErrorCodes_t holdDeviceReset(
        bool flag) {
    return er4cl::holdDeviceReset(flag);
}

ErrorCodes_t resetDigitalOffsetCompensation() {
    return er4cl::resetDigitalOffsetCompensation();
}

ErrorCodes_t setCompensationsChannel(
        uint16_t channelIdx) {
    return er4cl::setCompensationsChannel(channelIdx);
}

ErrorCodes_t turnCFastCompensationOn(
        bool on) {
    return er4cl::turnCFastCompensationOn(on);
}

ErrorCodes_t setCFastCompensationOptions(
        uint16_t optionIdx) {
    return er4cl::setCFastCompensationOptions(optionIdx);
}

ErrorCodes_t setCFastCapacitance(
        LVMeasurement_t lvValue) {
    Measurement_t value;
    input2Measurement(lvValue, value);
    return er4cl::setCFastCapacitance(value);
}

ErrorCodes_t setTtlPulseTrain(
        LVMeasurement_t pulseDurationIn,
        LVMeasurement_t pulseDelayIn,
        LVMeasurement_t periodIn,
        uint32_t numberOfPulses) {
    Measurement_t pulseDuration;
    Measurement_t pulseDelay;
    Measurement_t period;
    input2Measurement(pulseDurationIn, pulseDuration);
    input2Measurement(pulseDelayIn, pulseDelay);
    input2Measurement(periodIn, period);

    return er4cl::setTtlPulseTrain(pulseDuration, pulseDelay, period, numberOfPulses);
}

ErrorCodes_t startTtlPulseTrain() {
    return er4cl::startTtlPulseTrain();
}

ErrorCodes_t setDebugBit(
        uint16_t byteOffset,
        uint16_t bitOffset,
        bool status) {
    return er4cl::setDebugBit(byteOffset, bitOffset, status);
}

ErrorCodes_t setDebugByte(
        uint16_t byteOffset,
        uint16_t byteValue) {
    return er4cl::setDebugByte(byteOffset, byteValue);
}

/****************\
 *  Rx methods  *
\****************/

ErrorCodes_t getDeviceInfo(
        uint8_t &deviceVersion,
        uint8_t &deviceSubversion,
        uint32_t &firmwareVersion) {
    return er4cl::getDeviceInfo(deviceVersion, deviceSubversion, firmwareVersion);
}

ErrorCodes_t getQueueStatus(
        QueueStatus_t &status) {
    return er4cl::getQueueStatus(status);
}

ErrorCodes_t convertVoltageValue(
        uint16_t intValue,
        double &fltValue) {
    return er4cl::convertVoltageValue(intValue, fltValue);
}

ErrorCodes_t convertCurrentValue(
        uint16_t intValue,
        uint16_t channelIdx,
        double &fltValue) {
    return er4cl::convertCurrentValue(intValue, channelIdx, fltValue);
}

ErrorCodes_t convertGpValue(
        uint16_t intValue,
        uint16_t channelIdx,
        double &fltValue) {
    return er4cl::convertGpValue(intValue, channelIdx, fltValue);
}

ErrorCodes_t readData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        int16_t * lvBuffer) {
    uint16_t * buffer = (uint16_t *)lvBuffer;
    return er4cl::readData(dataToRead, dataRead, buffer);
}

ErrorCodes_t readAllData(
        unsigned int dataToRead,
        unsigned int &dataRead,
        int16_t * lvBuffer,
        int16_t * lvUnfilteredBuffer) {
    uint16_t * buffer = (uint16_t *)lvBuffer;
    uint16_t * unfilteredBuffer = (uint16_t *)lvUnfilteredBuffer;
    return er4cl::readAllData(dataToRead, dataRead, buffer, unfilteredBuffer);
}

ErrorCodes_t purgeData() {
    return er4cl::purgeData();
}

ErrorCodes_t getChannelsNumber(
        uint32_t &voltageChannelsNum,
        uint32_t &currentChannelsNum,
        uint32_t &gpChannelsNum) {
    return er4cl::getChannelsNumber(voltageChannelsNum, currentChannelsNum, gpChannelsNum);
}

ErrorCodes_t getCurrentRangesNum(
        uint16_t &currentRangesNum) {
    std::vector <RangedMeasurement_t> currentRanges;
    std::vector <uint16_t> defaultOptions;
    ErrorCodes_t ret = er4cl::getCurrentRanges(currentRanges, defaultOptions);
    currentRangesNum = currentRanges.size();
    return ret;
}

ErrorCodes_t getNthCurrentRange(
        LVRangedMeasurement_t * lvCurrentRange,
        uint16_t currentRangeIndex) {
    std::vector <RangedMeasurement_t> currentRanges;
    std::vector <uint16_t> defaultOptions;
    ErrorCodes_t ret = er4cl::getCurrentRanges(currentRanges, defaultOptions);
    if (currentRangeIndex >= currentRanges.size()) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(currentRanges[currentRangeIndex], * lvCurrentRange);
    }
    return ret;
}

ErrorCodes_t getCurrentRange(
        LVRangedMeasurement_t * lvCurrentRange,
        uint16_t channelIdx) {
    RangedMeasurement_t currentRange;
    ErrorCodes_t ret = er4cl::getCurrentRange(currentRange, channelIdx);
    if (ret == Success) {
        rangedMeasurement2Output(currentRange, lvCurrentRange[0]);
    }
    return ret;
}

ErrorCodes_t hasIndependentCurrentRanges() {
    return er4cl::hasIndependentCurrentRanges();
}

ErrorCodes_t getGpRangesNum(
        uint16_t &gpRangesNum,
        uint16_t channelIdx) {

    std::vector <std::vector <RangedMeasurement_t>> gpRanges;
    std::vector <uint16_t> defaultOptions;
    std::vector <std::string> names;
    ErrorCodes_t ret = er4cl::getGpRanges(gpRanges, defaultOptions, names);
    if (channelIdx >= gpRanges.size()) {
        return ErrorValueOutOfRange;
    }
    gpRangesNum = gpRanges[channelIdx].size();
    return ret;
}

ErrorCodes_t getNthGpRange(
        LVRangedMeasurement_t * lvGpRange,
        uint16_t rangeIdx,
        uint16_t channelIdx) {

    std::vector <std::vector <RangedMeasurement_t>> gpRanges;
    std::vector <uint16_t> defaultOptions;
    std::vector <std::string> names;
    ErrorCodes_t ret = er4cl::getGpRanges(gpRanges, defaultOptions, names);
    if (channelIdx >= gpRanges.size()) {
        return ErrorValueOutOfRange;
    }
    if (rangeIdx >= gpRanges[channelIdx].size()) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(gpRanges[channelIdx][rangeIdx], * lvGpRange);
    }
    return ret;
}

ErrorCodes_t getGpRange(
        LVRangedMeasurement_t * lvGpRange,
        uint16_t channelIdx) {
    RangedMeasurement_t gpRange;
    ErrorCodes_t ret = er4cl::getGpRange(gpRange, channelIdx);
    if (ret == Success) {
        rangedMeasurement2Output(gpRange, lvGpRange[0]);
    }
    return ret;
}

ErrorCodes_t getVoltageRangesNum(
        uint16 &voltageRangesNum) {
    std::vector <RangedMeasurement_t> voltageRanges;
    uint16_t defaultOptions;
    std::vector <std::string> extensions;
    ErrorCodes_t ret = er4cl::getVoltageRanges(voltageRanges, defaultOptions, extensions);
    voltageRangesNum = voltageRanges.size();
    return ret;
}

ErrorCodes_t getNthVoltageRange(
        LVRangedMeasurement_t * lvVoltageRange,
        uint16_t voltageRangeIndex){
    std::vector <RangedMeasurement_t> voltageRanges;
    uint16_t defaultOptions;
    std::vector <std::string> extensions;
    ErrorCodes_t ret = er4cl::getVoltageRanges(voltageRanges, defaultOptions, extensions);
    if (voltageRangeIndex >= voltageRanges.size()) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(voltageRanges[voltageRangeIndex], * lvVoltageRange);
    }
    return ret;
}


ErrorCodes_t getVoltageRange(
        LVRangedMeasurement_t * lvVoltageRange) {
    RangedMeasurement_t voltageRange;
    ErrorCodes_t ret = er4cl::getVoltageRange(voltageRange);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceRangesNum(
        uint16_t &voltageRangesNum){
    std::vector <RangedMeasurement_t> voltageRanges;
    uint16_t defaultOption;
    ErrorCodes_t ret = er4cl::getVoltageReferenceRanges(voltageRanges, defaultOption);
    voltageRangesNum = voltageRanges.size();
    return ret;
}

ErrorCodes_t getNthVoltageReferenceRange(
        LVRangedMeasurement_t * voltageReferneceRange,
        uint16_t voltageReferneceRangeIndex){
    std::vector <RangedMeasurement_t> voltageRanges;
    uint16_t defaultOption;
    ErrorCodes_t ret = er4cl::getVoltageReferenceRanges(voltageRanges, defaultOption);
    if (voltageReferneceRangeIndex >= voltageRanges.size()) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(voltageRanges[voltageReferneceRangeIndex], voltageReferneceRange[0]);
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceRange(
        LVRangedMeasurement_t * lvRange) {
    RangedMeasurement_t range;
    ErrorCodes_t ret = er4cl::getVoltageReferenceRange(range);
    if (ret == Success) {
        rangedMeasurement2Output(range, lvRange[0]);
    }
    return ret;
}

ErrorCodes_t getSamplingRatesNum(
        uint16_t &samplingRatesNum,
        uint16_t &defaultOption) {
    std::vector <Measurement_t> samplingRates;
    ErrorCodes_t ret = er4cl::getSamplingRates(samplingRates, defaultOption);
    samplingRatesNum = samplingRates.size();
    return ret;
}

ErrorCodes_t getNthSamplingRateRange(
        LVMeasurement_t * lvSamplingRate,
        uint16_t samplingRateIndex){
    std::vector <Measurement_t> samplingRates;
    uint16_t defaultOption;
    ErrorCodes_t ret = er4cl::getSamplingRates(samplingRates, defaultOption);
    if (ret == Success){
        measurement2Output(samplingRates[samplingRateIndex], lvSamplingRate[0]);
    }
    return ret;
}

ErrorCodes_t getSamplingRate(
        LVMeasurement_t * lvSamplingRate) {
    Measurement_t samplingRate;
    ErrorCodes_t ret = er4cl::getSamplingRate(samplingRate);
    if (ret == Success) {
        measurement2Output(samplingRate, lvSamplingRate[0]);
    }
    return ret;
}

ErrorCodes_t getRealSamplingRatesNum(
        unsigned short &samplingRatesNum) {
    std::vector <Measurement_t> samplingRates;
    ErrorCodes_t ret = er4cl::getRealSamplingRates(samplingRates);
    samplingRatesNum = samplingRates.size();
    return ret;
}

ErrorCodes_t getNthRealSamplingRate(
        LVMeasurement_t * realSamplingRate,
        unsigned short realSamplingRateIndex) {
    std::vector <Measurement_t> samplingRates;
    ErrorCodes_t ret = er4cl::getRealSamplingRates(samplingRates);
    if (ret == Success) {
        measurement2Output(samplingRates[realSamplingRateIndex], realSamplingRate[0]);
    }
    return ret;
}

ErrorCodes_t getRealSamplingRate(
        LVMeasurement_t  * lvSamplingRate) {
    Measurement_t samplingRate;
    ErrorCodes_t ret = er4cl::getRealSamplingRate(samplingRate);
    if (ret == Success) {
        measurement2Output(samplingRate, lvSamplingRate[0]);
    }
    return ret;
}

ErrorCodes_t getOversamplingRatios(
        uint16_t * lvOversamplingRatios) {
    std::vector <uint16_t> oversamplingRatios;
    ErrorCodes_t ret = er4cl::getOversamplingRatios(oversamplingRatios);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < oversamplingRatios.size(); idx++) {
            lvOversamplingRatios[idx] = oversamplingRatios[idx];
        }
    }
    return ret;
}

ErrorCodes_t getOversamplingRatio(
        uint16_t &oversamplingRatio) {
    return er4cl::getOversamplingRatio(oversamplingRatio);
}

ErrorCodes_t getVoltageStimulusLpfsNum(
        uint16_t &filterOptionsNum){
    std::vector <Measurement_t> filterOptions;
    uint16_t defaultOption;
    int16_t voltageRangeIdx;
    ErrorCodes_t ret = er4cl::getVoltageStimulusLpfs(filterOptions, defaultOption, voltageRangeIdx);
    filterOptionsNum = filterOptions.size();
    return ret;
}

ErrorCodes_t getVoltageStimulusLpfs(
        LVMeasurement_t * lvFilterOptions,
        uint16_t filterIdx,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    std::vector <Measurement_t> filterOptions;
    ErrorCodes_t ret = er4cl::getVoltageStimulusLpfs(filterOptions, defaultOption, voltageRangeIdx);
    uint16_t filterOptionsNum = filterOptions.size();
    if (ret == Success && filterIdx < filterOptionsNum && filterIdx > 0) {
        measurement2Output(filterOptions[filterIdx], lvFilterOptions[0]);
    }
    return ret;
}

ErrorCodes_t getVoltageReferenceLpfsNum(
        uint16_t &filterOptionsNum){
    std::vector <Measurement_t> filterOptions;
    uint16_t defaultOption;
    int16_t voltageRangeIdx;
    ErrorCodes_t ret = er4cl::getVoltageReferenceLpfs(filterOptions, defaultOption, voltageRangeIdx);
    filterOptionsNum = filterOptions.size();
    return ret;
}

ErrorCodes_t getVoltageReferenceLpfs(
        LVMeasurement_t * lvFilterOptions,
        uint16_t filterIdx,
        uint16_t &defaultOption,
        int16_t &voltageRangeIdx) {
    std::vector <Measurement_t> filterOptions;
    ErrorCodes_t ret = er4cl::getVoltageReferenceLpfs(filterOptions, defaultOption, voltageRangeIdx);
    if (filterIdx >= filterOptions.size()) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        measurement2Output(filterOptions[filterIdx], lvFilterOptions[0]);
    }
    return ret;
}

ErrorCodes_t hasSelectStimulusChannel(
        bool &selectStimulusChannelFlag,
        bool &singleChannelSSCFlag) {
    return er4cl::hasSelectStimulusChannel(selectStimulusChannelFlag, singleChannelSSCFlag);
}

ErrorCodes_t hasDigitalOffsetCompensation(
        bool &digitalOffsetCompensationFlag,
        bool &singleChannelDOCFlag,
        bool &selectableDOCAutostopFlag) {
    return er4cl::hasDigitalOffsetCompensation(digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag);
}

ErrorCodes_t hasZap(
        bool &zappableDeviceFlag,
        bool &singleChannelZapFlag) {
    return er4cl::hasZap(zappableDeviceFlag, singleChannelZapFlag);
}

ErrorCodes_t hasChannelOn(
        bool &channelOnFlag,
        bool &singleChannelOnFlag) {
    return er4cl::hasChannelOn(channelOnFlag, singleChannelOnFlag);
}

ErrorCodes_t getSwitchedOnChannels(
        uint32_t &channelsMask) {
    return er4cl::getSwitchedOnChannels(channelsMask);
}

ErrorCodes_t hasDigitalOffsetCompensationReset() {
    return er4cl::hasDigitalOffsetCompensationReset();
}

ErrorCodes_t hasDigitalOutput() {
    return er4cl::hasDigitalOutput();
}

ErrorCodes_t hasFrontEndResetDenoiser() {
    return er4cl::hasFrontEndResetDenoiser();
}

ErrorCodes_t getProtocolList(
        uint16_t * lvVoltages,
        uint16_t * lvTimes,
        uint16_t * lvSlopes,
        uint16_t * lvFrequencies,
        uint16_t * lvAdimensionals) {
    std::vector <std::string> names;
    std::vector <std::string> images;
    std::vector <std::vector <uint16_t>> voltages;
    std::vector <std::vector <uint16_t>> times;
    std::vector <std::vector <uint16_t>> slopes;
    std::vector <std::vector <uint16_t>> frequencies;
    std::vector <std::vector <uint16_t>> adimensionals;

    ErrorCodes_t ret = er4cl::getProtocolList(names, images, voltages, times, slopes, frequencies, adimensionals);
    if (ret == Success) {
        uint32_t voltageIdx = 0;
        uint32_t timeIdx = 0;
        uint32_t slopeIdx = 0;
        uint32_t frequencyIdx = 0;
        uint32_t adimensionalIdx = 0;
        for (uint32_t idx = 0; idx < names.size(); idx++) {
            for (uint32_t idx2 = 0; idx2 < voltages.size(); idx2++) {
                lvVoltages[voltageIdx++] = voltages[idx][idx2];
            }
            lvVoltages[voltageIdx++] = -1;
            for (uint32_t idx2 = 0; idx2 < times.size(); idx2++) {
                lvTimes[timeIdx++] = times[idx][idx2];
            }
            lvTimes[timeIdx++] = -1;
            for (uint32_t idx2 = 0; idx2 < slopes.size(); idx2++) {
                lvSlopes[slopeIdx++] = slopes[idx][idx2];
            }
            lvSlopes[slopeIdx++] = -1;
            for (uint32_t idx2 = 0; idx2 < frequencies.size(); idx2++) {
                lvFrequencies[frequencyIdx++] = frequencies[idx][idx2];
            }
            lvFrequencies[frequencyIdx++] = -1;
            for (uint32_t idx2 = 0; idx2 < adimensionals.size(); idx2++) {
                lvAdimensionals[adimensionalIdx++] = adimensionals[idx][idx2];
            }
            lvAdimensionals[adimensionalIdx++] = -1;
        }
    }
    return ret;
}

ErrorCodes_t getTriangularProtocolIdx(
        uint16_t &idx) {
    return er4cl::getTriangularProtocolIdx(idx);
}

ErrorCodes_t getSealTestProtocolIdx(
        uint16_t &idx) {
    return er4cl::getSealTestProtocolIdx(idx);
}

ErrorCodes_t getProtocolVoltageNum(
        uint16_t &protocolVoltageNum){
    std::vector <std::string> voltageNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolVoltage(voltageNames, ranges, defaultValues);
    protocolVoltageNum =  ranges.size();
    return ret;
}

ErrorCodes_t getProtocolVoltageFeature(
        LVRangedMeasurement_t * lvRanges,
        LVMeasurement_t * lvDefaultValues,
        uint16_t rangeIdx) {
    std::vector <std::string> voltageNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolVoltage(voltageNames, ranges, defaultValues);
    uint16_t rangesNum =  ranges.size();
    if(rangeIdx >= rangesNum) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(ranges[rangeIdx], lvRanges[0]);
        measurement2Output(defaultValues[rangeIdx], lvDefaultValues[0]);
    }
    return ret;
}

ErrorCodes_t getProtocolTimeNum(
        uint16_t &rangesNum){
    std::vector <std::string> timeNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolTime(timeNames, ranges, defaultValues);
    rangesNum = ranges.size();
    return ret;
}

ErrorCodes_t getProtocolTimeFeature(
        LVRangedMeasurement_t * lvRanges,
        LVMeasurement_t * lvDefaultValues,
        uint16_t rangeIdx) {
    std::vector <std::string> timeNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolTime(timeNames, ranges, defaultValues);
    uint16_t rangesNum = ranges.size();
    if (rangeIdx >= rangesNum) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(ranges[rangeIdx], lvRanges[0]);
        measurement2Output(defaultValues[rangeIdx], lvDefaultValues[0]);
    }
    return ret;
}

ErrorCodes_t getProtocolSlopeNum(
        uint16_t &rangesNum){
    std::vector <std::string> slopeNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolSlope(slopeNames, ranges, defaultValues);
    rangesNum =  ranges.size();
    return ret;
}

ErrorCodes_t getProtocolSlopeFeature(
        LVRangedMeasurement_t * lvRanges,
        LVMeasurement_t * lvDefaultValues,
        uint16_t rangeIdx) {
    std::vector <std::string> slopeNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolSlope(slopeNames, ranges, defaultValues);
    uint16_t rangesNum =  ranges.size();
    if (rangeIdx >= rangesNum) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(ranges[rangeIdx], lvRanges[0]);
        measurement2Output(defaultValues[rangeIdx], lvDefaultValues[0]);
    }
    return ret;
}

ErrorCodes_t getProtocolFrequencyNum(
        uint16_t &rangesNum) {
    std::vector <std::string> frequencyNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolFrequency(frequencyNames, ranges, defaultValues);
    rangesNum =  ranges.size();
    return ret;
}

ErrorCodes_t getProtocolFrequencyFeature(
        LVRangedMeasurement_t * lvRanges,
        LVMeasurement_t * lvDefaultValues,
        uint16_t rangeIdx) {
    std::vector <std::string> frequencyNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolFrequency(frequencyNames, ranges, defaultValues);
    uint16_t rangesNum =  ranges.size();
    if (rangeIdx >= rangesNum) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(ranges[rangeIdx], lvRanges[0]);
        measurement2Output(defaultValues[rangeIdx], lvDefaultValues[0]);
    }
    return ret;
}

ErrorCodes_t getProtocolAdimensionalNum(
        uint16_t &rangesNum) {
    std::vector <std::string> adimensionalNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolAdimensional(adimensionalNames, ranges, defaultValues);
    rangesNum =  ranges.size();
    return ret;
}

ErrorCodes_t getProtocolAdimensionalFeature(
        LVRangedMeasurement_t * lvRanges,
        LVMeasurement_t * lvDefaultValues,
        uint16_t rangeIdx) {
    std::vector <std::string> adimensionalNames;
    std::vector <RangedMeasurement_t> ranges;
    std::vector <Measurement_t> defaultValues;
    ErrorCodes_t ret = er4cl::getProtocolAdimensional(adimensionalNames, ranges, defaultValues);
    uint16_t rangesNum =  ranges.size();
    if (rangeIdx >= rangesNum) {
        return ErrorValueOutOfRange;
    }
    if (ret == Success) {
        rangedMeasurement2Output(ranges[rangeIdx], lvRanges[0]);
        measurement2Output(defaultValues[rangeIdx], lvDefaultValues[0]);
    }
    return ret;
}

ErrorCodes_t getVoltageOffsetControls(
        LVRangedMeasurement_t * lvVoltageRange) {
    RangedMeasurement_t voltageRange;
    ErrorCodes_t ret = er4cl::getVoltageOffsetControls(voltageRange);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
    }
    return ret;
}

ErrorCodes_t getInsertionPulseControls(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvDurationRange) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t durationRange;
    ErrorCodes_t ret = er4cl::getInsertionPulseControls(voltageRange, durationRange);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(durationRange, lvDurationRange[0]);
    }
    return ret;
}

ErrorCodes_t hasReferencePulseControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    return er4cl::hasReferencePulseControls(referencePulseImplemented, overrideReferencePulseImplemented);
}

ErrorCodes_t getReferencePulseControls(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvDurationRange) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t durationRange;
    ErrorCodes_t ret = er4cl::getReferencePulseControls(voltageRange, durationRange);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(durationRange, lvDurationRange[0]);
    }
    return ret;
}

ErrorCodes_t hasReferencePulseTrainControls(
        bool &referencePulseImplemented,
        bool &overrideReferencePulseImplemented) {
    return er4cl::hasReferencePulseTrainControls(referencePulseImplemented, overrideReferencePulseImplemented);
}

ErrorCodes_t getReferencePulseTrainControls(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvDurationRange,
        LVRangedMeasurement_t * lvPeriodRange,
        uint16_t &pulsesNumber) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t durationRange;
    RangedMeasurement_t periodRange;
    ErrorCodes_t ret = er4cl::getReferencePulseTrainControls(voltageRange, durationRange, periodRange, pulsesNumber);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(durationRange, lvDurationRange[0]);
        rangedMeasurement2Output(periodRange, lvPeriodRange[0]);
    }
    return ret;
}

ErrorCodes_t getRawDataFilterCutoffFrequency(
        LVRangedMeasurement_t * lvRange,
        LVMeasurement_t * lvDefaultValue) {
    RangedMeasurement_t range;
    Measurement_t defaultValue;
    ErrorCodes_t ret = er4cl::getRawDataFilterCutoffFrequency(range, defaultValue);
    if (ret == Success) {
        rangedMeasurement2Output(range, lvRange[0]);
        measurement2Output(defaultValue, lvDefaultValue[0]);
    }
    return ret;
}

ErrorCodes_t getLedsNumber(
        uint16_t &ledsNum) {
    return er4cl::getLedsNumber(ledsNum);
}

ErrorCodes_t getLedsColors(
        uint32_t * lvLedsColors) {
    std::vector <uint32_t> ledsColors;
    ErrorCodes_t ret = er4cl::getLedsColors(ledsColors);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < ledsColors.size(); idx++) {
            lvLedsColors[idx] = ledsColors[idx];
        }
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave1Range(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvTimeRange,
        uint16_t &nPulse) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t timeRange;
    ErrorCodes_t ret = er4cl::getFastReferencePulseProtocolWave1Range(voltageRange, timeRange, nPulse);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(timeRange, lvTimeRange[0]);
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseProtocolWave2Range(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvTimeRange,
        LVRangedMeasurement_t * lvDurationRange,
        uint16_t &nPulse) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t timeRange;
    RangedMeasurement_t durationRange;
    ErrorCodes_t ret = er4cl::getFastReferencePulseProtocolWave2Range(voltageRange, timeRange, durationRange, nPulse);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(timeRange, lvTimeRange[0]);
        rangedMeasurement2Output(durationRange, lvDurationRange[0]);
    }
    return ret;
}

ErrorCodes_t getFastReferencePulseTrainProtocolWave2Range(
        LVRangedMeasurement_t * lvVoltageRange,
        LVRangedMeasurement_t * lvTimeRange,
        LVRangedMeasurement_t * lvDurationRange,
        LVRangedMeasurement_t * lvPeriodRange,
        uint16_t &pulsesPerTrain,
        uint16_t &nTrains) {
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t timeRange;
    RangedMeasurement_t durationRange;
    RangedMeasurement_t periodRange;
    ErrorCodes_t ret = er4cl::getFastReferencePulseTrainProtocolWave2Range(voltageRange, timeRange, durationRange, periodRange, pulsesPerTrain, nTrains);
    if (ret == Success) {
        rangedMeasurement2Output(voltageRange, lvVoltageRange[0]);
        rangedMeasurement2Output(timeRange, lvTimeRange[0]);
        rangedMeasurement2Output(durationRange, lvDurationRange[0]);
        rangedMeasurement2Output(periodRange, lvPeriodRange[0]);
    }
    return ret;
}

ErrorCodes_t getCustomFlags(
        bool * lvCustomFlagsDefault) {
    std::vector <std::string> customFlags;
    std::vector <bool> customFlagsDefault;
    ErrorCodes_t ret = er4cl::getCustomFlags(customFlags, customFlagsDefault);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < customFlags.size(); idx++) {
            lvCustomFlagsDefault[idx] = customFlagsDefault[idx];
        }
    }
    return ret;
}

ErrorCodes_t getCustomDoubles(
        LVRangedMeasurement_t * lvCustomDoublesRanges,
        double * lvCustomDoublesDefault) {
    std::vector <std::string> customDoubles;
    std::vector <RangedMeasurement_t> customDoublesRanges;
    std::vector <double> customDoublesDefault;
    ErrorCodes_t ret = er4cl::getCustomDoubles(customDoubles, customDoublesRanges, customDoublesDefault);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < customDoubles.size(); idx++) {
            rangedMeasurement2Output(customDoublesRanges[idx], lvCustomDoublesRanges[idx]);
            lvCustomDoublesDefault[idx] = customDoublesDefault[idx];
        }
    }
    return ret;
}

ErrorCodes_t hasNanionTemperatureController() {
    return er4cl::hasNanionTemperatureController();
}

ErrorCodes_t getTemperatureControllerRange(
        int &minTemperature,
        int &maxTemperature) {
    return er4cl::getTemperatureControllerRange(minTemperature, maxTemperature);
}

ErrorCodes_t hasWasherControls() {
    return er4cl::hasWasherControls();
}

ErrorCodes_t getWasherSpeedRange(
        LVRangedMeasurement_t * lvRange) {
    RangedMeasurement_t range;
    ErrorCodes_t ret = er4cl::getWasherSpeedRange(range);
    if (ret == Success) {
        rangedMeasurement2Output(range, lvRange[0]);
    }
    return ret;
}

ErrorCodes_t getWasherStatus(
        WasherStatus_t &status,
        WasherError_t &error) {
    return er4cl::getWasherStatus(status, error);
}

ErrorCodes_t getWasherPresetSpeeds(
        int8_t * lvSpeedValue) {
    std::vector <int8_t> speedValue;
    ErrorCodes_t ret = er4cl::getWasherPresetSpeeds(speedValue);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < speedValue.size(); idx++) {
            lvSpeedValue[idx] = speedValue[idx];
        }
    }
    return ret;
}

ErrorCodes_t hasCFastCompensation() {
    return er4cl::hasCFastCompensation();
}

ErrorCodes_t getCFastCapacitanceControl(
        LVCompensationControl_t * lvControl) {
    CompensationControl_t control;
    ErrorCodes_t ret = er4cl::getCFastCapacitanceControl(control);
    if (ret == Success) {
        compensationControl2Output(control, lvControl[0]);
    }
    return ret;
}

ErrorCodes_t hasTtlPulseTrain() {
    return er4cl::hasTtlPulseTrain();
}

ErrorCodes_t getVoltageOffsetCompensations(
        LVMeasurement_t * lvOffsets) {
    std::vector <Measurement_t> offsets;
    ErrorCodes_t ret = er4cl::getVoltageOffsetCompensations(offsets);
    if (ret == Success) {
        for (uint32_t idx = 0; idx < offsets.size(); idx++) {
            measurement2Output(offsets[idx], lvOffsets[idx]);
        }
    }
    return ret;
}

void input2Measurement(LVMeasurement_t i, Measurement_t &m) {
    m.value = i.value;
    m.prefix = i.prefix;
    m.unit = "";
}

void measurement2Output(Measurement_t m, LVMeasurement_t &o) {
    o.value = m.value;
    o.prefix = m.prefix;
}

void rangedMeasurement2Output(RangedMeasurement_t r, LVRangedMeasurement_t &o) {
    o.min = r.min;
    o.max = r.max;
    o.step = r.step;
    o.prefix = r.prefix;
}

void compensationControl2Output(CompensationControl_t c, LVCompensationControl_t &o) {
    o.implemented = c.implemented;
    o.min = c.min;
    o.max = c.max;
    o.compensable = c.compensable;
    o.steps = c.steps;
    o.step = c.step;
    o.decimals = c.decimals;
    o.value = c.value;
    o.prefix = c.prefix;
}
