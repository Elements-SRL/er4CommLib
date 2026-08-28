#include "messagedispatcher_e16n_ramps.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_e16n_ramps_V01::MessageDispatcher_e16n_ramps_V01(string di) :
    MessageDispatcher_e16n_V01(di) {

    /************************\
     * Communication format *
    \************************/

    txDataBytes = 433;

    /*! Protocols parameters */
    double protocolFpgaClockFrequencyHz = 10.0e6;

    RangedMeasurement_t protocolTimeRange;
    protocolTimeRange.step = 1000.0/protocolFpgaClockFrequencyHz;
    protocolTimeRange.min = LINT32_MIN*protocolTimeRange.step;
    protocolTimeRange.max = LINT32_MAX*protocolTimeRange.step;
    protocolTimeRange.prefix = UnitPfxMilli;
    protocolTimeRange.unit = "s";

    rampTimeRange = protocolTimeRange;
    rampTimeRange.min = 0.0;

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    voltageOffsetCoders.clear();

    uint32_t vRampOffsetCodersOffset = 94;
    uint32_t vRampOffsetCodersSize = 21;

    /*! V Ramp Offset */
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    vInitRampOffsetCoders.resize(VoltageRangesNum);
    for (uint32_t rangeIdx = 0; rangeIdx < VoltageRangesNum; rangeIdx++) {
        doubleConfig.initialByte = vRampOffsetCodersOffset;
        doubleConfig.resolution = voltageRangesArray[rangeIdx].step;
        doubleConfig.minValue = voltageRangesArray[rangeIdx].min;
        doubleConfig.maxValue = voltageRangesArray[rangeIdx].max;
        vInitRampOffsetCoders[rangeIdx].resize(currentChannelsNum);
        for (uint32_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            vInitRampOffsetCoders[rangeIdx][channelIdx] = new DoubleTwosCompCoder(doubleConfig);
            doubleConfig.initialByte += vRampOffsetCodersSize;
        }
    }

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    vFinalRampOffsetCoders.resize(VoltageRangesNum);
    for (uint32_t rangeIdx = 0; rangeIdx < VoltageRangesNum; rangeIdx++) {
        doubleConfig.initialByte = vRampOffsetCodersOffset+3;
        doubleConfig.resolution = voltageRangesArray[rangeIdx].step;
        doubleConfig.minValue = voltageRangesArray[rangeIdx].min;
        doubleConfig.maxValue = voltageRangesArray[rangeIdx].max;
        vFinalRampOffsetCoders[rangeIdx].resize(currentChannelsNum);
        for (uint32_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            vFinalRampOffsetCoders[rangeIdx][channelIdx] = new DoubleTwosCompCoder(doubleConfig);
            doubleConfig.initialByte += vRampOffsetCodersSize;
        }
    }

    doubleConfig.initialByte = vRampOffsetCodersOffset+6;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 32;
    doubleConfig.resolution = rampTimeRange.step;
    doubleConfig.minValue = rampTimeRange.min;
    doubleConfig.maxValue = rampTimeRange.max;
    tRampOffsetCoders.resize(currentChannelsNum);

    for (uint32_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        tRampOffsetCoders[channelIdx] = new DoubleTwosCompCoder(doubleConfig);
        doubleConfig.initialByte += vRampOffsetCodersSize;
    }

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 32;
    quotRampOffsetCoders.resize(VoltageRangesNum);
    for (uint32_t rangeIdx = 0; rangeIdx < VoltageRangesNum; rangeIdx++) {
        doubleConfig.initialByte = vRampOffsetCodersOffset+11;
        doubleConfig.resolution = voltageRangesArray[rangeIdx].step/rampTimeRange.step;
        doubleConfig.minValue = LINT32_MIN*doubleConfig.resolution;
        doubleConfig.maxValue = LINT32_MAX*doubleConfig.resolution;
        quotRampOffsetCoders[rangeIdx].resize(currentChannelsNum);

        for (uint32_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            quotRampOffsetCoders[rangeIdx][channelIdx] = new DoubleTwosCompCoder(doubleConfig);
            doubleConfig.initialByte += vRampOffsetCodersSize;
        }
    }

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 32;
    remRampOffsetCoders.resize(VoltageRangesNum);
    for (uint32_t rangeIdx = 0; rangeIdx < VoltageRangesNum; rangeIdx++) {
        doubleConfig.initialByte = vRampOffsetCodersOffset+16;
        doubleConfig.resolution = voltageRangesArray[rangeIdx].step;
        doubleConfig.minValue = LINT32_MIN*doubleConfig.resolution;
        doubleConfig.maxValue = LINT32_MAX*doubleConfig.resolution;
        remRampOffsetCoders[rangeIdx].resize(currentChannelsNum);

        for (uint32_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
            remRampOffsetCoders[rangeIdx][channelIdx] = new DoubleTwosCompCoder(doubleConfig);
            doubleConfig.initialByte += vRampOffsetCodersSize;
        }
    }

    /*! Activate ramp Offsets */
    boolConfig.initialByte = 430;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    activateRampOffsetCoders.resize(currentChannelsNum);
    for (uint32_t idx = 0; idx < currentChannelsNum; idx++) {
        activateRampOffsetCoders[idx] = new BoolArrayCoder(boolConfig);
        boolConfig.initialBit++;
        if (boolConfig.initialBit == 7) {
            boolConfig.initialBit = 0;
            boolConfig.initialByte++;
        }
    }

    /*******************\
     * Default status  *
    \*******************/

    txStatus.resize(txDataBytes);
}

MessageDispatcher_e16n_ramps_V01::~MessageDispatcher_e16n_ramps_V01() {

}
