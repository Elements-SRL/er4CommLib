#include "messagedispatcher_enpr_nooma.h"

MessageDispatcher_eNPR_Nooma::MessageDispatcher_eNPR_Nooma(string di) :
    MessageDispatcher(di) {

    ftdiEepromId = FtdiEepromId56;
    rxChannel = 'B';
    txChannel = 'B';

    rxSyncWord = 0xFFFF0000;
    txSyncWord = 0x80;

    packetsPerFrame = 16;

    fsmStateChannelsNum = 1;
    voltageChannelsNum = 2;
    currentChannelsNum = 2;
    filteredCurrentChannelsNum = currentChannelsNum;
    averagedCurrentChannelsNum = currentChannelsNum;
    totalChannelsNum = fsmStateChannelsNum+voltageChannelsNum+
            currentChannelsNum+filteredCurrentChannelsNum+averagedCurrentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+(totalChannelsNum*packetsPerFrame)*FTD_RX_WORD_SIZE;

    maxOutputPacketsNum = E4OCL_DATA_ARRAY_SIZE/totalChannelsNum;

    /*! Current ranges */
    /*! VC */
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200nA].min = -200000.0;
    currentRangesArray[CurrentRange200nA].max = 200000.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxPico;
    currentRangesArray[CurrentRange200nA].unit = "A";
    currentRangesArray[CurrentRange4uA].min = -4000000.0;
    currentRangesArray[CurrentRange4uA].max = 4000000.0;
    currentRangesArray[CurrentRange4uA].step = currentRangesArray[CurrentRange4uA].max/SHORT_MAX;
    currentRangesArray[CurrentRange4uA].prefix = UnitPfxPico;
    currentRangesArray[CurrentRange4uA].unit = "A";

    /*! Voltage ranges */
    /*! VC */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange800mV].min = -200.0;
    voltageRangesArray[VoltageRange800mV].max = 800.0;
    voltageRangesArray[VoltageRange800mV].step = 0.0625;
    voltageRangesArray[VoltageRange800mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange800mV].unit = "V";
    voltageRangesArray[VoltageRange500mV].min = -500.0;
    voltageRangesArray[VoltageRange500mV].max = 500.0;
    voltageRangesArray[VoltageRange500mV].step = 0.0625;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    voltageRangesArray[VoltageRange15V].min = -15.0;
    voltageRangesArray[VoltageRange15V].max = 15.0;
    voltageRangesArray[VoltageRange15V].step = 0.0625e-3*8.0;
    voltageRangesArray[VoltageRange15V].prefix = UnitPfxNone;
    voltageRangesArray[VoltageRange15V].unit = "V";

    voltageOffsetArray.resize(voltageRangesNum);
    for (unsigned int voltageRangeIdx = 0; voltageRangeIdx < voltageRangesNum; voltageRangeIdx++) {
        voltageOffsetArray[voltageRangeIdx] = (uint16_t)round(0.5*(voltageRangesArray[voltageRangeIdx].max+voltageRangesArray[voltageRangeIdx].min)/voltageRangesArray[voltageRangeIdx].step);
    }
    voltageOffsetArray[VoltageRange15V] = -1950*(uint16_t)round(1/0.0625); /*! half range - Vcm / dac_ext resolution */

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate200kHz].value = 200.0;
    samplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate200kHz].unit = "Hz";
    samplingRatesArray[SamplingRate312_5kHz].value = 312.5;
    samplingRatesArray[SamplingRate312_5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate312_5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate320kHz].value = 320.0;
    samplingRatesArray[SamplingRate320kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate320kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate320kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate200kHz].value = 200.0;
    realSamplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate200kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate312_5kHz].value = 312.5;
    realSamplingRatesArray[SamplingRate312_5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate312_5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate320kHz].value = 320.0;
    realSamplingRatesArray[SamplingRate320kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate320kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate200kHz].value = 5.0;
    integrationStepArray[SamplingRate200kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate200kHz].unit = "s";
    integrationStepArray[SamplingRate312_5kHz].value = 3.2;
    integrationStepArray[SamplingRate312_5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate312_5kHz].unit = "s";
    integrationStepArray[SamplingRate320kHz].value = 3.125;
    integrationStepArray[SamplingRate320kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate320kHz].unit = "s";

    txDataBytes = 205;

    /*! This will never change so it makes sense to initialize it here */
    /*! Default values */
    currentRange = currentRangesArray[CurrentRange200nA];
    currentResolution = currentRangesArray[CurrentRange200nA].step;
    voltageRange = voltageRangesArray[VoltageRange800mV];
    voltageResolution = voltageRangesArray[VoltageRange800mV].step;
    voltageOffset = voltageOffsetArray[VoltageRange800mV];
    samplingRate = realSamplingRatesArray[defaultSamplingRateIdx];
    integrationStep = integrationStepArray[defaultSamplingRateIdx];

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Device reset */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    deviceResetCoder = new BoolArrayCoder(boolConfig);

    /*! Digital offset compensation */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationCoder = new BoolArrayCoder(boolConfig);

    /*! Zap */
    zapCoders.resize(currentChannelsNum);
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    zapCoders[0] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    zapCoders[1] = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    zapStates[0] = false;
    zapStates[1] = false;

    /*! Conditioning modality */
    boolConfig.initialByte = 184;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    conditioningModalityCoder = new BoolArrayCoder(boolConfig);

    /*! Conditioning channel */
    boolConfig.initialByte = 184;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    conditioningChannelCoder = new BoolArrayCoder(boolConfig);

    sequencingVoltageRangeIdx = VoltageRange800mV;
    conditioningVoltageRangeIdx = VoltageRange15V;
    checkingVoltageRangeIdx = VoltageRange500mV;

    sequencingCurrentRangeIdx = CurrentRange200nA;
    conditioningCurrentRangeIdx = CurrentRange4uA;
    checkingCurrentRangeIdx = CurrentRange200nA;

    /*! Sampling rate */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    samplingRateCoder->addMapItem(10); /*!< 200kHz -> 0b1010 */
    samplingRateCoder->addMapItem(11); /*!< 312.5kHz -> 0b1011 */
    samplingRateCoder->addMapItem(3); /*!< 320kHz -> 0b0011 */

    /*! External DAC */
    dacExtRange.step = 0.0625;
    dacExtRange.min = 0.0;
    dacExtRange.max = 65535.0*dacExtRange.step;
    dacExtRange.prefix = UnitPfxMilli;
    dacExtRange.unit = "V";

    doubleConfig.initialByte = 5;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = dacExtRange.step;
    doubleConfig.minValue = dacExtRange.min;
    doubleConfig.maxValue = dacExtRange.max;
    doubleConfig.offset = 0.0;
    dacExtCoder = new DoubleOffsetBinaryCoder(doubleConfig);

    dacExtDefault.value = 1650.0;
    dacExtDefault.prefix = UnitPfxMilli;
    dacExtDefault.unit = "V";

    /*! Protocol selection */
    protocolsNames.resize(ProtocolsNum);
    protocolsNames[ProtocolConstant] = "Constant";
    protocolsNames[ProtocolTriangular] = "Triangular";
    protocolsNames[ProtocolSquareWave] = "Square wave";
    triangularIdx = ProtocolTriangular;

    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Holding voltage";
    protocolVoltageNames[ProtocolVPulse] = "Pulse voltage";

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 4;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltage */
    protocolVoltageRanges.resize(ProtocolVoltagesNum);
    protocolVoltageRanges[ProtocolVHold].step = 1.0;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange800mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange800mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";
    protocolVoltageRanges[ProtocolVPulse].step = 1.0;
    protocolVoltageRanges[ProtocolVPulse].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVPulse].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPulse].unit = "V";

    doubleConfig.initialBit = 4;
    doubleConfig.bitsNum = 12;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.offset = 300.0;
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 8;
    protocolVoltageCoders[ProtocolVHold] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    doubleConfig.offset = 0.0;
    doubleConfig.initialByte = 11;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);

    protocolVoltageDefault.resize(ProtocolVoltagesNum);
    protocolVoltageDefault[ProtocolVHold].value = 0.0;
    protocolVoltageDefault[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVHold].unit = "V";
    protocolVoltageDefault[ProtocolVPulse].value = 100.0;
    protocolVoltageDefault[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVPulse].unit = "V";

    /*! Triangular voltage */
    triangularVoltageRange.step = 25.0;
    triangularVoltageRange.min = 0.0;
    triangularVoltageRange.max = 4.0*triangularVoltageRange.step;
    triangularVoltageRange.prefix = UnitPfxMilli;
    triangularVoltageRange.unit = "V";

    doubleConfig.initialByte = 22;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = triangularVoltageRange.step;
    doubleConfig.minValue = triangularVoltageRange.min;
    doubleConfig.maxValue = triangularVoltageRange.max;
    doubleConfig.offset = 0.0;
    triangularVoltageCoder = new DoubleOffsetBinaryCoder(doubleConfig);

    triangularVoltageDefault.value = 100.0;
    triangularVoltageDefault.prefix = UnitPfxMilli;
    triangularVoltageDefault.unit = "V";

    /*! Protocol time */
    protocolTimesNum = ProtocolTimesNum;
    protocolTimeNames.resize(ProtocolTimesNum);
    protocolTimeNames[ProtocolTHold] = "Holding time";
    protocolTimeNames[ProtocolTPulse] = "Pulse duration";

    protocolTimeRange.step = 1.0;
    protocolTimeRange.min = 1.0;
    protocolTimeRange.max = UINT28_MAX*protocolTimeRange.step;
    protocolTimeRange.prefix = UnitPfxMilli;
    protocolTimeRange.unit = "s";

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRange.step;
    doubleConfig.minValue = 0.0;
    doubleConfig.maxValue = protocolTimeRange.max;
    doubleConfig.offset = 0.0;
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 14;
    protocolTimeCoders[ProtocolTHold] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 18;
    protocolTimeCoders[ProtocolTPulse] = new DoubleOffsetBinaryCoder(doubleConfig);

    protocolTimeDefault.resize(ProtocolTimesNum);
    protocolTimeDefault[ProtocolTHold].value = 100.0;
    protocolTimeDefault[ProtocolTHold].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTHold].unit = "s";
    protocolTimeDefault[ProtocolTPulse].value = 100.0;
    protocolTimeDefault[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPulse].unit = "s";

    /*! Triangular time */
    triangularTimeRange.step = 1.0;
    triangularTimeRange.min = triangularTimeRange.step;
    triangularTimeRange.max = 1023.0*triangularTimeRange.step;
    triangularTimeRange.prefix = UnitPfxMilli;
    triangularTimeRange.unit = "s";

    doubleConfig.initialByte = 22;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = triangularTimeRange.step;
    doubleConfig.minValue = 0.0;
    doubleConfig.maxValue = triangularTimeRange.max;
    doubleConfig.offset = 0.0;
    triangularTimeCoder = new DoubleOffsetBinaryCoder(doubleConfig);

    triangularTimeDefault.value = 100.0;
    triangularTimeDefault.prefix = UnitPfxMilli;
    triangularTimeDefault.unit = "s";

    /*! FSM flags */
    fsmFlagsNum = FsmFlagsNum;
    fsmFlagNames.resize(FsmFlagsNum);
    fsmFlagNames[FsmRun] = "Run";
    fsmFlagNames[FsmEnhance] = "Enhancements";
    fsmFlagNames[FsmEnableReScan] = "Enable Re-scan";
    fsmFlagNames[FsmEnableP2Exit] = "Enable P2 Exit";

    boolConfig.initialByte = 4;
    boolConfig.bitsNum = 1;
    fsmFlagCoders.resize(FsmFlagsNum);
    for (unsigned int idx = 0; idx < FsmFlagsNum; idx++) {
        boolConfig.initialBit = 1+idx;
        fsmFlagCoders[idx] = new BoolArrayCoder(boolConfig);
    }

    fsmFlagDefault.resize(FsmFlagsNum);
    fsmFlagDefault[FsmRun] = false;
    fsmFlagDefault[FsmEnhance] = false;
    fsmFlagDefault[FsmEnableReScan] = false;
    fsmFlagDefault[FsmEnableP2Exit] = false;

    /*! FSM voltage */
    fsmVoltagesNum = FsmVoltagesNum;
    fsmVoltageNames.resize(FsmVoltagesNum);
    fsmVoltageNames[FsmV1PreCapture] = "Pre-Capture V1";
    fsmVoltageNames[FsmV1L2R] = "L-to-R cal V1";
    fsmVoltageNames[FsmV1R2L] = "R-to-L cal V1";
    fsmVoltageNames[FsmV1PrePause] = "Pre-Pause V1";
    fsmVoltageNames[FsmV1Capture] = "Capture V1";
    fsmVoltageNames[FsmV1Pause] = "Pause V1";
    fsmVoltageNames[FsmV2Reverse] = "Reverse V2";
    fsmVoltageNames[FsmV2Capture] = "Capture V2";
    fsmVoltageNames[FsmV12Push] = "Push V12";
    fsmVoltageNames[FsmV12Pull] = "Pull V12";

    fsmVoltageRange.step = 1.0;
    fsmVoltageRange.min = -200.0*fsmVoltageRange.step;
    fsmVoltageRange.max = 800.0*fsmVoltageRange.step;
    fsmVoltageRange.prefix = UnitPfxMilli;
    fsmVoltageRange.unit = "V";

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = fsmVoltageRange.step;
    doubleConfig.minValue = fsmVoltageRange.min;
    doubleConfig.maxValue = fsmVoltageRange.max;
    doubleConfig.offset = 300.0;
    fsmVoltageCoders.resize(FsmVoltagesNum);
    for (unsigned int idx = 0; idx < FsmVoltagesNum; idx++) {
        doubleConfig.initialByte = 24+2*idx;
        fsmVoltageCoders[idx] = new DoubleTwosCompCoder(doubleConfig);
    }

    fsmVoltageDefault.resize(FsmVoltagesNum);
    fsmVoltageDefault[FsmV1PreCapture].value = 200.0;
    fsmVoltageDefault[FsmV1PreCapture].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1PreCapture].unit = "V";
    fsmVoltageDefault[FsmV1L2R].value = 250.0;
    fsmVoltageDefault[FsmV1L2R].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1L2R].unit = "V";
    fsmVoltageDefault[FsmV1R2L].value = 650.0;
    fsmVoltageDefault[FsmV1R2L].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1R2L].unit = "V";
    fsmVoltageDefault[FsmV1PrePause].value = 0.0;
    fsmVoltageDefault[FsmV1PrePause].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1PrePause].unit = "V";
    fsmVoltageDefault[FsmV1Capture].value = -200.0;
    fsmVoltageDefault[FsmV1Capture].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1Capture].unit = "V";
    fsmVoltageDefault[FsmV1Pause].value = 0.0;
    fsmVoltageDefault[FsmV1Pause].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV1Pause].unit = "V";
    fsmVoltageDefault[FsmV2Reverse].value = 300.0;
    fsmVoltageDefault[FsmV2Reverse].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV2Reverse].unit = "V";
    fsmVoltageDefault[FsmV2Capture].value = 400.0;
    fsmVoltageDefault[FsmV2Capture].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV2Capture].unit = "V";
    fsmVoltageDefault[FsmV12Push].value = 200.0;
    fsmVoltageDefault[FsmV12Push].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV12Push].unit = "V";
    fsmVoltageDefault[FsmV12Pull].value = -200.0;
    fsmVoltageDefault[FsmV12Pull].prefix = UnitPfxMilli;
    fsmVoltageDefault[FsmV12Pull].unit = "V";

    /*! FSM threshold currents */
    fsmCurrentsNum = FsmCurrentsNum;
    fsmCurrentNames.resize(FsmCurrentsNum);
    fsmCurrentNames[FsmP1PreCaptureThreshold] = "P1 Pre-Capture Threshold";
    fsmCurrentNames[FsmP1CaptureThreshold] = "P1 Capture Threshold";
    fsmCurrentNames[FsmP2CaptureThreshold] = "P2 Capture Threshold";
    fsmCurrentNames[FsmP2TagThreshold] = "P2 Tag Threshold";
    fsmCurrentNames[FsmP2TagThresholdHigh] = "P2 Tag Threshold high";
    fsmCurrentNames[FsmP2ExitThreshold] = "P2 Exit Threshold";
    fsmCurrentNames[FsmP2FoldThreshold] = "P2 Fold Threshold";
    fsmCurrentNames[FsmPullCaptureThreshold] = "Pull Capture Threshold";

    fsmThresholdCurrentRange.step = currentRangesArray[CurrentRange200nA].step;
    fsmThresholdCurrentRange.min = 0.0;
    fsmThresholdCurrentRange.max = 16383.0*fsmThresholdCurrentRange.step;
    fsmThresholdCurrentRange.prefix = UnitPfxPico;
    fsmThresholdCurrentRange.unit = "A";

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.resolution = fsmThresholdCurrentRange.step;
    doubleConfig.minValue = fsmThresholdCurrentRange.min;
    doubleConfig.maxValue = fsmThresholdCurrentRange.max;
    doubleConfig.offset = 0.0;
    fsmThresholdCurrentCoders.resize(FsmCurrentsNum);
    for (unsigned int idx = 0; idx < FsmCurrentsNum; idx++) {
        doubleConfig.initialByte = 44+2*idx;
        fsmThresholdCurrentCoders[idx] = new DoubleOffsetBinaryCoder(doubleConfig);
    }

    fsmThresholdCurrentDefault.resize(FsmCurrentsNum);
    fsmThresholdCurrentDefault[FsmP1PreCaptureThreshold].value = 100.0;
    fsmThresholdCurrentDefault[FsmP1PreCaptureThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP1PreCaptureThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmP1CaptureThreshold].value = 90.0;
    fsmThresholdCurrentDefault[FsmP1CaptureThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP1CaptureThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmP2CaptureThreshold].value = 200.0;
    fsmThresholdCurrentDefault[FsmP2CaptureThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP2CaptureThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmP2TagThreshold].value = 70.0;
    fsmThresholdCurrentDefault[FsmP2TagThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP2TagThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmP2TagThresholdHigh].value = 1000000.0;
    fsmThresholdCurrentDefault[FsmP2TagThresholdHigh].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP2TagThresholdHigh].unit = "A";
    fsmThresholdCurrentDefault[FsmP2ExitThreshold].value = 170.0;
    fsmThresholdCurrentDefault[FsmP2ExitThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP2ExitThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmP2FoldThreshold].value = 100.0;
    fsmThresholdCurrentDefault[FsmP2FoldThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmP2FoldThreshold].unit = "A";
    fsmThresholdCurrentDefault[FsmPullCaptureThreshold].value = 90.0;
    fsmThresholdCurrentDefault[FsmPullCaptureThreshold].prefix = UnitPfxPico;
    fsmThresholdCurrentDefault[FsmPullCaptureThreshold].unit = "A";

    /*! FSM time */
    fsmTimesNum = FsmTimesNum;
    fsmTimeNames.resize(FsmTimesNum);
    fsmTimeNames[FsmTransientSettleTime] = "Transient Settle Time";
    fsmTimeNames[FsmTofHoldTime] = "TOF Hold";
    fsmTimeNames[FsmPauseFilterResetTime] = "Pause Filter Reset";
    fsmTimeNames[FsmPreCaptureLowLimitTime] = "Pre-Capture low limit";
    fsmTimeNames[FsmPreCaptureHighLimitTime] = "Pre-Capture high limit";
    fsmTimeNames[FsmL2RCalibrationTime] = "L-to-R Cali Wait";
    fsmTimeNames[FsmR2LCalibrationTime] = "R-to-L Cali Wait";
    fsmTimeNames[FsmPrePauseWaitTime] = "Pre-Pause Wait";
    fsmTimeNames[FsmP1CaptureTimeout] = "P1 Capture Timeout";
    fsmTimeNames[FsmP1CaptureWaitTime] = "P1 Capture Wait";
    fsmTimeNames[FsmP2CaptureTimeout] = "P2 Capture Timeout";
    fsmTimeNames[FsmP2CaptureWaitTime] = "P2 Capture Wait";
    fsmTimeNames[FsmP2ReverseTimeout] = "P2 Reverse Timeout";
    fsmTimeNames[FsmP2ExitWaitTime] = "P2 Exit Wait";
    fsmTimeNames[FsmTagDurationMaxTime] = "Tag duration max";
    fsmTimeNames[FsmTagDurationMinTime] = "Tag duration min";
    fsmTimeNames[FsmTagWaitTime] = "Tag wait";
    fsmTimeNames[FsmTagHoldTime] = "Tag hold";
    fsmTimeNames[FsmBaselineSettleTime] = "Baseline settle";
    fsmTimeNames[FsmPushTime] = "Push time";
    fsmTimeNames[FsmPullTimeout] = "Pull Timeout";
    fsmTimeNames[FsmPullInDelayTime] = "Pull in delay";
    fsmTimeNames[FsmRecaptureTimeout] = "Recapture timeout";

    fsmTimeRange.step = 0.1;
    fsmTimeRange.min = 0.0;
    fsmTimeRange.max = UINT28_MAX*fsmTimeRange.step;
    fsmTimeRange.prefix = UnitPfxMicro;
    fsmTimeRange.unit = "s";

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = fsmTimeRange.step;
    doubleConfig.minValue = fsmTimeRange.min;
    doubleConfig.maxValue = fsmTimeRange.max;
    doubleConfig.offset = 0.0;
    fsmTimeCoders.resize(FsmTimesNum);
    for (unsigned int idx = 0; idx < FsmTimesNum; idx++) {
        doubleConfig.initialByte = 60+4*idx;
        fsmTimeCoders[idx] = new DoubleOffsetBinaryCoder(doubleConfig);
    }

    fsmTimeDefault.resize(FsmTimesNum);
    fsmTimeDefault[FsmTransientSettleTime].value = 1000.0;
    fsmTimeDefault[FsmTransientSettleTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTransientSettleTime].unit = "s";
    fsmTimeDefault[FsmTofHoldTime].value = 500.0;
    fsmTimeDefault[FsmTofHoldTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTofHoldTime].unit = "s";
    fsmTimeDefault[FsmPauseFilterResetTime].value = 100.0;
    fsmTimeDefault[FsmPauseFilterResetTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmPauseFilterResetTime].unit = "s";
    fsmTimeDefault[FsmPreCaptureLowLimitTime].value = 0.5;
    fsmTimeDefault[FsmPreCaptureLowLimitTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPreCaptureLowLimitTime].unit = "s";
    fsmTimeDefault[FsmPreCaptureHighLimitTime].value = 20.0;
    fsmTimeDefault[FsmPreCaptureHighLimitTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPreCaptureHighLimitTime].unit = "s";
    fsmTimeDefault[FsmL2RCalibrationTime].value = 11.0;
    fsmTimeDefault[FsmL2RCalibrationTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmL2RCalibrationTime].unit = "s";
    fsmTimeDefault[FsmR2LCalibrationTime].value = 11.0;
    fsmTimeDefault[FsmR2LCalibrationTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmR2LCalibrationTime].unit = "s";
    fsmTimeDefault[FsmPrePauseWaitTime].value = 10.0;
    fsmTimeDefault[FsmPrePauseWaitTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPrePauseWaitTime].unit = "s";
    fsmTimeDefault[FsmP1CaptureTimeout].value = 300.0;
    fsmTimeDefault[FsmP1CaptureTimeout].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmP1CaptureTimeout].unit = "s";
    fsmTimeDefault[FsmP1CaptureWaitTime].value = 0.5;
    fsmTimeDefault[FsmP1CaptureWaitTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmP1CaptureWaitTime].unit = "s";
    fsmTimeDefault[FsmP2CaptureTimeout].value = 2.0;
    fsmTimeDefault[FsmP2CaptureTimeout].prefix = UnitPfxNone;
    fsmTimeDefault[FsmP2CaptureTimeout].unit = "s";
    fsmTimeDefault[FsmP2CaptureWaitTime].value = 0.013;
    fsmTimeDefault[FsmP2CaptureWaitTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmP2CaptureWaitTime].unit = "s";
    fsmTimeDefault[FsmP2ReverseTimeout].value = 1.0;
    fsmTimeDefault[FsmP2ReverseTimeout].prefix = UnitPfxNone;
    fsmTimeDefault[FsmP2ReverseTimeout].unit = "s";
    fsmTimeDefault[FsmP2ExitWaitTime].value = 5000.0;
    fsmTimeDefault[FsmP2ExitWaitTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmP2ExitWaitTime].unit = "s";
    fsmTimeDefault[FsmTagDurationMaxTime].value = 2000.0;
    fsmTimeDefault[FsmTagDurationMaxTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTagDurationMaxTime].unit = "s";
    fsmTimeDefault[FsmTagDurationMinTime].value = 7.0;
    fsmTimeDefault[FsmTagDurationMinTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTagDurationMinTime].unit = "s";
    fsmTimeDefault[FsmTagWaitTime].value = 600.0;
    fsmTimeDefault[FsmTagWaitTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTagWaitTime].unit = "s";
    fsmTimeDefault[FsmTagHoldTime].value = 300.0;
    fsmTimeDefault[FsmTagHoldTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmTagHoldTime].unit = "s";
    fsmTimeDefault[FsmBaselineSettleTime].value = 30.0;
    fsmTimeDefault[FsmBaselineSettleTime].prefix = UnitPfxMicro;
    fsmTimeDefault[FsmBaselineSettleTime].unit = "s";
    fsmTimeDefault[FsmPushTime].value = 30.0;
    fsmTimeDefault[FsmPushTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPushTime].unit = "s";
    fsmTimeDefault[FsmPullTimeout].value = 1000.0;
    fsmTimeDefault[FsmPullTimeout].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPullTimeout].unit = "s";
    fsmTimeDefault[FsmPullInDelayTime].value = 20.0;
    fsmTimeDefault[FsmPullInDelayTime].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmPullInDelayTime].unit = "s";
    fsmTimeDefault[FsmRecaptureTimeout].value = 1000.0;
    fsmTimeDefault[FsmRecaptureTimeout].prefix = UnitPfxMilli;
    fsmTimeDefault[FsmRecaptureTimeout].unit = "s";

    /*! FSM integers */
    fsmIntegersNum = FsmIntegersNum;
    fsmIntegerNames.resize(FsmIntegersNum);
    fsmIntegerNames[FsmInitialTagMax] = "Initial tag max";
    fsmIntegerNames[FsmTagCountStop] = "Tag count stop";
    fsmIntegerNames[FsmScanCountMax] = "Scan count max";
    fsmIntegerNames[FsmRecaptureTagCount] = "Recapture tag count";

    fsmIntegerRange.step = 1.0;
    fsmIntegerRange.min = 0.0;
    fsmIntegerRange.max = 127.0;
    fsmIntegerRange.prefix = UnitPfxNone;
    fsmIntegerRange.unit = " ";

    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 7;
    fsmIntegerCoders.resize(FsmIntegersNum);
    for (unsigned int idx = 0; idx < FsmIntegersNum; idx++) {
        boolConfig.initialByte = 152+idx;
        fsmIntegerCoders[idx] = new BoolArrayCoder(boolConfig);
    }

    fsmIntegerDefault.resize(FsmIntegersNum);
    fsmIntegerDefault[FsmInitialTagMax].value = 2.0;
    fsmIntegerDefault[FsmInitialTagMax].prefix = UnitPfxNone;
    fsmIntegerDefault[FsmInitialTagMax].unit = " ";
    fsmIntegerDefault[FsmTagCountStop].value = 6.0;
    fsmIntegerDefault[FsmTagCountStop].prefix = UnitPfxNone;
    fsmIntegerDefault[FsmTagCountStop].unit = " ";
    fsmIntegerDefault[FsmScanCountMax].value = 4.0;
    fsmIntegerDefault[FsmScanCountMax].prefix = UnitPfxNone;
    fsmIntegerDefault[FsmScanCountMax].unit = " ";
    fsmIntegerDefault[FsmRecaptureTagCount].value = 3.0;
    fsmIntegerDefault[FsmRecaptureTagCount].prefix = UnitPfxNone;
    fsmIntegerDefault[FsmRecaptureTagCount].unit = " ";

    /*! Moving average filter */
    movingAverageFilterDurationRange.step = 1.0;
    movingAverageFilterDurationRange.min = 1.0;
    movingAverageFilterDurationRange.max = 100.0;
    movingAverageFilterDurationRange.prefix = UnitPfxMilli;
    movingAverageFilterDurationRange.unit = "s";

    doubleConfig.initialByte = 156;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = pow(2.0, -33.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    doubleConfig.offset = 0.0;
    movingAverageFilterKxCoder = new DoubleTwosCompCoder(doubleConfig);

    movingAverageFilterDurationDefault.value = 10.0;
    movingAverageFilterDurationDefault.prefix = UnitPfxMilli;
    movingAverageFilterDurationDefault.unit = "s";
    movingAverageFilterDuration = movingAverageFilterDurationDefault;

    /*! 4th order filter */
    fourthOrderFilterCutoffFrequencyRange.step = 0.1;
    fourthOrderFilterCutoffFrequencyRange.min = 1.0;
    fourthOrderFilterCutoffFrequencyRange.max = 10.0;
    fourthOrderFilterCutoffFrequencyRange.prefix = UnitPfxKilo;
    fourthOrderFilterCutoffFrequencyRange.unit = "Hz";

    doubleConfig.initialByte = 160;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = pow(2.0, -26.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    doubleConfig.offset = 0.0;
    fourthOrderFilterA01Coder = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 164;
    doubleConfig.resolution = pow(2.0, -27.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    fourthOrderFilterA02Coder = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 168;
    doubleConfig.resolution = pow(2.0, -28.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    fourthOrderFilterK0Coder = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 172;
    doubleConfig.resolution = pow(2.0, -26.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    fourthOrderFilterA11Coder = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 176;
    doubleConfig.resolution = pow(2.0, -27.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    fourthOrderFilterA12Coder = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 180;
    doubleConfig.resolution = pow(2.0, -27.0);
    doubleConfig.minValue = -doubleConfig.resolution*pow(2.0, 27.0);
    doubleConfig.maxValue = -doubleConfig.minValue-doubleConfig.resolution;
    fourthOrderFilterK1Coder = new DoubleTwosCompCoder(doubleConfig);

    fourthOrderFilterCutoffFrequencyDefault.value = 10.0;
    fourthOrderFilterCutoffFrequencyDefault.prefix = UnitPfxKilo;
    fourthOrderFilterCutoffFrequencyDefault.unit = "Hz";
    fourthOrderFilterCutoffFrequency = fourthOrderFilterCutoffFrequencyDefault;

    /*! Raw data filter */
    rawDataFilterCutoffFrequencyRange.step = 0.1;
    rawDataFilterCutoffFrequencyRange.min = 0.0;
    rawDataFilterCutoffFrequencyRange.max = 30.0;
    rawDataFilterCutoffFrequencyRange.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyRange.unit = "Hz";

    rawDataFilterCutoffFrequencyDefault.value = 30.0;
    rawDataFilterCutoffFrequencyDefault.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyDefault.unit = "Hz";
    rawDataFilterCutoffFrequency = rawDataFilterCutoffFrequencyDefault;

    /*! Conditioning protocol voltage */
    conditioningVoltagesNum = ConditioningVoltagesNum;
    conditioningProtocolVoltageNames.resize(ConditioningVoltagesNum);
    conditioningProtocolVoltageNames[ConditioningVChargePos] = "V Charge +";
    conditioningProtocolVoltageNames[ConditioningVChargeNeg] = "V Charge -";
    conditioningProtocolVoltageNames[ConditioningVObservePos] = "V Observe +";
    conditioningProtocolVoltageNames[ConditioningVObserveNeg] = "V Observe -";

    conditioningProtocolVoltageRanges.resize(ConditioningVoltagesNum);
    conditioningProtocolVoltageRanges[ConditioningVChargePos] = voltageRangesArray[VoltageRange15V];
    conditioningProtocolVoltageRanges[ConditioningVChargeNeg] = voltageRangesArray[VoltageRange15V];
    conditioningProtocolVoltageRanges[ConditioningVObservePos] = voltageRangesArray[VoltageRange15V];
    conditioningProtocolVoltageRanges[ConditioningVObservePos].convertValues(UnitPfxMilli);
    conditioningProtocolVoltageRanges[ConditioningVObserveNeg] = voltageRangesArray[VoltageRange15V];
    conditioningProtocolVoltageRanges[ConditioningVObserveNeg].convertValues(UnitPfxMilli);

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = voltageRangesArray[VoltageRange15V].step;
    doubleConfig.minValue = ((double)((int16_t)voltageOffsetArray[VoltageRange15V]))*voltageRangesArray[VoltageRange15V].step;
    doubleConfig.maxValue = (65535.0+(double)((int16_t)voltageOffsetArray[VoltageRange15V]))*voltageRangesArray[VoltageRange15V].step;
    doubleConfig.offset = 0.0;
    doubleConfig.initialByte = 185;
    conditioningProtocolVoltageCoders.resize(ConditioningVoltagesNum);
    for (unsigned int idx = 0; idx < ConditioningVoltagesNum; idx++) {
        doubleConfig.initialByte = 185+3*idx;
        conditioningProtocolVoltageCoders[idx] = new DoubleOffsetBinaryCoder(doubleConfig);
    }

    conditioningProtocolVoltageDefault.resize(ConditioningVoltagesNum);
    conditioningProtocolVoltageDefault[ConditioningVChargePos].value = 5.0;
    conditioningProtocolVoltageDefault[ConditioningVChargePos].prefix = UnitPfxNone;
    conditioningProtocolVoltageDefault[ConditioningVChargePos].unit = "V";
    conditioningProtocolVoltageDefault[ConditioningVChargeNeg].value = -5.0;
    conditioningProtocolVoltageDefault[ConditioningVChargeNeg].prefix = UnitPfxNone;
    conditioningProtocolVoltageDefault[ConditioningVChargeNeg].unit = "V";
    conditioningProtocolVoltageDefault[ConditioningVObservePos].value = 500.0;
    conditioningProtocolVoltageDefault[ConditioningVObservePos].prefix = UnitPfxMilli;
    conditioningProtocolVoltageDefault[ConditioningVObservePos].unit = "V";
    conditioningProtocolVoltageDefault[ConditioningVObserveNeg].value = -500.0;
    conditioningProtocolVoltageDefault[ConditioningVObserveNeg].prefix = UnitPfxMilli;
    conditioningProtocolVoltageDefault[ConditioningVObserveNeg].unit = "V";

    /*! Checking protocol voltage */
    checkingVoltagesNum = CheckingVoltagesNum;
    checkingProtocolVoltageNames.resize(CheckingVoltagesNum);
    checkingProtocolVoltageNames[CheckingVHold] = "Holding voltage";

    checkingProtocolVoltageRanges.resize(CheckingVoltagesNum);
    checkingProtocolVoltageRanges[CheckingVHold] = voltageRangesArray[VoltageRange500mV];

    doubleConfig.initialBit = 4;
    doubleConfig.bitsNum = 12;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = voltageRangesArray[VoltageRange500mV].min;
    doubleConfig.maxValue = voltageRangesArray[VoltageRange500mV].max;
    doubleConfig.offset = 0.0;
    doubleConfig.initialByte = 8;
    checkingProtocolVoltageCoders.resize(CheckingVoltagesNum);
    checkingProtocolVoltageCoders[CheckingVHold] = new DoubleSignAbsCoder(doubleConfig);

    checkingProtocolVoltageDefault.resize(CheckingVoltagesNum);
    checkingProtocolVoltageDefault[CheckingVHold].value = 200.0;
    checkingProtocolVoltageDefault[CheckingVHold].prefix = UnitPfxMilli;
    checkingProtocolVoltageDefault[CheckingVHold].unit = "V";

    /*! Conditioning protocol time */
    conditioningTimesNum = ConditioningTimesNum;
    conditioningProtocolTimeNames.resize(ConditioningTimesNum);
    conditioningProtocolTimeNames[ConditioningTCharge] = "T Charge";
    conditioningProtocolTimeNames[ConditioningTObserve] = "T Observe";

    conditioningProtocolTimeRanges.resize(ConditioningTimesNum);
    conditioningProtocolTimeRanges[ConditioningVChargePos] = protocolTimeRange;
    conditioningProtocolTimeRanges[ConditioningVChargeNeg] = protocolTimeRange;

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRange.step;
    doubleConfig.minValue = 0.0;
    doubleConfig.maxValue = protocolTimeRange.max;
    doubleConfig.offset = 0.0;
    conditioningProtocolTimeCoders.resize(ConditioningTimesNum);
    for (unsigned int idx = 0; idx < ConditioningTimesNum; idx++) {
        doubleConfig.initialByte = 197+4*idx;
        conditioningProtocolTimeCoders[idx] = new DoubleOffsetBinaryCoder(doubleConfig);
    }

    conditioningProtocolTimeDefault.resize(ConditioningTimesNum);
    conditioningProtocolTimeDefault[ConditioningTCharge].value = 250.0;
    conditioningProtocolTimeDefault[ConditioningTCharge].prefix = UnitPfxMilli;
    conditioningProtocolTimeDefault[ConditioningTCharge].unit = "s";
    conditioningProtocolTimeDefault[ConditioningTObserve].value = 250.0;
    conditioningProtocolTimeDefault[ConditioningTObserve].prefix = UnitPfxMilli;
    conditioningProtocolTimeDefault[ConditioningTObserve].unit = "s";

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    fEResetDenoiserCoder = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 2;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    dacExtFilterCoder = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    vcIntCoder = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 2;
    vcmForceCoder = new BoolArrayCoder(boolConfig);

    /*! Default status */
    txStatus.resize(txDataBytes);

    txStatus[0] = txSyncWord; // HDR
    txStatus[1] = 0x2E; // CG0
    txStatus[2] = 0x23; // CFG1
    txStatus[3] = 0x00; // CFG2
    txStatus[4] = 0x00; // CFG3
    txStatus[5] = 0x60; // Dac_ext
    txStatus[6] = 0x41;
    txStatus[7] = 0x01;
    txStatus[8] = 0x00; // Vhold
    txStatus[9] = 0x00;
    txStatus[10] = 0x00;
    txStatus[11] = 0x00; // Vpulse
    txStatus[12] = 0x00;
    txStatus[13] = 0x00;
    txStatus[14] = 0x10; // Thold
    txStatus[15] = 0x4E;
    txStatus[16] = 0x00;
    txStatus[17] = 0x00;
    txStatus[18] = 0x10; // Tpulse
    txStatus[19] = 0x4E;
    txStatus[20] = 0x00;
    txStatus[21] = 0x00;
    txStatus[22] = 0x24; // Triangular
    txStatus[23] = 0x00;
    txStatus[24] = 0x00; // V1Precapture
    txStatus[25] = 0x00;
    txStatus[26] = 0x00; // V1L2R
    txStatus[27] = 0x00;
    txStatus[28] = 0x00; // V1R2L
    txStatus[29] = 0x00;
    txStatus[30] = 0x00; // V1PrePause
    txStatus[31] = 0x00;
    txStatus[32] = 0x00; // V1Capture
    txStatus[33] = 0x00;
    txStatus[34] = 0x00; // V1Pause
    txStatus[35] = 0x00;
    txStatus[36] = 0x00; // V2Reverse
    txStatus[37] = 0x00;
    txStatus[38] = 0x00; // V2Capture
    txStatus[39] = 0x00;
    txStatus[40] = 0x00; // V12Push
    txStatus[41] = 0x00;
    txStatus[42] = 0x00; // V12Pull
    txStatus[43] = 0x00;
    txStatus[44] = 0x00; // p1PreCaptureThreshold
    txStatus[45] = 0x00;
    txStatus[46] = 0x00; // p1CaptureThreshold
    txStatus[47] = 0x00;
    txStatus[48] = 0x00; // p2CaptureThreshold
    txStatus[49] = 0x00;
    txStatus[50] = 0x00; // p2TagThreshold
    txStatus[51] = 0x00;
    txStatus[52] = 0x00; // p2TagThresholdHigh
    txStatus[53] = 0x00;
    txStatus[54] = 0x00; // p2ExitThreshold
    txStatus[55] = 0x00;
    txStatus[56] = 0x00; // p2FoldThreshold
    txStatus[57] = 0x00;
    txStatus[58] = 0x00; // pullCaptureThreshold
    txStatus[59] = 0x00;
    txStatus[60] = 0x01; // transientSettleTime
    txStatus[61] = 0x00;
    txStatus[62] = 0x00;
    txStatus[63] = 0x00;
    txStatus[64] = 0x01; // tofHoldTime
    txStatus[65] = 0x00;
    txStatus[66] = 0x00;
    txStatus[67] = 0x00;
    txStatus[68] = 0x01; // pauseFilterResetTime
    txStatus[69] = 0x00;
    txStatus[70] = 0x00;
    txStatus[71] = 0x00;
    txStatus[72] = 0x01; // preCaptureLowLimitTime
    txStatus[73] = 0x00;
    txStatus[74] = 0x00;
    txStatus[75] = 0x00;
    txStatus[76] = 0x01; // preCaptureHighLimitTime
    txStatus[77] = 0x00;
    txStatus[78] = 0x00;
    txStatus[79] = 0x00;
    txStatus[80] = 0x01; // L2RCalibrationTime
    txStatus[81] = 0x00;
    txStatus[82] = 0x00;
    txStatus[83] = 0x00;
    txStatus[84] = 0x01; // R2LCalibrationTime
    txStatus[85] = 0x00;
    txStatus[86] = 0x00;
    txStatus[87] = 0x00;
    txStatus[88] = 0x01; // prePauseWaitTime
    txStatus[89] = 0x00;
    txStatus[90] = 0x00;
    txStatus[91] = 0x00;
    txStatus[92] = 0x01; // p1CaptureTimeout
    txStatus[93] = 0x00;
    txStatus[94] = 0x00;
    txStatus[95] = 0x00;
    txStatus[96] = 0x01; // p1CaptureWaitTime
    txStatus[97] = 0x00;
    txStatus[98] = 0x00;
    txStatus[99] = 0x00;
    txStatus[100] = 0x01; // p2CaptureTimeout
    txStatus[101] = 0x00;
    txStatus[102] = 0x00;
    txStatus[103] = 0x00;
    txStatus[104] = 0x01; // p2CaptureWaitTime
    txStatus[105] = 0x00;
    txStatus[106] = 0x00;
    txStatus[107] = 0x00;
    txStatus[108] = 0x01; // p2ReverseTimeout
    txStatus[109] = 0x00;
    txStatus[110] = 0x00;
    txStatus[111] = 0x00;
    txStatus[112] = 0x01; // p2ExitWaitTime
    txStatus[113] = 0x00;
    txStatus[114] = 0x00;
    txStatus[115] = 0x00;
    txStatus[116] = 0x01; // tagDurationMaxTime
    txStatus[117] = 0x00;
    txStatus[118] = 0x00;
    txStatus[119] = 0x00;
    txStatus[120] = 0x01; // tagDurationMinTime
    txStatus[121] = 0x00;
    txStatus[122] = 0x00;
    txStatus[123] = 0x00;
    txStatus[124] = 0x01; // tagWaitTime
    txStatus[125] = 0x00;
    txStatus[126] = 0x00;
    txStatus[127] = 0x00;
    txStatus[128] = 0x01; // tagHoldTime
    txStatus[129] = 0x00;
    txStatus[130] = 0x00;
    txStatus[131] = 0x00;
    txStatus[132] = 0x01; // baselineSettleTime
    txStatus[133] = 0x00;
    txStatus[134] = 0x00;
    txStatus[135] = 0x00;
    txStatus[136] = 0x01; // pushTime
    txStatus[137] = 0x00;
    txStatus[138] = 0x00;
    txStatus[139] = 0x00;
    txStatus[140] = 0x01; // pullTimeout
    txStatus[141] = 0x00;
    txStatus[142] = 0x00;
    txStatus[143] = 0x00;
    txStatus[144] = 0x01; // pullInDelayTime
    txStatus[145] = 0x00;
    txStatus[146] = 0x00;
    txStatus[147] = 0x00;
    txStatus[148] = 0x01; // recaptureTimeout
    txStatus[149] = 0x00;
    txStatus[150] = 0x00;
    txStatus[151] = 0x00;
    txStatus[152] = 0x01; // initialTagMax
    txStatus[153] = 0x01; // tagCountStop
    txStatus[154] = 0x01; // scanCountMax
    txStatus[155] = 0x01; // recaptureTagCount
    txStatus[156] = 0x47; // FOF_KX
    txStatus[157] = 0x73;
    txStatus[158] = 0x36;
    txStatus[159] = 0x14;
    txStatus[160] = 0x48; // IIR_A01
    txStatus[161] = 0x20;
    txStatus[162] = 0x53;
    txStatus[163] = 0x50;
    txStatus[164] = 0x11; // IIR_A02
    txStatus[165] = 0x25;
    txStatus[166] = 0x49;
    txStatus[167] = 0x23;
    txStatus[168] = 0x42; // IIR_K0
    txStatus[169] = 0x4C;
    txStatus[170] = 0x5F;
    txStatus[171] = 0x09;
    txStatus[172] = 0x68; // IIR_A11
    txStatus[173] = 0x6C;
    txStatus[174] = 0x48;
    txStatus[175] = 0x49;
    txStatus[176] = 0x74; // IIR_A12
    txStatus[177] = 0x31;
    txStatus[178] = 0x3B;
    txStatus[179] = 0x32;
    txStatus[180] = 0x44; // IIR_K1
    txStatus[181] = 0x0B;
    txStatus[182] = 0x4D;
    txStatus[183] = 0x05;
    txStatus[184] = 0x00; // Interposer config 0
    txStatus[185] = 0x38; // VCharge+
    txStatus[186] = 0x40;
    txStatus[187] = 0x01;
    txStatus[188] = 0x08; // VCharge-
    txStatus[189] = 0x43;
    txStatus[190] = 0x01;
    txStatus[191] = 0x38; // VObserve+
    txStatus[192] = 0x40;
    txStatus[193] = 0x01;
    txStatus[194] = 0x08; // VObserve-
    txStatus[195] = 0x43;
    txStatus[196] = 0x01;
    txStatus[197] = 0x01; // TCharge
    txStatus[198] = 0x00;
    txStatus[199] = 0x00;
    txStatus[200] = 0x00;
    txStatus[201] = 0x01; // TObserve
    txStatus[202] = 0x00;
    txStatus[203] = 0x00;
    txStatus[204] = 0x00;
}

MessageDispatcher_eNPR_Nooma::~MessageDispatcher_eNPR_Nooma() {

}

void MessageDispatcher_eNPR_Nooma::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    int16_t vcOffsetInt = (int16_t)ftdiEeprom->getVcOffset();
    dacExtDefault.value = dacExtRange.step*(double)vcOffsetInt;
    this->applyDacExt(dacExtDefault, false);

    this->resetDevice(false);
    this->digitalOffsetCompensation(false);

    this->selectVoltageProtocol(0);

    /*! This one is up here, so that the vhold value is overwritten by the vhold of protocol 0 */
    for (int voltageIdx = 0; voltageIdx < CheckingVoltagesNum; voltageIdx++) {
        this->setCheckingProtocolVoltage(voltageIdx, checkingProtocolVoltageDefault[voltageIdx], false);
    }

    for (int voltageIdx = 0; voltageIdx < ProtocolVoltagesNum; voltageIdx++) {
        this->setProtocolVoltage(voltageIdx, protocolVoltageDefault[voltageIdx], false);
    }

    this->setTriangularVoltage(triangularVoltageDefault, false);

    for (int timeIdx = 0; timeIdx < ProtocolTimesNum; timeIdx++) {
        this->setProtocolTime(timeIdx, protocolTimeDefault[timeIdx], false);
    }

    this->setTriangularTime(triangularTimeDefault, false);

    for (int flagIdx = 0; flagIdx < FsmFlagsNum; flagIdx++) {
        this->setFsmFlag(flagIdx, fsmFlagDefault[flagIdx], false);
    }

    for (int voltageIdx = 0; voltageIdx < FsmVoltagesNum; voltageIdx++) {
        this->setFsmVoltage(voltageIdx, fsmVoltageDefault[voltageIdx], false);
    }

    for (int currentIdx = 0; currentIdx < FsmCurrentsNum; currentIdx++) {
        this->setFsmThresholdCurrent(currentIdx, fsmThresholdCurrentDefault[currentIdx], false);
    }

    for (int timeIdx = 0; timeIdx < FsmTimesNum; timeIdx++) {
        this->setFsmTime(timeIdx, fsmTimeDefault[timeIdx], false);
    }

    for (int integerIdx = 0; integerIdx < FsmIntegersNum; integerIdx++) {
        this->setFsmInteger(integerIdx, (unsigned int)fsmIntegerDefault[integerIdx].value, false);
    }

    this->setMovingAverageFilterDuration(movingAverageFilterDurationDefault);
    this->set4thOrderFilterCutoffFrequency(fourthOrderFilterCutoffFrequencyDefault);

    for (int voltageIdx = 0; voltageIdx < ConditioningVoltagesNum; voltageIdx++) {
        this->setConditioningProtocolVoltage(voltageIdx, conditioningProtocolVoltageDefault[voltageIdx], false);
    }

    for (int timeIdx = 0; timeIdx < ConditioningTimesNum; timeIdx++) {
        this->setConditioningProtocolTime(timeIdx, conditioningProtocolTimeDefault[timeIdx], false);
    }

    this->setConditioningCheck(true);
    this->setConditioningChannel(0);

    this->setInterposerInserted(false, false);

    this->activateFEResetDenoiser(false, false);
    this->activateDacIntFilter(false, false);
    this->activateDacExtFilter(true, false);
    this->setVcmForce(false, false);
}
