#include "messagedispatcher_e16n.h"

MessageDispatcher_e16n::MessageDispatcher_e16n(string di) :
    MessageDispatcher(di) {

    ftdiEepromId = FtdiEepromId56;
    rxChannel = 'B';
    txChannel = 'B';

    rxSyncWord = 0xFFFF0000;
    txSyncWord = 0x80;

    packetsPerFrame = 16;

    voltageChannelsNum = 1;
    currentChannelsNum = 16;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+(totalChannelsNum*packetsPerFrame)*FTD_RX_WORD_SIZE;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    /*! Current ranges */
    /*! VC */
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200pA].min = -200.0;
    currentRangesArray[CurrentRange200pA].max = 200.0;
    currentRangesArray[CurrentRange200pA].step = currentRangesArray[CurrentRange200pA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200pA].prefix = UnitPfxPico;
    currentRangesArray[CurrentRange200pA].unit = "A";
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange2nA].min = -2.0;
    currentRangesArray[CurrentRange2nA].max = 2.0;
    currentRangesArray[CurrentRange2nA].step = currentRangesArray[CurrentRange2nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange2nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange2nA].unit = "A";
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange20nA].min = -20.0;
    currentRangesArray[CurrentRange20nA].max = 20.0;
    currentRangesArray[CurrentRange20nA].step = currentRangesArray[CurrentRange20nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange20nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange20nA].unit = "A";
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200nA].min = -200.0;
    currentRangesArray[CurrentRange200nA].max = 200.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange200nA].unit = "A";
    defaultCurrentRangeIdx = CurrentRange200pA;

    /*! Voltage ranges */
    /*! VC */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange500mV].min = -511.0;
    voltageRangesArray[VoltageRange500mV].max = 511.0;
    voltageRangesArray[VoltageRange500mV].step = 1.0;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange500mV;

    voltageOffsetArray.resize(voltageRangesNum);
    for (unsigned int voltageRangeIdx = 0; voltageRangeIdx < voltageRangesNum; voltageRangeIdx++) {
        voltageOffsetArray[voltageRangeIdx] = (uint16_t)round(0.5*(voltageRangesArray[voltageRangeIdx].max+voltageRangesArray[voltageRangeIdx].min)/voltageRangesArray[voltageRangeIdx].step);
    }
    voltageOffsetArray[VoltageRange500mV] = 65536-512;

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate1_25kHz].value = 1.25;
    samplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    samplingRatesArray[SamplingRate5kHz].value = 5.0;
    samplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate10kHz].value = 10.0;
    samplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate10kHz].unit = "Hz";
    samplingRatesArray[SamplingRate20kHz].value = 20.0;
    samplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate20kHz].unit = "Hz";
    samplingRatesArray[SamplingRate50kHz].value = 50.0;
    samplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate50kHz].unit = "Hz";
    samplingRatesArray[SamplingRate100kHz].value = 100.0;
    samplingRatesArray[SamplingRate100kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate100kHz].unit = "Hz";
    samplingRatesArray[SamplingRate200kHz].value = 200.0;
    samplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate200kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate1_25kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate1_25kHz].value = 1.25e3/1024.0;
    realSamplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate5kHz].value = 1.25e3/256.0;
    realSamplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate10kHz].value = 1.25e3/128.0;
    realSamplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate10kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate20kHz].value = 1.25e3/64.0;
    realSamplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate20kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate50kHz].value = 50.0;
    realSamplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate50kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate100kHz].value = 100.0;
    realSamplingRatesArray[SamplingRate100kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate100kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate200kHz].value = 200.0;
    realSamplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate200kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate1_25kHz].value = 1024.0/1.25;
    integrationStepArray[SamplingRate1_25kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate1_25kHz].unit = "s";
    integrationStepArray[SamplingRate5kHz].value = 256.0/1.25;
    integrationStepArray[SamplingRate5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate5kHz].unit = "s";
    integrationStepArray[SamplingRate10kHz].value = 128.0/1.25;
    integrationStepArray[SamplingRate10kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate10kHz].unit = "s";
    integrationStepArray[SamplingRate20kHz].value = 64.0/1.25;
    integrationStepArray[SamplingRate20kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate20kHz].unit = "s";
    integrationStepArray[SamplingRate50kHz].value = 20.0;
    integrationStepArray[SamplingRate50kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate50kHz].unit = "s";
    integrationStepArray[SamplingRate100kHz].value = 10.0;
    integrationStepArray[SamplingRate100kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate100kHz].unit = "s";
    integrationStepArray[SamplingRate200kHz].value = 5.0;
    integrationStepArray[SamplingRate200kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate200kHz].unit = "s";

    txDataBytes = 90;

    /*! Default values */
    selectedVoltageRangeIdx = defaultVoltageRangeIdx;
    selectedCurrentRangeIdx = defaultCurrentRangeIdx;
    selectedSamplingRateIdx = defaultSamplingRateIdx;

    currentRange = currentRangesArray[selectedCurrentRangeIdx];
    currentResolution = currentRangesArray[selectedCurrentRangeIdx].step;
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step;
    voltageOffset = voltageOffsetArray[selectedVoltageRangeIdx];
    samplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
    integrationStep = integrationStepArray[selectedSamplingRateIdx];

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Device reset */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    deviceResetCoder = new BoolArrayCoder(boolConfig);

    /*! Digital offset compensations */
    digitalOffsetCompensationFlag = true;
    singleChannelDOCFlag = true;

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 16;
    digitalOffsetCompensationCoder = new BoolArrayCoder(boolConfig);

    digitalOffsetCompensationStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        digitalOffsetCompensationStates[currentIdx] = false;
    }

    boolConfig.initialByte = 8;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationResetCoder = new BoolArrayCoder(boolConfig);

    /*! Zap */
    zappableDeviceFlag = true;
    singleChannelZapFlag = true;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 16;
    zapCoder = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        zapStates[currentIdx] = false;
    }

    /*! Channel off */
    channelOnFlag = true;
    singleChannelOnFlag = true;

    boolConfig.initialByte = 10;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 16;
    channelOnCoder = new BoolNegatedArrayCoder(boolConfig);

    channelOnStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        channelOnStates[currentIdx] = false;
    }

    /*! Channel select */
    boolConfig.initialByte = 13;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 16;
    channelSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Current rate */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 3;
    currentRangeCoder = new BoolRandomArrayCoder(boolConfig);
    currentRangeCoder->addMapItem(0); /*!< 200pA    -> 0b000 */
    currentRangeCoder->addMapItem(2); /*!< 2nA      -> 0b010 */
    currentRangeCoder->addMapItem(3); /*!< 20nA     -> 0b011 */
    currentRangeCoder->addMapItem(7); /*!< 200pA    -> 0b111 */

    /*! Voltage range */
    boolConfig.initialByte = 0;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    voltageRangeCoder = new BoolRandomArrayCoder(boolConfig);
    voltageRangeCoder->addMapItem(0); /*!< No controls  -> 0b0 */

    /*! Sampling rate */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 4;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    samplingRateCoder->addMapItem(0); /*!< 1.25kHz  -> 0b0000 */
    samplingRateCoder->addMapItem(1); /*!< 5kHz     -> 0b0001 */
    samplingRateCoder->addMapItem(2); /*!< 10kHz    -> 0b0010 */
    samplingRateCoder->addMapItem(3); /*!< 20kHz    -> 0b0011 */
    samplingRateCoder->addMapItem(8); /*!< 50kHz    -> 0b1000 */
    samplingRateCoder->addMapItem(9); /*!< 100kHz   -> 0b1001 */
    samplingRateCoder->addMapItem(10); /*!< 200kHz  -> 0b1010 */

    /*! Protocol selection */
    protocolsNames.resize(ProtocolsNum);
    protocolsNames[ProtocolConstant] = "Constant";
    protocolsNames[ProtocolTriangular] = "Triangular";
    protocolsNames[ProtocolSquareWave] = "Square wave";
    protocolsNames[ProtocolConductance] = "Conductance";
    protocolsNames[ProtocolVariableAmplitude] = "Variable Amplitude";
    protocolsNames[ProtocolVariableDuration] = "Variable Duration";
    protocolsNames[ProtocolRamp] = "Ramp";
    protocolsNames[ProtocolCyclicVoltammetry] = "Cyclic Voltammetry";

    boolConfig.initialByte = 9;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 9;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    defaultProtocol = ProtocolConstant;

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Holding voltage";
    protocolVoltageNames[ProtocolVPulse] = "Pulse voltage";
    protocolVoltageNames[ProtocolVStep] = "Voltage step";
    protocolVoltageNames[ProtocolVPk] = "Voltage peak";
    protocolVoltageNames[ProtocolVMax] = "Max voltage";
    protocolVoltageNames[ProtocolVMin] = "Min voltage";

    protocolVoltageRanges.resize(ProtocolVoltagesNum);
    protocolVoltageRanges[ProtocolVHold].step = 1.0;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";
    protocolVoltageRanges[ProtocolVPulse].step = 1.0;
    protocolVoltageRanges[ProtocolVPulse].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVPulse].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPulse].unit = "V";
    protocolVoltageRanges[ProtocolVStep].step = 1.0;
    protocolVoltageRanges[ProtocolVStep].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVStep].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVStep].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVStep].unit = "V";
    protocolVoltageRanges[ProtocolVPk].step = 25.0;
    protocolVoltageRanges[ProtocolVPk].min = 0.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVMax].step = 1.0;
    protocolVoltageRanges[ProtocolVMax].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVMax].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVMax].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVMax].unit = "V";
    protocolVoltageRanges[ProtocolVMin].step = 1.0;
    protocolVoltageRanges[ProtocolVMin].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVMin].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVMin].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVMin].unit = "V";

    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 16;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVHold] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 50;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 55;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 74;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVPk] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 78;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMax].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMax].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMax].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVMax] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 80;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMin].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMin].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMin].max;
    doubleConfig.offset = 0.0;
    protocolVoltageCoders[ProtocolVMin] = new DoubleSignAbsCoder(doubleConfig);

    protocolVoltageDefault.resize(ProtocolVoltagesNum);
    protocolVoltageDefault[ProtocolVHold].value = 0.0;
    protocolVoltageDefault[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVHold].unit = "V";
    protocolVoltageDefault[ProtocolVPulse].value = 100.0;
    protocolVoltageDefault[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVPulse].unit = "V";
    protocolVoltageDefault[ProtocolVStep].value = 100.0;
    protocolVoltageDefault[ProtocolVStep].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVStep].unit = "V";
    protocolVoltageDefault[ProtocolVPk].value = 100.0;
    protocolVoltageDefault[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVPk].unit = "V";
    protocolVoltageDefault[ProtocolVMax].value = 100.0;
    protocolVoltageDefault[ProtocolVMax].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVMax].unit = "V";
    protocolVoltageDefault[ProtocolVMin].value = -100.0;
    protocolVoltageDefault[ProtocolVMin].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVMin].unit = "V";

    /*! Protocol times */
    protocolTimesNum = ProtocolTimesNum;
    protocolTimeNames.resize(ProtocolTimesNum);
    protocolTimeNames[ProtocolTHold] = "Holding time";
    protocolTimeNames[ProtocolTPulse] = "Pulse duration";
    protocolTimeNames[ProtocolTStep] = "Duration step";
    protocolTimeNames[ProtocolTPe] = "Period";

    protocolTimeRanges.resize(ProtocolTimesNum);
    protocolTimeRanges[ProtocolTHold].step = 1.0;
    protocolTimeRanges[ProtocolTHold].min = 1.0;
    protocolTimeRanges[ProtocolTHold].max = UINT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTHold].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTHold].unit = "s";
    protocolTimeRanges[ProtocolTPulse].step = 1.0;
    protocolTimeRanges[ProtocolTPulse].min = 1.0;
    protocolTimeRanges[ProtocolTPulse].max = UINT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTPulse].unit = "s";
    protocolTimeRanges[ProtocolTStep].step = 1.0;
    protocolTimeRanges[ProtocolTStep].min = INT28_MIN*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTStep].max = INT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTStep].unit = "s";
    protocolTimeRanges[ProtocolTPe].step = 1.0;
    protocolTimeRanges[ProtocolTPe].min = protocolTimeRanges[ProtocolTPe].step;
    protocolTimeRanges[ProtocolTPe].max = UINT10_MAX*protocolTimeRanges[ProtocolTPe].step;
    protocolTimeRanges[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTPe].unit = "s";

    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 56;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    doubleConfig.offset = 0.0;
    protocolTimeCoders[ProtocolTHold] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 60;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    doubleConfig.offset = 0.0;
    protocolTimeCoders[ProtocolTPulse] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 66;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTStep].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTStep].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTStep].max;
    doubleConfig.offset = 0.0;
    protocolTimeCoders[ProtocolTStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 74;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    doubleConfig.offset = 0.0;
    protocolTimeCoders[ProtocolTPe] = new DoubleOffsetBinaryCoder(doubleConfig);

    protocolTimeDefault.resize(ProtocolTimesNum);
    protocolTimeDefault[ProtocolTHold].value = 100.0;
    protocolTimeDefault[ProtocolTHold].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTHold].unit = "s";
    protocolTimeDefault[ProtocolTPulse].value = 100.0;
    protocolTimeDefault[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPulse].unit = "s";
    protocolTimeDefault[ProtocolTStep].value = 100.0;
    protocolTimeDefault[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTStep].unit = "s";
    protocolTimeDefault[ProtocolTPe].value = 100.0;
    protocolTimeDefault[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPe].unit = "s";

    /*! Protocol slope */
    protocolSlopesNum = ProtocolSlopesNum;
    protocolSlopeNames.resize(ProtocolSlopesNum);
    protocolSlopeNames[ProtocolSlope] = "Slope";

    protocolSlopeRanges.resize(ProtocolSlopesNum);
    protocolSlopeRanges[ProtocolSlope].step = 1.0;
    protocolSlopeRanges[ProtocolSlope].min = 1.0;
    protocolSlopeRanges[ProtocolSlope].max = UINT10_MAX*protocolSlopeRanges[ProtocolSlope].step;
    protocolSlopeRanges[ProtocolSlope].prefix = UnitPfxMilli;
    protocolSlopeRanges[ProtocolSlope].unit = "V/ms";

    protocolSlopeCoders.resize(ProtocolSlopesNum);
    doubleConfig.initialByte = 76;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolSlopeRanges[ProtocolSlope].step;
    doubleConfig.minValue = protocolSlopeRanges[ProtocolSlope].min;
    doubleConfig.maxValue = protocolSlopeRanges[ProtocolSlope].max;
    doubleConfig.offset = 0.0;
    protocolSlopeCoders[ProtocolSlope] = new DoubleOffsetBinaryCoder(doubleConfig);

    protocolSlopeDefault.resize(ProtocolSlopesNum);
    protocolSlopeDefault[ProtocolSlope].value = 1.0;
    protocolSlopeDefault[ProtocolSlope].prefix = UnitPfxMilli;
    protocolSlopeDefault[ProtocolSlope].unit = "s";

    /*! Protocol integers */
    protocolIntegersNum = ProtocolIntegersNum;
    protocolIntegerNames.resize(ProtocolIntegersNum);
    protocolIntegerNames[ProtocolN] = "N";
    protocolIntegerNames[ProtocolNR] = "NR";

    protocolIntegerRanges.resize(ProtocolIntegersNum);
    protocolIntegerRanges[ProtocolN].step = 1.0;
    protocolIntegerRanges[ProtocolN].min = 1.0;
    protocolIntegerRanges[ProtocolN].max = UINT10_MAX*protocolIntegerRanges[ProtocolN].step;
    protocolIntegerRanges[ProtocolN].prefix = UnitPfxNone;
    protocolIntegerRanges[ProtocolN].unit = "";
    protocolIntegerRanges[ProtocolNR].step = 1.0;
    protocolIntegerRanges[ProtocolNR].min = 0.0;
    protocolIntegerRanges[ProtocolNR].max = UINT10_MAX*protocolIntegerRanges[ProtocolNR].step;
    protocolIntegerRanges[ProtocolNR].prefix = UnitPfxNone;
    protocolIntegerRanges[ProtocolNR].unit = "";

    protocolIntegerCoders.resize(ProtocolIntegersNum);
    boolConfig.initialByte = 70;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 10;
    protocolIntegerCoders[ProtocolN] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 72;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 10;
    protocolIntegerCoders[ProtocolNR] = new BoolArrayCoder(boolConfig);

    protocolIntegerDefault.resize(ProtocolIntegersNum);
    protocolIntegerDefault[ProtocolN] = 5;
    protocolIntegerDefault[ProtocolNR] = 0;

    edhFormat =
            "EDH Version: 2.0\n"
            "\n"
            "Elements e16n\n"
            "Channels: 16\n"
            "\n"
            "Data header file\n"
            "\n"
            "Amplifier Setup\n"
            "Range: %currentRange%\n" // 200 pA
            "Sampling frequency (SR): %samplingRate%\n" // 1.25 kHz
            "Final Bandwidth: SR/2 (no filter)\n"
            "\n"
            "Acquisition start time: %dateHour%\n" // 04/11/2020 11:28:55.130
            "\n"
            "Active channels: %activeChannels%\n"; // 2 3 4 5 6 7 8 9 10 12 13 14 15 16

    /*! Raw data filter */
    rawDataFilterCutoffFrequencyRange.step = 0.1;
    rawDataFilterCutoffFrequencyRange.min = 0.0;
    rawDataFilterCutoffFrequencyRange.max = 100.0;
    rawDataFilterCutoffFrequencyRange.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyRange.unit = "Hz";

    rawDataFilterCutoffFrequencyDefault.value = 30.0;
    rawDataFilterCutoffFrequencyDefault.prefix = UnitPfxKilo;
    rawDataFilterCutoffFrequencyDefault.unit = "Hz";
    rawDataFilterCutoffFrequency = rawDataFilterCutoffFrequencyDefault;

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolArrayCoder(boolConfig);

    /*! Default status */
    txStatus.resize(txDataBytes);

    txStatus[0] = txSyncWord; // HDR
    txStatus[1] = 0x20; // CG0
    txStatus[2] = 0x03; // CFG1
    txStatus[3] = 0x00; // CFG2
    txStatus[4] = 0x00; // CFG3
    txStatus[5] = 0x00; // CFG4
    txStatus[6] = 0x00; // CFG5
    txStatus[7] = 0x00; // CFG6
    txStatus[8] = 0x00; // CFG7
    txStatus[9] = 0x00; // CFG8
    txStatus[10] = 0x00; // CFG9
    txStatus[11] = 0x00; // CFG10
    txStatus[12] = 0x00; // CFG11
    txStatus[13] = 0x7F; // CFG12
    txStatus[14] = 0x7F; // CFG13
    txStatus[15] = 0x03; // CFG14
    txStatus[16] = 0x00; // Vhold
    txStatus[17] = 0x00;
    txStatus[18] = 0x00; // VOfs1
    txStatus[19] = 0x00;
    txStatus[20] = 0x00; // VOfs2
    txStatus[21] = 0x00;
    txStatus[22] = 0x00; // VOfs3
    txStatus[23] = 0x00;
    txStatus[24] = 0x00; // VOfs4
    txStatus[25] = 0x00;
    txStatus[26] = 0x00; // VOfs5
    txStatus[27] = 0x00;
    txStatus[28] = 0x00; // VOfs6
    txStatus[29] = 0x00;
    txStatus[30] = 0x00; // VOfs7
    txStatus[31] = 0x00;
    txStatus[32] = 0x00; // VOfs8
    txStatus[33] = 0x00;
    txStatus[34] = 0x00; // VOfs9
    txStatus[35] = 0x00;
    txStatus[36] = 0x00; // VOfs10
    txStatus[37] = 0x00;
    txStatus[38] = 0x00; // VOfs11
    txStatus[39] = 0x00;
    txStatus[40] = 0x00; // VOfs12
    txStatus[41] = 0x00;
    txStatus[42] = 0x00; // VOfs13
    txStatus[43] = 0x00;
    txStatus[44] = 0x00; // VOfs14
    txStatus[45] = 0x00;
    txStatus[46] = 0x00; // VOfs15
    txStatus[47] = 0x00;
    txStatus[48] = 0x00; // VOfs16
    txStatus[49] = 0x00;
    txStatus[50] = 0x00; // VPulse
    txStatus[51] = 0x00;
    txStatus[52] = 0x00; // VInsPulse
    txStatus[53] = 0x00;
    txStatus[54] = 0x00; // VStep
    txStatus[55] = 0x00;
    txStatus[56] = 0x00; // THold
    txStatus[57] = 0x00;
    txStatus[58] = 0x00;
    txStatus[59] = 0x00;
    txStatus[60] = 0x00; // TPulse
    txStatus[61] = 0x00;
    txStatus[62] = 0x00;
    txStatus[63] = 0x00;
    txStatus[64] = 0x00; // TInsPulse
    txStatus[65] = 0x00;
    txStatus[66] = 0x00; // TStep
    txStatus[67] = 0x00;
    txStatus[68] = 0x00;
    txStatus[69] = 0x00;
    txStatus[70] = 0x00; // N
    txStatus[71] = 0x00;
    txStatus[72] = 0x00; // NR
    txStatus[73] = 0x00;
    txStatus[74] = 0x00; // Triangular
    txStatus[75] = 0x00;
    txStatus[76] = 0x00; // Slope
    txStatus[77] = 0x00;
    txStatus[78] = 0x00; // VMax
    txStatus[79] = 0x00;
    txStatus[80] = 0x00; // VMin
    txStatus[81] = 0x00;
    txStatus[82] = 0x00;
    txStatus[83] = 0x00;
    txStatus[84] = 0x00;
    txStatus[85] = 0x00;
    txStatus[86] = 0x00;
    txStatus[87] = 0x00;
    txStatus[88] = 0x00;
    txStatus[89] = 0x00;
}

MessageDispatcher_e16n::~MessageDispatcher_e16n() {

}

void MessageDispatcher_e16n::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->digitalOffsetCompensation(currentChannelsNum, false);
    this->switchChannelOn(currentChannelsNum, true, false);

    this->selectVoltageProtocol(defaultProtocol);

    for (int voltageIdx = 0; voltageIdx < ProtocolVoltagesNum; voltageIdx++) {
        this->setProtocolVoltage(voltageIdx, protocolVoltageDefault[voltageIdx], false);
    }

    for (int timeIdx = 0; timeIdx < ProtocolTimesNum; timeIdx++) {
        this->setProtocolTime(timeIdx, protocolTimeDefault[timeIdx], false);
    }

    for (int slopeIdx = 0; slopeIdx < ProtocolSlopesNum; slopeIdx++) {
        this->setProtocolSlope(slopeIdx, protocolSlopeDefault[slopeIdx], false);
    }

    for (int integerIdx = 0; integerIdx < ProtocolIntegersNum; integerIdx++) {
        this->setProtocolInteger(integerIdx, protocolIntegerDefault[integerIdx], false);
    }

//    this->activateDacIntFilter(false, false);
//    this->activateDacExtFilter(true, false);
}
