#include "messagedispatcher_e1b_el03c_legacyedr3_pcbv02.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02::MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02(string id) :
    MessageDispatcherLegacyEdr3(id) {

    /************************\
     * Communication format *
    \************************/

    ftdiEepromId = FtdiEepromId56;
    rxChannel = 'B';
    txChannel = 'B';

    rxSyncWord = 0xFFFF0000;
    txSyncWord = 0x80;

    packetsPerFrame = 16;

    voltageChannelsNum = 1;
    currentChannelsNum = 1;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+(packetsPerFrame*(int)totalChannelsNum)*(int)FTD_RX_WORD_SIZE;

    infoStructSize = sizeof(InfoStruct_t);
    infoStructPtr = (uint8_t *)&infoStruct;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    txDataBytes = 49;

    /**********************\
     * Available settings *
    \**********************/

    /*! Current ranges */
    independentCurrentRangesFlag = false;
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200pA].min = -200.0;
    currentRangesArray[CurrentRange200pA].max = 200.0;
    currentRangesArray[CurrentRange200pA].step = currentRangesArray[CurrentRange200pA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200pA].prefix = UnitPfxPico;
    currentRangesArray[CurrentRange200pA].unit = "A";
    currentRangesArray[CurrentRange2nA].min = -2.0;
    currentRangesArray[CurrentRange2nA].max = 2.0;
    currentRangesArray[CurrentRange2nA].step = currentRangesArray[CurrentRange2nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange2nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange2nA].unit = "A";
    currentRangesArray[CurrentRange20nA].min = -20.0;
    currentRangesArray[CurrentRange20nA].max = 20.0;
    currentRangesArray[CurrentRange20nA].step = currentRangesArray[CurrentRange20nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange20nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange20nA].unit = "A";
    currentRangesArray[CurrentRange200nA].min = -200.0;
    currentRangesArray[CurrentRange200nA].max = 200.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange200nA].unit = "A";
    defaultCurrentRangesIdx.resize(currentChannelsNum);
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        defaultCurrentRangesIdx[channelIdx] = CurrentRange200pA;
    }

    /*! Voltage ranges */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange2000mV].step = 1;
    voltageRangesArray[VoltageRange2000mV].min = -voltageRangesArray[VoltageRange2000mV].step*2047.0;
    voltageRangesArray[VoltageRange2000mV].max = voltageRangesArray[VoltageRange2000mV].step*2047.0;
    voltageRangesArray[VoltageRange2000mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange2000mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange2000mV;
    rawVoltageZero = 2048;

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

    /*! Overampling ratios */
    oversamplingImplemented = false;
    oversamplingRatiosNum = OversamplingRatiosNum;
    oversamplingRatiosArray.resize(oversamplingRatiosNum);
    oversamplingRatiosArray[OversamplingRatioX1] = 1;

    /*! Voltage filters */
    dacIntFilterAvailable = false;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);

    dacExtFilterAvailable = true;
    voltageReferenceLpfOptionsNum = VoltageReferenceLpfsNum;
    voltageReferenceLpfOptions.resize(voltageReferenceLpfOptionsNum);
    voltageReferenceLpfOptions[VoltageReferenceLpf3Hz].value = 3.0;
    voltageReferenceLpfOptions[VoltageReferenceLpf3Hz].prefix = UnitPfxNone;
    voltageReferenceLpfOptions[VoltageReferenceLpf3Hz].unit = "Hz";
    voltageReferenceLpfOptions[VoltageReferenceLpf180kHz].value = 180.0;
    voltageReferenceLpfOptions[VoltageReferenceLpf180kHz].prefix = UnitPfxKilo;
    voltageReferenceLpfOptions[VoltageReferenceLpf180kHz].unit = "Hz";

    /*! Front end denoiser */
    ferdImplementedFlag = true;
    maxFerdSize = 512;

    /*! Default values */
    selectedVoltageRangeIdx = defaultVoltageRangeIdx;
    selectedCurrentRangesIdx = defaultCurrentRangesIdx;
    selectedSamplingRateIdx = defaultSamplingRateIdx;

    currentRanges.resize(currentChannelsNum);
    currentResolutions.resize(currentChannelsNum);
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        currentRanges[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]];
        currentResolutions[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]].step;
    }
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step;
    baseSamplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
    samplingRate = baseSamplingRate;
    integrationStep = integrationStepArray[selectedSamplingRateIdx];

    /*************\
     * Protocols *
    \*************/

    /*! Voltage ranges */
    protocolVoltageRangesArray.resize(ProtocolVoltageRangesNum);
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].min = -500.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].max = 500.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].step = 0.0625;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].unit = "V";
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV].min = -1650.0;
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV].max = 1650.0;
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV].step = 0.0625;
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV].unit = "V";
    protocolVoltageRangesArray[ProtocolVoltageRange2000mV].min = -2000.0;
    protocolVoltageRangesArray[ProtocolVoltageRange2000mV].max = 2000.0;
    protocolVoltageRangesArray[ProtocolVoltageRange2000mV].step = 0.0625;
    protocolVoltageRangesArray[ProtocolVoltageRange2000mV].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange2000mV].unit = "V";

    /*! Time ranges */
    protocolTimeRangesArray.resize(ProtocolTimeRangesNum);
    protocolTimeRangesArray[ProtocolTimeRange1_1000ms].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1_1000ms].max = 1000.0;
    protocolTimeRangesArray[ProtocolTimeRange1_1000ms].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1_1000ms].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1_1000ms].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange0to2_20].min = 0.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_20].max = 1.0e6;
    protocolTimeRangesArray[ProtocolTimeRange0to2_20].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_20].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange0to2_20].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_20].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_20].max = 1.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_20].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_20].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_20].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1orMore].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].max = (numeric_limits <double> ::max)();
    protocolTimeRangesArray[ProtocolTimeRange1orMore].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].min = -1.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].max = 1.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].unit = "s";

    /*! Slope ranges */
    protocolSlopeRangesArray.resize(ProtocolSlopeRangesNum);
    protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].min = 1.0;
    protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].max = 1000.0;
    protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].step = 1.0;
    protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].prefix = UnitPfxMilli;
    protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].unit = "V/ms";

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
    defaultProtocol = ProtocolConstant;
    selectedProtocol = defaultProtocol;
    triangularProtocolIdx = ProtocolTriangular;
    sealTestProtocolIdx = ProtocolSquareWave;

    protocolsImages.resize(ProtocolsNum);
    protocolsImages[ProtocolConstant] = "holdingVoltage001";
    protocolsImages[ProtocolTriangular] = "triangularParametric001";
    protocolsImages[ProtocolSquareWave] = "sealTest001";
    protocolsImages[ProtocolConductance] = "conductance001";
    protocolsImages[ProtocolVariableAmplitude] = "stepVariableAmplitude001";
    protocolsImages[ProtocolVariableDuration] = "stepVariableDuration001";
    protocolsImages[ProtocolRamp] = "ramp5_edr3";
    protocolsImages[ProtocolCyclicVoltammetry] = "cyclicvolt3_edr3";

    protocolsAvailableVoltages.resize(ProtocolsNum);
    protocolsAvailableTimes.resize(ProtocolsNum);
    protocolsAvailableSlopes.resize(ProtocolsNum);
    protocolsAvailableFrequencies.resize(ProtocolsNum);
    protocolsAvailableAdimensionals.resize(ProtocolsNum);

    protocolsAvailableVoltages[ProtocolConstant].push_back(ProtocolVHold);

    protocolsAvailableVoltages[ProtocolTriangular].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolTriangular].push_back(ProtocolVPk);
    protocolsAvailableTimes[ProtocolTriangular].push_back(ProtocolTPe);

    protocolsAvailableVoltages[ProtocolSquareWave].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSquareWave].push_back(ProtocolVPulse);
    protocolsAvailableTimes[ProtocolSquareWave].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSquareWave].push_back(ProtocolTPulse);

    protocolsAvailableVoltages[ProtocolConductance].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolConductance].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolConductance].push_back(ProtocolVStep);
    protocolsAvailableTimes[ProtocolConductance].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolConductance].push_back(ProtocolTPulse);
    protocolsAvailableAdimensionals[ProtocolConductance].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolConductance].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolVariableAmplitude].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolVariableAmplitude].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolVariableAmplitude].push_back(ProtocolVStep);
    protocolsAvailableTimes[ProtocolVariableAmplitude].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolVariableAmplitude].push_back(ProtocolTPulse);
    protocolsAvailableAdimensionals[ProtocolVariableAmplitude].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolVariableAmplitude].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolVariableDuration].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolVariableDuration].push_back(ProtocolVPulse);
    protocolsAvailableTimes[ProtocolVariableDuration].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolVariableDuration].push_back(ProtocolTPulse);
    protocolsAvailableTimes[ProtocolVariableDuration].push_back(ProtocolTStep);
    protocolsAvailableAdimensionals[ProtocolVariableDuration].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolVariableDuration].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVMax);
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVMin);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTPulse);
    protocolsAvailableSlopes[ProtocolRamp].push_back(ProtocolSlope);
    protocolsAvailableAdimensionals[ProtocolRamp].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVMax);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVMin);
    protocolsAvailableTimes[ProtocolCyclicVoltammetry].push_back(ProtocolTHold);
    protocolsAvailableSlopes[ProtocolRamp].push_back(ProtocolSlope);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolNR);

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Vhold";
    protocolVoltageNames[ProtocolVPulse] = "Vpulse";
    protocolVoltageNames[ProtocolVStep] = "Vstep";
    protocolVoltageNames[ProtocolVPk] = "Vamp";
    protocolVoltageNames[ProtocolVMax] = "Vmax";
    protocolVoltageNames[ProtocolVMin] = "Vmin";

    protocolVoltageRanges.resize(ProtocolVoltagesNum);
    protocolVoltageRanges[ProtocolVHold].step = 1.0;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange2000mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange2000mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";
    protocolVoltageRanges[ProtocolVPulse].step = 1.0;
    protocolVoltageRanges[ProtocolVPulse].min = voltageRangesArray[VoltageRange2000mV].min;
    protocolVoltageRanges[ProtocolVPulse].max = voltageRangesArray[VoltageRange2000mV].max;
    protocolVoltageRanges[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPulse].unit = "V";
    protocolVoltageRanges[ProtocolVStep].step = 1.0;
    protocolVoltageRanges[ProtocolVStep].min = voltageRangesArray[VoltageRange2000mV].min;
    protocolVoltageRanges[ProtocolVStep].max = voltageRangesArray[VoltageRange2000mV].max;
    protocolVoltageRanges[ProtocolVStep].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVStep].unit = "V";
    protocolVoltageRanges[ProtocolVPk].step = 25.0;
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVMax].step = 1.0;
    protocolVoltageRanges[ProtocolVMax].min = voltageRangesArray[VoltageRange2000mV].min;
    protocolVoltageRanges[ProtocolVMax].max = voltageRangesArray[VoltageRange2000mV].max;
    protocolVoltageRanges[ProtocolVMax].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVMax].unit = "V";
    protocolVoltageRanges[ProtocolVMin].step = 1.0;
    protocolVoltageRanges[ProtocolVMin].min = voltageRangesArray[VoltageRange2000mV].min;
    protocolVoltageRanges[ProtocolVMin].max = voltageRangesArray[VoltageRange2000mV].max;
    protocolVoltageRanges[ProtocolVMin].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVMin].unit = "V";

    protocolVoltageDefault.resize(ProtocolVoltagesNum);
    protocolVoltageDefault[ProtocolVHold].value = 0.0;
    protocolVoltageDefault[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVHold].unit = "V";
    protocolVoltageDefault[ProtocolVPulse].value = 100.0;
    protocolVoltageDefault[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVPulse].unit = "V";
    protocolVoltageDefault[ProtocolVStep].value = 20.0;
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
    selectedProtocolVoltage.resize(ProtocolVoltagesNum);
    for (unsigned int idx = 0; idx < ProtocolVoltagesNum; idx++) {
        selectedProtocolVoltage[idx] = protocolVoltageDefault[idx];
    }

    /*! Protocol times */
    protocolTimesNum = ProtocolTimesNum;
    protocolTimeNames.resize(ProtocolTimesNum);
    protocolTimeNames[ProtocolTHold] = "Thold";
    protocolTimeNames[ProtocolTPulse] = "Tpulse";
    protocolTimeNames[ProtocolTStep] = "Tstep";
    protocolTimeNames[ProtocolTPe] = "TPeriod";

    protocolTimeRanges.resize(ProtocolTimesNum);
    protocolTimeRanges[ProtocolTHold].step = 1.0;
    protocolTimeRanges[ProtocolTHold].min = 0.0;
    protocolTimeRanges[ProtocolTHold].max = UINT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTHold].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTHold].unit = "s";
    protocolTimeRanges[ProtocolTPulse].step = 1.0;
    protocolTimeRanges[ProtocolTPulse].min = 0.0;
    protocolTimeRanges[ProtocolTPulse].max = UINT28_MAX*protocolTimeRanges[ProtocolTPulse].step;
    protocolTimeRanges[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTPulse].unit = "s";
    protocolTimeRanges[ProtocolTStep].step = 1.0;
    protocolTimeRanges[ProtocolTStep].min = INT28_MIN*protocolTimeRanges[ProtocolTStep].step;
    protocolTimeRanges[ProtocolTStep].max = INT28_MAX*protocolTimeRanges[ProtocolTStep].step;
    protocolTimeRanges[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTStep].unit = "s";
    protocolTimeRanges[ProtocolTPe].step = 1.0;
    protocolTimeRanges[ProtocolTPe].min = 0.0;
    protocolTimeRanges[ProtocolTPe].max = UINT10_MAX*protocolTimeRanges[ProtocolTPe].step;
    protocolTimeRanges[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTPe].unit = "s";

    protocolTimeDefault.resize(ProtocolTimesNum);
    protocolTimeDefault[ProtocolTHold].value = 100.0;
    protocolTimeDefault[ProtocolTHold].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTHold].unit = "s";
    protocolTimeDefault[ProtocolTPulse].value = 100.0;
    protocolTimeDefault[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPulse].unit = "s";
    protocolTimeDefault[ProtocolTStep].value = 20.0;
    protocolTimeDefault[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTStep].unit = "s";
    protocolTimeDefault[ProtocolTPe].value = 100.0;
    protocolTimeDefault[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPe].unit = "s";
    selectedProtocolTime.resize(ProtocolTimesNum);
    for (unsigned int idx = 0; idx < ProtocolTimesNum; idx++) {
        selectedProtocolTime[idx] = protocolTimeDefault[idx];
    }

    /*! Protocol slope */
    protocolSlopesNum = ProtocolSlopesNum;
    protocolSlopeNames.resize(ProtocolSlopesNum);
    protocolSlopeNames[ProtocolSlope] = "Slope";

    protocolSlopeRanges.resize(ProtocolSlopesNum);
    protocolSlopeRanges[ProtocolSlope].step = protocolSlopeRangesArray[ProtocolSlope].step;
    protocolSlopeRanges[ProtocolSlope].min = protocolSlopeRangesArray[ProtocolSlope].min;
    protocolSlopeRanges[ProtocolSlope].max = protocolSlopeRangesArray[ProtocolSlope].max;
    protocolSlopeRanges[ProtocolSlope].prefix = UnitPfxMilli;
    protocolSlopeRanges[ProtocolSlope].unit = "V/ms";

    protocolSlopeDefault.resize(ProtocolSlopesNum);
    protocolSlopeDefault[ProtocolSlope].value = 1.0;
    protocolSlopeDefault[ProtocolSlope].prefix = UnitPfxMilli;
    protocolSlopeDefault[ProtocolSlope].unit = "V/ms";
    selectedProtocolSlope.resize(ProtocolSlopesNum);
    for (unsigned int idx = 0; idx < ProtocolSlopesNum; idx++) {
        selectedProtocolSlope[idx] = protocolSlopeDefault[idx];
    }

    /*! Protocol adimensionals */
    protocolAdimensionalsNum = ProtocolAdimensionalsNum;
    protocolAdimensionalNames.resize(ProtocolAdimensionalsNum);
    protocolAdimensionalNames[ProtocolN] = "N";
    protocolAdimensionalNames[ProtocolNR] = "NR";

    protocolAdimensionalRanges.resize(ProtocolAdimensionalsNum);
    protocolAdimensionalRanges[ProtocolN].step = 1.0;
    protocolAdimensionalRanges[ProtocolN].min = 0.0;
    protocolAdimensionalRanges[ProtocolN].max = UINT10_MAX*protocolAdimensionalRanges[ProtocolN].step;
    protocolAdimensionalRanges[ProtocolN].prefix = UnitPfxNone;
    protocolAdimensionalRanges[ProtocolN].unit = "";
    protocolAdimensionalRanges[ProtocolNR].step = 1.0;
    protocolAdimensionalRanges[ProtocolNR].min = 0.0;
    protocolAdimensionalRanges[ProtocolNR].max = UINT10_MAX*protocolAdimensionalRanges[ProtocolNR].step;
    protocolAdimensionalRanges[ProtocolNR].prefix = UnitPfxNone;
    protocolAdimensionalRanges[ProtocolNR].unit = "";

    protocolAdimensionalDefault.resize(ProtocolAdimensionalsNum);
    protocolAdimensionalDefault[ProtocolN].value = 5.0;
    protocolAdimensionalDefault[ProtocolN].prefix = UnitPfxNone;
    protocolAdimensionalDefault[ProtocolN].unit = "";
    protocolAdimensionalDefault[ProtocolNR].value = 0.0;
    protocolAdimensionalDefault[ProtocolNR].prefix = UnitPfxNone;
    protocolAdimensionalDefault[ProtocolNR].unit = "";
    selectedProtocolAdimensional.resize(ProtocolAdimensionalsNum);
    for (unsigned int idx = 0; idx < ProtocolAdimensionalsNum; idx++) {
        selectedProtocolAdimensional[idx] = protocolAdimensionalDefault[idx];
    }

    voltageOffsetControlImplemented = false;
    selectedVoltageOffset.clear();

    insertionPulseImplemented = true;
    insertionPulseVoltageRange.step = 1.0;
    insertionPulseVoltageRange.min = -500.0;
    insertionPulseVoltageRange.max = 500.0;
    insertionPulseVoltageRange.prefix = UnitPfxMilli;
    insertionPulseVoltageRange.unit = "V";
    insertionPulseDurationRange.step = 1.0;
    insertionPulseDurationRange.min = 1.0;
    insertionPulseDurationRange.max = 15000.0;
    insertionPulseDurationRange.prefix = UnitPfxMilli;
    insertionPulseDurationRange.unit = "s";

    /**************\
     * EDH format *
    \**************/

    edhFormat =
        "EDH Version: 2.0\n"
        "\n"
        "Elements e1b\n"
        "Channels: 1\n"
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
        "Active channels: %activeChannels%\n"; // 1

    /****************************\
     * Device specific controls *
    \****************************/

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Device reset */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    deviceResetCoder = new BoolArrayCoder(boolConfig);

    /*! Select stimulus channel */
    selectStimulusChannelFlag = false;
    singleChannelSSCFlag = false;

    /*! Digital offset compensations */
    digitalOffsetCompensationFlag = true;
    singleChannelDOCFlag = true;
    selectableDOCAutostopFlag = false;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationCoder = new BoolArrayCoder(boolConfig);

    digitalOffsetCompensationStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        digitalOffsetCompensationStates[currentIdx] = false;
    }

    /*! Zap */
    zappableDeviceFlag = true;
    singleChannelZapFlag = true;

    boolConfig.initialByte = 4;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    zapCoder = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        zapStates[currentIdx] = false;
    }

    /*! Channel off */
    channelOnFlag = false;
    singleChannelOnFlag = false;

    /*! Current range */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 3;
    currentRangeCoders.resize(1);
    currentRangeCoders[0] = new BoolRandomArrayCoder(boolConfig);
    static_cast <BoolRandomArrayCoder *> (currentRangeCoders[0])->addMapItem(0); /*!< 200pA    -> 0b000 */
    static_cast <BoolRandomArrayCoder *> (currentRangeCoders[0])->addMapItem(2); /*!< 2nA      -> 0b010 */
    static_cast <BoolRandomArrayCoder *> (currentRangeCoders[0])->addMapItem(3); /*!< 20nA     -> 0b011 */
    static_cast <BoolRandomArrayCoder *> (currentRangeCoders[0])->addMapItem(7); /*!< 200nA    -> 0b111 */

    /*! Voltage range */
    boolConfig.initialByte = 0;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    voltageRangeCoder = new BoolArrayCoder(boolConfig);

    /*! Sampling rate */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 4;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(0); /*!< 1.25kHz  -> 0b0000 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(1); /*!< 5kHz     -> 0b0001 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(2); /*!< 10kHz    -> 0b0010 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(3); /*!< 20kHz    -> 0b0011 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(8); /*!< 50kHz    -> 0b1000 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(9); /*!< 100kHz   -> 0b1001 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(10); /*!< 200kHz  -> 0b1010 */

    /*! Protocol selection */
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 4;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltages */
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 7;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 12;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    protocolVoltageCoders[ProtocolVHold] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 11;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 15;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    protocolVoltageCoders[ProtocolVStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 35;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageCoders[ProtocolVPk] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 39;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMax].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMax].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMax].step;
    protocolVoltageCoders[ProtocolVMax] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 41;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMin].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMin].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMin].step;
    protocolVoltageCoders[ProtocolVMin] = new DoubleSignAbsCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 17;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    protocolTimeCoders[ProtocolTHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 21;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    protocolTimeCoders[ProtocolTPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 27;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTStep].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTStep].max;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTStep].step;
    protocolTimeCoders[ProtocolTStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 35;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    protocolTimeCoders[ProtocolTPe] = new DoubleTwosCompCoder(doubleConfig);

    /*! Protocol slope */
    protocolSlopeCoders.resize(ProtocolSlopesNum);
    doubleConfig.initialByte = 37;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolSlopeRanges[ProtocolSlope].step;
    doubleConfig.minValue = protocolSlopeRanges[ProtocolSlope].min;
    doubleConfig.maxValue = protocolSlopeRanges[ProtocolSlope].max;
    protocolSlopeCoders[ProtocolSlope] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);
    doubleConfig.initialByte = 31;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolN].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolN].max;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolN].step;
    protocolAdimensionalCoders[ProtocolN] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 33;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolNR].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolNR].max;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolNR].step;
    protocolAdimensionalCoders[ProtocolNR] = new DoubleTwosCompCoder(doubleConfig);

    /*! Internal DAC filter */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolNegatedArrayCoder(boolConfig);

    /*! External DAC filter */
    boolConfig.initialByte = 6;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    dacExtFilterCoder = new BoolArrayCoder(boolConfig);

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);

    /*! Insertion pulse */
    doubleConfig.initialByte = 16;
    doubleConfig.initialBit = 4;
    doubleConfig.bitsNum = 12;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    doubleConfig.resolution = insertionPulseVoltageRange.step;
    insertionPulseVoltageCoder = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 30;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = insertionPulseDurationRange.min;
    doubleConfig.maxValue = insertionPulseDurationRange.max;
    doubleConfig.resolution = insertionPulseDurationRange.step;
    insertionPulseDurationCoder = new DoubleTwosCompCoder(doubleConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    insertionPulseApplyCoder = new BoolArrayCoder(boolConfig);

    /*! Device specific controls */

    customFlagsNum = 1;
    customFlagsNames.resize(customFlagsNum);
    customFlagsNames[0] = "Trigger selection";
    customFlagsDefault.resize(customFlagsNum);
    customFlagsDefault[0] = false;

    customFlagsCoders.resize(customFlagsNum);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    customFlagsCoders[0] = new BoolArrayCoder(boolConfig);

    /*******************\
     * Default status  *
    \*******************/

    txStatus.resize(txDataBytes);

    int txStatusIdx = 0;
    txStatus[txStatusIdx++] = txSyncWord; // HDR
    txStatus[txStatusIdx++] = 0x20; // CFG0
    txStatus[txStatusIdx++] = 0x03; // CFG1
    txStatus[txStatusIdx++] = 0x00; // CFG2
    txStatus[txStatusIdx++] = 0x00; // CFG3
    txStatus[txStatusIdx++] = 0x00; // COMP0
    txStatus[txStatusIdx++] = 0x00; // COMP1
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Vofs
    txStatus[txStatusIdx++] = 0x00; // Vofs
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // free
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VInsPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VStep
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // THold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TInsPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TStep
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // N
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // NR
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Triangular
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Slope
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Vmax
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Vmin
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02::~MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02() {

}

void MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->digitalOffsetCompensation(currentChannelsNum, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-2000,2000]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold+Vamp\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold-Vamp\nmust be within [-2000,2000]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1_1000ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_20].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 1e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                     selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
                                                                                     selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold-Vpulse-Vstep(N-1)\nmust be within [-2000,2000]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_20].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 1e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableAmplitude:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                     selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-2000,2000]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_20].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 1e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableDuration:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-2000,2000]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_20].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 1e6]ms";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRangeSigned2_20].includes(selectedProtocolTime[ProtocolTStep]))) {
            validFlag = false;
            message = "Tstep\nmust be within [-1e6, 1e6]ms";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1orMore].includes(selectedProtocolTime[ProtocolTPulse]+
                                                                                selectedProtocolTime[ProtocolTStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Tpulse+Tstep(N-1)\nmust be at least 1ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolRamp:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vmax\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vhold+Vmax\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vmin\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vhold+Vmin\nmust be within [-2000,2000]mV";

        } else if (selectedProtocolVoltage[ProtocolVMin] >= selectedProtocolVoltage[ProtocolVMax]) {
            validFlag = false;
            message = "Vax-Vmin\nmust be greater than 0mV";

        } else if (!(protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].includes(selectedProtocolSlope[ProtocolSlope]))) {
            validFlag = false;
            message = "Slope\nmust be within [1, 1000]mV/ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vmax\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vhold+Vmax\nmust be within [-2000,2000]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vmin\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange2000mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vhold+Vmin\nmust be within [-2000,2000]mV";

        } else if (selectedProtocolVoltage[ProtocolVMin] >= selectedProtocolVoltage[ProtocolVMax]) {
            validFlag = false;
            message = "Vax-Vmin\nmust be greater than 0mV";

        } else if (!(protocolSlopeRangesArray[ProtocolSlopeRange1_1000mVms].includes(selectedProtocolSlope[ProtocolSlope]))) {
            validFlag = false;
            message = "Slope\nmust be within [1, 1000]mV/ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;
    }
    return validFlag;
}

void MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02::setFerdParameters() {
    unsigned int rangeCoeff;
    /*! At the moment the front end reset denoiser is only available for devices that apply the same current range on all channels */
    if (selectedCurrentRangesIdx[0] < CurrentRange200nA) {
        rangeCoeff = 4;

    } else {
        rangeCoeff = 1;
    }

    if (selectedSamplingRateIdx < SamplingRate20kHz) {
        /*! sampling rate too low for reset */
        ferdL = 1;
        ferdInhibition = true;

    } else if (selectedSamplingRateIdx == SamplingRate20kHz) {
        if (rangeCoeff == 1) {
            /*! sampling rate too low for reset (in the highest range the reset is 4 times faster) */
            ferdL = 1;
            ferdInhibition = true;

        } else {
            ferdL = oversamplingRatio*rangeCoeff/2;
            ferdInhibition = false;
        }

    } else {
        ferdL = oversamplingRatio*((unsigned int)round(baseSamplingRate.getNoPrefixValue()/50.0e3))*rangeCoeff*32; /*! It should be osrSR/baseSR*baseSR/50kHz */
        ferdInhibition = false;
    }
    ferdK = 2.0/(2.0+1024.0/(double)rangeCoeff);

    MessageDispatcher::setFerdParameters();
}
