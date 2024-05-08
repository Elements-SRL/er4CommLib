#include "messagedispatcher_e4n.h"

using namespace std;
using namespace er4CommLib;

MessageDispatcher_e4n_V01::MessageDispatcher_e4n_V01(string di) :
    MessageDispatcher(di) {

    /************************\
     * Communication format *
    \************************/

    ftdiEepromId = FtdiEepromId56;
    rxChannel = 'B';
    txChannel = 'B';

    rxSyncWord = 0x7FFF8000;
    txSyncWord = 0x80;

    packetsPerFrame = 16;

    voltageChannelsNum = 1;
    currentChannelsNum = 4;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+FTD_RX_INFO_WORD_SIZE+(packetsPerFrame*(int)totalChannelsNum)*(int)FTD_RX_WORD_SIZE;

    infoStructSize = sizeof(InfoStruct_t);
    infoStructPtr = (uint8_t *)&infoStruct;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    txDataBytes = 51;

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
    voltageRangesArray[VoltageRange500mV].min = -511.0;
    voltageRangesArray[VoltageRange500mV].max = 511.0;
    voltageRangesArray[VoltageRange500mV].step = 0.0625;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange500mV;

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

    voltageOffsetCompensationGain.value = 100.0/1023.0;
    voltageOffsetCompensationGain.prefix = UnitPfxMilli;
    voltageOffsetCompensationGain.unit = "V";

    /*! Overampling ratios */
    oversamplingImplemented = false;
    oversamplingRatiosNum = OversamplingRatiosNum;
    oversamplingRatiosArray.resize(oversamplingRatiosNum);
    oversamplingRatiosArray[OversamplingRatioX1] = 1;

    /*! Voltage filters */
    dacIntFilterAvailable = true;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].value = 100.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].prefix = UnitPfxNone;
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].unit = "Hz";
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].value = 10.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].prefix = UnitPfxKilo;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].unit = "Hz";

    dacExtFilterAvailable = false;
    voltageReferenceLpfOptionsNum = VoltageReferenceLpfsNum;
    voltageReferenceLpfOptions.resize(voltageReferenceLpfOptionsNum);

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

    /*! Time ranges */
    protocolTimeRangesArray.resize(ProtocolTimeRangesNum);
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].max = 1000.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].min = 0.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1orMore].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].max = (numeric_limits <double> ::max)();
    protocolTimeRangesArray[ProtocolTimeRange1orMore].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].min = -100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].max = 100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].max = 30.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].unit = "s";

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
    protocolsImages[ProtocolRamp] = "ramp002";
    protocolsImages[ProtocolCyclicVoltammetry] = "cyclicVoltammetry002";

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
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVInit);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTPulse);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTRamp);
    protocolsAvailableAdimensionals[ProtocolRamp].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVInit);
    protocolsAvailableTimes[ProtocolCyclicVoltammetry].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolCyclicVoltammetry].push_back(ProtocolTRamp);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolNR);

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Vhold";
    protocolVoltageNames[ProtocolVPulse] = "Vpulse";
    protocolVoltageNames[ProtocolVStep] = "Vstep";
    protocolVoltageNames[ProtocolVPk] = "Vamp";
    protocolVoltageNames[ProtocolVFinal] = "Vfinal";
    protocolVoltageNames[ProtocolVInit] = "Vinit";

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
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVFinal].step = 1.0;
    protocolVoltageRanges[ProtocolVFinal].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVFinal].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVFinal].unit = "V";
    protocolVoltageRanges[ProtocolVInit].step = 1.0;
    protocolVoltageRanges[ProtocolVInit].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVInit].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVInit].unit = "V";

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
    protocolVoltageDefault[ProtocolVFinal].value = 100.0;
    protocolVoltageDefault[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVFinal].unit = "V";
    protocolVoltageDefault[ProtocolVInit].value = -100.0;
    protocolVoltageDefault[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVInit].unit = "V";
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
    protocolTimeNames[ProtocolTRamp] = "Tramp";
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
    protocolTimeRanges[ProtocolTRamp].step = 1.0;
    protocolTimeRanges[ProtocolTRamp].min = 0.0;
    protocolTimeRanges[ProtocolTRamp].max = UINT28_MAX*protocolTimeRanges[ProtocolTRamp].step;
    protocolTimeRanges[ProtocolTRamp].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTRamp].unit = "s";
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
    protocolTimeDefault[ProtocolTRamp].value = 1000.0;
    protocolTimeDefault[ProtocolTRamp].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTRamp].unit = "s";
    protocolTimeDefault[ProtocolTPe].value = 100.0;
    protocolTimeDefault[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPe].unit = "s";
    selectedProtocolTime.resize(ProtocolTimesNum);
    for (unsigned int idx = 0; idx < ProtocolTimesNum; idx++) {
        selectedProtocolTime[idx] = protocolTimeDefault[idx];
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

    voltageOffsetControlImplemented = true;
    selectedVoltageOffset.resize(currentChannelsNum);
    voltageOffsetRange.step = 1.0;
    voltageOffsetRange.min = -500.0;
    voltageOffsetRange.max = 500.0;
    voltageOffsetRange.prefix = UnitPfxMilli;
    voltageOffsetRange.unit = "V";
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        selectedVoltageOffset[channelIdx].value = 0.0;
        selectedVoltageOffset[channelIdx].prefix = voltageOffsetRange.prefix;
        selectedVoltageOffset[channelIdx].unit = voltageOffsetRange.unit;
    }

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
            "Elements e4\n"
            "Channels: 4\n"
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
            "Active channels: %activeChannels%\n"; // 2 3 4

    /****************************\
     * Device specific controls *
    \****************************/

    nanionTemperatureControllerFlag = true;

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
    selectStimulusChannelFlag = true;
    singleChannelSSCFlag = true;

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    selectStimulusChannelCoder = new BoolArrayCoder(boolConfig);

    selectStimulusChannelStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        selectStimulusChannelStates[currentIdx] = false;
    }

    /*! Digital offset compensations */
    digitalOffsetCompensationFlag = true;
    singleChannelDOCFlag = true;
    selectableDOCAutostopFlag = false;
    digitalOffsetCompensationResetFlag = true;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 4;
    digitalOffsetCompensationCoder = new BoolArrayCoder(boolConfig);

    digitalOffsetCompensationStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        digitalOffsetCompensationStates[currentIdx] = false;
    }

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationResetCoder = new BoolArrayCoder(boolConfig);

    /*! Zap */
    zappableDeviceFlag = true;
    singleChannelZapFlag = true;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    zapCoder = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        zapStates[currentIdx] = false;
    }

    /*! Channel off */
    channelOnFlag = true;
    singleChannelOnFlag = true;

    boolConfig.initialByte = 5;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    channelOnCoder = new BoolNegatedArrayCoder(boolConfig);

    channelOnStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        channelOnStates[currentIdx] = false;
    }

    /*! Current range */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 3;
    currentRangeCoders.resize(1);
    currentRangeCoders[0] = new BoolRandomArrayCoder(boolConfig);
    currentRangeCoders[0]->addMapItem(0); /*!< 200pA    -> 0b000 */
    currentRangeCoders[0]->addMapItem(2); /*!< 2nA      -> 0b010 */
    currentRangeCoders[0]->addMapItem(3); /*!< 20nA     -> 0b011 */
    currentRangeCoders[0]->addMapItem(7); /*!< 200nA    -> 0b111 */

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
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    protocolVoltageCoders[ProtocolVHold] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 17;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 21;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    protocolVoltageCoders[ProtocolVStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 41;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    protocolVoltageCoders[ProtocolVPk] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 47;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVFinal].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVFinal].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVFinal].max;
    protocolVoltageCoders[ProtocolVFinal] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 49;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVInit].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVInit].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVInit].max;
    protocolVoltageCoders[ProtocolVInit] = new DoubleSignAbsCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 23;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    protocolTimeCoders[ProtocolTHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 27;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    protocolTimeCoders[ProtocolTPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 33;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTStep].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTStep].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTStep].max;
    protocolTimeCoders[ProtocolTStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 43;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTRamp].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTRamp].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTRamp].max;
    protocolTimeCoders[ProtocolTRamp] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 41;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    protocolTimeCoders[ProtocolTPe] = new DoubleTwosCompCoder(doubleConfig);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);
    doubleConfig.initialByte = 37;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolN].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolN].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolN].max;
    protocolAdimensionalCoders[ProtocolN] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 39;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolNR].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolNR].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolNR].max;
    protocolAdimensionalCoders[ProtocolNR] = new DoubleTwosCompCoder(doubleConfig);

    /*! Internal DAC filter */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolRandomArrayCoder(boolConfig);
    dacIntFilterCoder->addMapItem(1); /*!< 1kHz  -> 0b1 */
    dacIntFilterCoder->addMapItem(0); /*!< 10kHz  -> 0b0 */

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        doubleConfig.initialByte = 9+2*channelIdx;
        voltageOffsetCoders[channelIdx] = new DoubleSignAbsCoder(doubleConfig);
    }

    /*! Insertion pulse */
    doubleConfig.initialByte = 19;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = insertionPulseVoltageRange.step;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    insertionPulseVoltageCoder = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 31;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.resolution = insertionPulseDurationRange.step;
    doubleConfig.minValue = insertionPulseDurationRange.min;
    doubleConfig.maxValue = insertionPulseDurationRange.max;
    insertionPulseDurationCoder = new DoubleTwosCompCoder(doubleConfig);
    boolConfig.initialByte = 5;
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
    boolConfig.initialByte = 5;
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
    txStatus[txStatusIdx++] = 0x00; // CFG4
    txStatus[txStatusIdx++] = 0x0F; // CFG5
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs1
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs2
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs3
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs4
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
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // N
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // NR
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Triangular
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TRamp
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Vfinal
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VInit
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_e4n_V01::~MessageDispatcher_e4n_V01() {

}

ErrorCodes_t MessageDispatcher_e4n_V01::getTemperatureControllerRange(int &minTemperature, int &maxTemperature) {
    minTemperature = minControllerTemperature;
    maxTemperature = maxControllerTemperature;
    return Success;
}

void MessageDispatcher_e4n_V01::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->selectStimulusChannel(currentChannelsNum, true);
    this->digitalOffsetCompensation(currentChannelsNum, false);
    this->switchChannelOn(currentChannelsNum, true, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_e4n_V01::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold+Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold-Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange2_10ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableAmplitude:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableDuration:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].includes(selectedProtocolTime[ProtocolTStep]))) {
            validFlag = false;
            message = "Tstep\nmust be within [-100e6, 100e6]ms";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

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

void MessageDispatcher_e4n_V01::setFerdParameters() {
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

ErrorCodes_t MessageDispatcher_e4n_V01::updateVoltageOffsetCompensations(vector <Measurement_t> &offsets) {
    for (int idx = 0; idx < currentChannelsNum; idx++) {
        offsets[idx] = voltageOffsetCompensationGain*(double)(infoStruct.offset[idx]);
    }
    return Success;
}

MessageDispatcher_e4n_sine_V01::MessageDispatcher_e4n_sine_V01(string di) :
    MessageDispatcher_e4n_V01(di) {

    /************************\
     * Communication format *
    \************************/

    txDataBytes = 53;

    /*************\
     * Protocols *
    \*************/

    /*! Frequency ranges */
    protocolFrequencyRangesArray.resize(ProtocolFrequencyRangesNum);
    protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].min = 0.0;
    protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].max = 35.0;
    protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].step = 1.0;
    protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].prefix = UnitPfxNone;
    protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].unit = "Hz";

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
    protocolsNames[ProtocolSinusoid] = "Sinusoid";
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
    protocolsImages[ProtocolRamp] = "ramp002";
    protocolsImages[ProtocolCyclicVoltammetry] = "cyclicVoltammetry002";
    protocolsImages[ProtocolSinusoid] = "sineWave001";

    protocolsAvailableVoltages.clear();
    protocolsAvailableTimes.clear();
    protocolsAvailableSlopes.clear();
    protocolsAvailableFrequencies.clear();
    protocolsAvailableAdimensionals.clear();
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
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolRamp].push_back(ProtocolVInit);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTPulse);
    protocolsAvailableTimes[ProtocolRamp].push_back(ProtocolTRamp);
    protocolsAvailableAdimensionals[ProtocolRamp].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolCyclicVoltammetry].push_back(ProtocolVInit);
    protocolsAvailableTimes[ProtocolCyclicVoltammetry].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolCyclicVoltammetry].push_back(ProtocolTRamp);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolCyclicVoltammetry].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolSinusoid].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinusoid].push_back(ProtocolVPulse);
    protocolsAvailableFrequencies[ProtocolSinusoid].push_back(ProtocolFrequency);

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Vhold";
    protocolVoltageNames[ProtocolVPulse] = "Vpulse";
    protocolVoltageNames[ProtocolVStep] = "Vstep";
    protocolVoltageNames[ProtocolVPk] = "Vamp";
    protocolVoltageNames[ProtocolVFinal] = "Vfinal";
    protocolVoltageNames[ProtocolVInit] = "Vinit";

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
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVFinal].step = 1.0;
    protocolVoltageRanges[ProtocolVFinal].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVFinal].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVFinal].unit = "V";
    protocolVoltageRanges[ProtocolVInit].step = 1.0;
    protocolVoltageRanges[ProtocolVInit].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVInit].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVInit].unit = "V";

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
    protocolVoltageDefault[ProtocolVFinal].value = 100.0;
    protocolVoltageDefault[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVFinal].unit = "V";
    protocolVoltageDefault[ProtocolVInit].value = -100.0;
    protocolVoltageDefault[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVInit].unit = "V";
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
    protocolTimeNames[ProtocolTRamp] = "Tramp";
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
    protocolTimeRanges[ProtocolTRamp].step = 1.0;
    protocolTimeRanges[ProtocolTRamp].min = 0.0;
    protocolTimeRanges[ProtocolTRamp].max = UINT28_MAX*protocolTimeRanges[ProtocolTRamp].step;
    protocolTimeRanges[ProtocolTRamp].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTRamp].unit = "s";
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
    protocolTimeDefault[ProtocolTRamp].value = 1000.0;
    protocolTimeDefault[ProtocolTRamp].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTRamp].unit = "s";
    protocolTimeDefault[ProtocolTPe].value = 100.0;
    protocolTimeDefault[ProtocolTPe].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTPe].unit = "s";
    selectedProtocolTime.resize(ProtocolTimesNum);
    for (unsigned int idx = 0; idx < ProtocolTimesNum; idx++) {
        selectedProtocolTime[idx] = protocolTimeDefault[idx];
    }

    /*! Protocol frequencies */
    protocolFrequenciesNum = ProtocolFrequenciesNum;
    protocolFrequencyNames.resize(ProtocolFrequenciesNum);
    protocolFrequencyNames[ProtocolFrequency] = "Frequency";

    protocolFrequencyRanges.resize(ProtocolFrequenciesNum);
    protocolFrequencyRanges[ProtocolFrequency].step = 10.0e6/256.0/16777216.0;
    protocolFrequencyRanges[ProtocolFrequency].min = 0.0;
    protocolFrequencyRanges[ProtocolFrequency].max = UINT14_MAX*protocolFrequencyRanges[ProtocolFrequency].step;
    protocolFrequencyRanges[ProtocolFrequency].prefix = UnitPfxNone;
    protocolFrequencyRanges[ProtocolFrequency].unit = "Hz";

    protocolFrequencyDefault.resize(ProtocolFrequenciesNum);
    protocolFrequencyDefault[ProtocolFrequency].value = 10.0;
    protocolFrequencyDefault[ProtocolFrequency].prefix = UnitPfxNone;
    protocolFrequencyDefault[ProtocolFrequency].unit = "Hz";
    selectedProtocolFrequency.resize(ProtocolFrequenciesNum);
    for (unsigned int idx = 0; idx < ProtocolFrequenciesNum; idx++) {
        selectedProtocolFrequency[idx] = protocolFrequencyDefault[idx];
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

    /**************\
     * EDH format *
    \**************/

    edhFormat =
            "EDH Version: 2.0\n"
            "\n"
            "Elements e4\n"
            "Channels: 4\n"
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
            "Active channels: %activeChannels%\n"; // 2 3 4

    /****************************\
     * Device specific controls *
    \****************************/

    nanionTemperatureControllerFlag = true;

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Protocol frequencies */
    protocolFrequencyCoders.resize(ProtocolFrequenciesNum);
    doubleConfig.initialByte = 51;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolFrequencyRanges[ProtocolFrequency].min;
    doubleConfig.maxValue = protocolFrequencyRanges[ProtocolFrequency].max;
    doubleConfig.resolution = protocolFrequencyRanges[ProtocolFrequency].step;
    protocolFrequencyCoders[ProtocolFrequency] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*! Device specific controls */

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
    txStatus[txStatusIdx++] = 0x00; // CFG4
    txStatus[txStatusIdx++] = 0x0F; // CFG5
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs1
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs2
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs3
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs4
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
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // N
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // NR
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Triangular
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TRamp
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Vfinal
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VInit
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Frequency
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_e4n_sine_V01::~MessageDispatcher_e4n_sine_V01() {

}

bool MessageDispatcher_e4n_sine_V01::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold+Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold-Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange2_10ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableAmplitude:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableDuration:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].includes(selectedProtocolTime[ProtocolTStep]))) {
            validFlag = false;
            message = "Tstep\nmust be within [-100e6, 100e6]ms";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSinusoid:
        if (!(protocolVoltageRangesArray[selectedVoltageRangeIdx].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[selectedVoltageRangeIdx].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].includes(selectedProtocolFrequency[ProtocolFrequency]))) {
            validFlag = false;
            message = "Frequency\nmust be within [0,35]Hz";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;
    }
    return validFlag;
}

MessageDispatcher_e4n_El03c_LegacyEdr3_V04::MessageDispatcher_e4n_El03c_LegacyEdr3_V04(string id) :
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
    currentChannelsNum = 4;
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
    voltageRangesArray[VoltageRange500mV].min = -511.0;
    voltageRangesArray[VoltageRange500mV].max = 511.0;
    voltageRangesArray[VoltageRange500mV].step = 1.0;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange500mV;
    rawVoltageZero = 512;

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
    dacIntFilterAvailable = true;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].value = 100.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].prefix = UnitPfxNone;
    voltageStimulusLpfOptions[VoltageStimulusLpf100Hz].unit = "Hz";
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].value = 10.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].prefix = UnitPfxKilo;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].unit = "Hz";

    dacExtFilterAvailable = false;
    voltageReferenceLpfOptionsNum = VoltageReferenceLpfsNum;
    voltageReferenceLpfOptions.resize(voltageReferenceLpfOptionsNum);

    /*! Front end denoiser */
    ferdImplementedFlag = false;
    maxFerdSize = 1;

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
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].min = -511.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].max = 511.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].step = 1.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].unit = "V";

    /*! Time ranges */
    protocolTimeRangesArray.resize(ProtocolTimeRangesNum);
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].max = 1000.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].min = 0.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1orMore].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].max = (numeric_limits <double> ::max)();
    protocolTimeRangesArray[ProtocolTimeRange1orMore].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].min = -100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].max = 100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].max = 30.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].unit = "s";

    /*! Slope ranges */
    protocolSlopeRangesArray.resize(ProtocolSlopeRangesNum);
    protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].min = 0.0;
    protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].max = 1000.0;
    protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].step = 1.0;
    protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].prefix = UnitPfxMilli;
    protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].unit = "V/ms";

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
    protocolsImages[ProtocolConductance] = "conductance002";
    protocolsImages[ProtocolVariableAmplitude] = "stepVariableAmplitude001";
    protocolsImages[ProtocolVariableDuration] = "stepVariableDuration001";
    protocolsImages[ProtocolRamp] = "ramp003";
    protocolsImages[ProtocolCyclicVoltammetry] = "cyclicVoltammetry003";

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
    protocolsAvailableSlopes[ProtocolCyclicVoltammetry].push_back(ProtocolSlope);
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
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
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

    voltageOffsetControlImplemented = true;
    selectedVoltageOffset.resize(currentChannelsNum);
    voltageOffsetRange.step = 1.0;
    voltageOffsetRange.min = -500.0;
    voltageOffsetRange.max = 500.0;
    voltageOffsetRange.prefix = UnitPfxMilli;
    voltageOffsetRange.unit = "V";
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        selectedVoltageOffset[channelIdx].value = 0.0;
        selectedVoltageOffset[channelIdx].prefix = voltageOffsetRange.prefix;
        selectedVoltageOffset[channelIdx].unit = voltageOffsetRange.unit;
    }

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
            "Elements e4\n"
            "Channels: 4\n"
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
            "Active channels: %activeChannels%\n"; // 2 3 4

    /****************************\
     * Device specific controls *
    \****************************/

    nanionTemperatureControllerFlag = true;

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
    selectStimulusChannelFlag = true;
    singleChannelSSCFlag = true;

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    selectStimulusChannelCoder = new BoolArrayCoder(boolConfig);

    selectStimulusChannelStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        selectStimulusChannelStates[currentIdx] = false;
    }

    /*! Digital offset compensations */
    digitalOffsetCompensationFlag = true;
    singleChannelDOCFlag = true;
    selectableDOCAutostopFlag = false;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 4;
    digitalOffsetCompensationCoder = new BoolArrayCoder(boolConfig);

    digitalOffsetCompensationStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        digitalOffsetCompensationStates[currentIdx] = false;
    }

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationResetCoder = new BoolArrayCoder(boolConfig);

    /*! Zap */
    zappableDeviceFlag = true;
    singleChannelZapFlag = true;

    boolConfig.initialByte = 3;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    zapCoder = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        zapStates[currentIdx] = false;
    }

    /*! Channel off */
    channelOnFlag = true;
    singleChannelOnFlag = true;

    boolConfig.initialByte = 5;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    channelOnCoder = new BoolNegatedArrayCoder(boolConfig);

    channelOnStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        channelOnStates[currentIdx] = false;
    }

    /*! Current range */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 3;
    currentRangeCoders.resize(1);
    currentRangeCoders[0] = new BoolRandomArrayCoder(boolConfig);
    currentRangeCoders[0]->addMapItem(0); /*!< 200pA    -> 0b000 */
    currentRangeCoders[0]->addMapItem(2); /*!< 2nA      -> 0b010 */
    currentRangeCoders[0]->addMapItem(3); /*!< 20nA     -> 0b011 */
    currentRangeCoders[0]->addMapItem(7); /*!< 200nA    -> 0b111 */

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
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    protocolVoltageCoders[ProtocolVHold] = new DoubleOffsetBinaryCoder(doubleConfig);
    doubleConfig.initialByte = 17;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 21;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    protocolVoltageCoders[ProtocolVStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 41;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    protocolVoltageCoders[ProtocolVPk] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 45;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMax].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMax].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMax].max;
    protocolVoltageCoders[ProtocolVMax] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 47;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVMin].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVMin].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVMin].max;
    protocolVoltageCoders[ProtocolVMin] = new DoubleSignAbsCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 23;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    protocolTimeCoders[ProtocolTHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 27;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    protocolTimeCoders[ProtocolTPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 33;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTStep].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTStep].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTStep].max;
    protocolTimeCoders[ProtocolTStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 41;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    protocolTimeCoders[ProtocolTPe] = new DoubleTwosCompCoder(doubleConfig);

    /*! Protocol slope */
    protocolSlopeCoders.resize(ProtocolSlopesNum);
    doubleConfig.initialByte = 43;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolSlopeRanges[ProtocolSlope].step;
    doubleConfig.minValue = protocolSlopeRanges[ProtocolSlope].min;
    doubleConfig.maxValue = protocolSlopeRanges[ProtocolSlope].max;
    protocolSlopeCoders[ProtocolSlope] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);
    doubleConfig.initialByte = 37;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolN].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolN].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolN].max;
    protocolAdimensionalCoders[ProtocolN] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 39;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolNR].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolNR].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolNR].max;
    protocolAdimensionalCoders[ProtocolNR] = new DoubleTwosCompCoder(doubleConfig);

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolRandomArrayCoder(boolConfig);
    dacIntFilterCoder->addMapItem(1); /*!< 1kHz  -> 0b1 */
    dacIntFilterCoder->addMapItem(0); /*!< 10kHz  -> 0b0 */

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        doubleConfig.initialByte = 9+2*channelIdx;
        voltageOffsetCoders[channelIdx] = new DoubleSignAbsCoder(doubleConfig);
    }

    /*! Insertion pulse */
    doubleConfig.initialByte = 19;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = insertionPulseVoltageRange.step;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    insertionPulseVoltageCoder = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 31;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.resolution = insertionPulseDurationRange.step;
    doubleConfig.minValue = insertionPulseDurationRange.min;
    doubleConfig.maxValue = insertionPulseDurationRange.max;
    insertionPulseDurationCoder = new DoubleTwosCompCoder(doubleConfig);
    boolConfig.initialByte = 5;
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
    boolConfig.initialByte = 5;
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
    txStatus[txStatusIdx++] = 0x00; // CFG4
    txStatus[txStatusIdx++] = 0x0F; // CFG5
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs1
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs2
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs3
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs4
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

MessageDispatcher_e4n_El03c_LegacyEdr3_V04::~MessageDispatcher_e4n_El03c_LegacyEdr3_V04() {

}

ErrorCodes_t MessageDispatcher_e4n_El03c_LegacyEdr3_V04::getTemperatureControllerRange(int &minTemperature, int &maxTemperature) {
    minTemperature = minControllerTemperature;
    maxTemperature = maxControllerTemperature;
    return Success;
}

void MessageDispatcher_e4n_El03c_LegacyEdr3_V04::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->selectStimulusChannel(currentChannelsNum, true);
    this->digitalOffsetCompensation(currentChannelsNum, false);
    this->switchChannelOn(currentChannelsNum, true, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_e4n_El03c_LegacyEdr3_V04::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold+Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold-Vamp\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange2_10ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableAmplitude:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(selectedProtocolAdimensional[ProtocolN].value > 0)) {
            validFlag = false;
            message = "N\nmust be at least 1";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolVariableDuration:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].includes(selectedProtocolTime[ProtocolTStep]))) {
            validFlag = false;
            message = "Tstep\nmust be within [-100e6, 100e6]ms";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vmax\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vmin\nmust be within [-500,500]mV";

        } else if (!(protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].includes(selectedProtocolSlope[ProtocolSlope]))) {
            validFlag = false;
            message = "Slope\nmust be within [0, 1000]mV/ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMax]))) {
            validFlag = false;
            message = "Vmax\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVMin]))) {
            validFlag = false;
            message = "Vmin\nmust be within [-500,500]mV";

        } else if (!(protocolSlopeRangesArray[ProtocolSlopeRange2_10mVms].includes(selectedProtocolSlope[ProtocolSlope]))) {
            validFlag = false;
            message = "Slope\nmust be within [0, 1000]mV/ms";

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

MessageDispatcher_e4n_El03c_LegacyEdr3_V05::MessageDispatcher_e4n_El03c_LegacyEdr3_V05(string di) :
    MessageDispatcher_e4e_El03c_LegacyEdr3_V05(di) {

    /****************************\
     * Device specific controls *
    \****************************/

    nanionTemperatureControllerFlag = true;

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;

    digitalOffsetCompensationResetFlag = true;

    /*! Device specific controls */

    customFlagsNum = 1;
    customFlagsNames.resize(customFlagsNum);
    customFlagsNames[0] = "Trigger selection";
    customFlagsDefault.resize(customFlagsNum);
    customFlagsDefault[0] = false;

    customFlagsCoders.resize(customFlagsNum);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    customFlagsCoders[0] = new BoolArrayCoder(boolConfig);
}

MessageDispatcher_e4n_El03c_LegacyEdr3_V05::~MessageDispatcher_e4n_El03c_LegacyEdr3_V05() {

}

ErrorCodes_t MessageDispatcher_e4n_El03c_LegacyEdr3_V05::getTemperatureControllerRange(int &minTemperature, int &maxTemperature) {
    minTemperature = minControllerTemperature;
    maxTemperature = maxControllerTemperature;
    return Success;
}
