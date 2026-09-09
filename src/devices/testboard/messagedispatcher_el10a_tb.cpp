#include "messagedispatcher_el10a_tb.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_EL10a_TB::MessageDispatcher_EL10a_TB(string di) :
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
    currentChannelsNum = 1;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+FTD_RX_INFO_WORD_SIZE+(packetsPerFrame*(int)totalChannelsNum)*(int)FTD_RX_WORD_SIZE;

    infoStructSize = sizeof(InfoStruct_t);
    infoStructPtr = (uint8_t *)&infoStruct;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    txDataBytes = 70;

    /**********************\
     * Available settings *
    \**********************/

    /*! Current ranges */
    independentCurrentRangesFlag = false;
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange25nA].min = -25.0;
    currentRangesArray[CurrentRange25nA].max = 25.0;
    currentRangesArray[CurrentRange25nA].step = currentRangesArray[CurrentRange25nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange25nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange25nA].unit = "A";
    currentRangesArray[CurrentRange50nA].min = -50.0;
    currentRangesArray[CurrentRange50nA].max = 50.0;
    currentRangesArray[CurrentRange50nA].step = currentRangesArray[CurrentRange50nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange50nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange50nA].unit = "A";
    currentRangesArray[CurrentRange100nA].min = -100.0;
    currentRangesArray[CurrentRange100nA].max = 100.0;
    currentRangesArray[CurrentRange100nA].step = currentRangesArray[CurrentRange100nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange100nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange100nA].unit = "A";
    currentRangesArray[CurrentRange200nA].min = -200.0;
    currentRangesArray[CurrentRange200nA].max = 200.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange200nA].unit = "A";
    defaultCurrentRangesIdx.resize(currentChannelsNum);
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        defaultCurrentRangesIdx[channelIdx] = CurrentRange25nA;
    }

    /*! Voltage ranges */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange1650mV].min = -1650.0;
    voltageRangesArray[VoltageRange1650mV].max = 1650.0;
    voltageRangesArray[VoltageRange1650mV].step = voltageRangesArray[VoltageRange1650mV].delta()/(UINT10_MAX-1.0);
    voltageRangesArray[VoltageRange1650mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange1650mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange1650mV;

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate1_25kHz].value = 1.25;
    samplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    samplingRatesArray[SamplingRate2_5kHz].value = 2.5;
    samplingRatesArray[SamplingRate2_5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate2_5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate5kHz].value = 5.0;
    samplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate10kHz].value = 10.0;
    samplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate10kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate1_25kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate1_25kHz].value = 1.25e3/1024.0;
    realSamplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate2_5kHz].value = 1.25e3/512.0;
    realSamplingRatesArray[SamplingRate2_5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate2_5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate5kHz].value = 1.25e3/256.0;
    realSamplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate10kHz].value = 1.25e3/128.0;
    realSamplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate10kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate1_25kHz].value = 1024.0/1.25;
    integrationStepArray[SamplingRate1_25kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate1_25kHz].unit = "s";
    integrationStepArray[SamplingRate2_5kHz].value = 512.0/1.25;
    integrationStepArray[SamplingRate2_5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate2_5kHz].unit = "s";
    integrationStepArray[SamplingRate5kHz].value = 256.0/1.25;
    integrationStepArray[SamplingRate5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate5kHz].unit = "s";
    integrationStepArray[SamplingRate10kHz].value = 128.0/1.25;
    integrationStepArray[SamplingRate10kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate10kHz].unit = "s";

    /*! Oversampling ratios */
    oversamplingImplemented = false;
    oversamplingRatiosNum = OversamplingRatiosNum;
    oversamplingRatiosArray.resize(oversamplingRatiosNum);
    oversamplingRatiosArray[OversamplingRatioX1] = 1;

    /*! Voltage filters */
    dacIntFilterAvailable = false;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);

    dacExtFilterAvailable = false;
    voltageReferenceLpfOptionsNum = VoltageReferenceLpfsNum;
    voltageReferenceLpfOptions.resize(voltageReferenceLpfOptionsNum);

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
    protocolVoltageRangesArray[ProtocolVoltageRange1650mV] = voltageRangesArray[VoltageRange1650mV];

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
    protocolVoltageRanges[ProtocolVHold].step = voltageRangesArray[VoltageRange1650mV].step;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange1650mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange1650mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";
    protocolVoltageRanges[ProtocolVPulse].step = voltageRangesArray[VoltageRange1650mV].step;
    protocolVoltageRanges[ProtocolVPulse].min = voltageRangesArray[VoltageRange1650mV].min;
    protocolVoltageRanges[ProtocolVPulse].max = voltageRangesArray[VoltageRange1650mV].max;
    protocolVoltageRanges[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPulse].unit = "V";
    protocolVoltageRanges[ProtocolVStep].step = voltageRangesArray[VoltageRange1650mV].step;
    protocolVoltageRanges[ProtocolVStep].min = voltageRangesArray[VoltageRange1650mV].min;
    protocolVoltageRanges[ProtocolVStep].max = voltageRangesArray[VoltageRange1650mV].max;
    protocolVoltageRanges[ProtocolVStep].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVStep].unit = "V";
    protocolVoltageRanges[ProtocolVPk].step = 25.0;
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVFinal].step = voltageRangesArray[VoltageRange1650mV].step;
    protocolVoltageRanges[ProtocolVFinal].min = voltageRangesArray[VoltageRange1650mV].min;
    protocolVoltageRanges[ProtocolVFinal].max = voltageRangesArray[VoltageRange1650mV].max;
    protocolVoltageRanges[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVFinal].unit = "V";
    protocolVoltageRanges[ProtocolVInit].step = voltageRangesArray[VoltageRange1650mV].step;
    protocolVoltageRanges[ProtocolVInit].min = voltageRangesArray[VoltageRange1650mV].min;
    protocolVoltageRanges[ProtocolVInit].max = voltageRangesArray[VoltageRange1650mV].max;
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
    voltageOffsetRange.step = voltageRangesArray[VoltageRange1650mV].step;
    voltageOffsetRange.min = voltageRangesArray[VoltageRange1650mV].min;
    voltageOffsetRange.max = voltageRangesArray[VoltageRange1650mV].max;
    voltageOffsetRange.prefix = UnitPfxMilli;
    voltageOffsetRange.unit = "V";
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        selectedVoltageOffset[channelIdx].value = 0.0;
        selectedVoltageOffset[channelIdx].prefix = voltageOffsetRange.prefix;
        selectedVoltageOffset[channelIdx].unit = voltageOffsetRange.unit;
    }

    insertionPulseImplemented = true;
    insertionPulseVoltageRange.step = voltageRangesArray[VoltageRange1650mV].step;
    insertionPulseVoltageRange.min = voltageRangesArray[VoltageRange1650mV].min;
    insertionPulseVoltageRange.max = voltageRangesArray[VoltageRange1650mV].max;
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
        "Elements EL10a testboardn\n"
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
        "Active channels: %activeChannels%\n"; // 2 3 4 5 6 7 8 9 10 12 13 14 15 16

    /****************************\
     * Device specific controls *
    \****************************/

    customFlagsNum = CustomFlagsNum;
    customFlagsNames.resize(customFlagsNum);
    customFlagsNames[CustomFlagBgRef3_3VLdoEn] = "Enable BG reference for 3.3V LDO";
    customFlagsNames[CustomFlag3_3VLdoEn] = "Enable 3.3V LDO";
    customFlagsNames[CustomFlagVddUcLdoEn] = "Enable Vdd_uC LDO";
    customFlagsNames[CustomFlag5VLdoEn] = "Enable 5V LDO";
    customFlagsNames[CustomFlagBiasChargePump4VEn] = "Enable Bias voltage for 4V charge pump";
    customFlagsNames[CustomFlagChargePump4VEn] = "Enable 4V charge pump";
    customFlagsNames[CustomFlagChargePump6VEn] = "Enable 6V charge pump";
    customFlagsNames[CustomFlagVmidEn] = "Enable Vmid generator";
    customFlagsNames[CustomFlagHeater1En] = "Heater 1 ON";
    customFlagsNames[CustomFlagHeater2En] = "Heater 2 ON";
    customFlagsNames[CustomFlagHeater3En] = "Heater 3 ON";
    customFlagsNames[CustomFlagReSel1] = "Select RE 1";
    customFlagsNames[CustomFlagReSel2] = "Select RE 2";
    customFlagsNames[CustomFlagReSel3] = "Select RE 3";
    customFlagsNames[CustomFlagReSel4] = "Select RE 4";
    customFlagsNames[CustomFlagWeSel1] = "Select WE 1";
    customFlagsNames[CustomFlagWeSel2] = "Select WE 2";
    customFlagsNames[CustomFlagWeSel3] = "Select WE 3";
    customFlagsNames[CustomFlagWeSel4] = "Select WE 4";
    customFlagsDefault.resize(customFlagsNum);
    customFlagsDefault[CustomFlagBgRef3_3VLdoEn] = false;
    customFlagsDefault[CustomFlag3_3VLdoEn] = false;
    customFlagsDefault[CustomFlagVddUcLdoEn] = false;
    customFlagsDefault[CustomFlag5VLdoEn] = false;
    customFlagsDefault[CustomFlagBiasChargePump4VEn] = false;
    customFlagsDefault[CustomFlagChargePump4VEn] = false;
    customFlagsDefault[CustomFlagChargePump6VEn] = false;
    customFlagsDefault[CustomFlagVmidEn] = false;
    customFlagsDefault[CustomFlagHeater1En] = false;
    customFlagsDefault[CustomFlagHeater2En] = false;
    customFlagsDefault[CustomFlagHeater3En] = false;
    customFlagsDefault[CustomFlagReSel1] = true;
    customFlagsDefault[CustomFlagReSel2] = false;
    customFlagsDefault[CustomFlagReSel3] = false;
    customFlagsDefault[CustomFlagReSel4] = false;
    customFlagsDefault[CustomFlagWeSel1] = true;
    customFlagsDefault[CustomFlagWeSel2] = false;
    customFlagsDefault[CustomFlagWeSel3] = false;
    customFlagsDefault[CustomFlagWeSel4] = false;

    customOptionsNum = CustomOptionsNum;
    customOptionsNames.resize(customOptionsNum);
    customOptionsNames[CustomOptionVmidSel] = "Vmid voltage";
    customOptionsNames[CustomOptionCeSel] = "CE selection";
    customOptionsNames[CustomOptionReFilter] = "RE filter";
    customOptionsNames[CustomOptionClock] = "Clock";
    customOptionsDescriptions.resize(customOptionsNum);
    customOptionsDescriptions[CustomOptionVmidSel].resize(3);
    customOptionsDescriptions[CustomOptionVmidSel][0] = "0.2V";
    customOptionsDescriptions[CustomOptionVmidSel][1] = "1.65V";
    customOptionsDescriptions[CustomOptionVmidSel][2] = "3.1V";
    customOptionsDescriptions[CustomOptionCeSel].resize(4);
    customOptionsDescriptions[CustomOptionCeSel][0] = "CE 1";
    customOptionsDescriptions[CustomOptionCeSel][1] = "CE 2";
    customOptionsDescriptions[CustomOptionCeSel][2] = "CE 3";
    customOptionsDescriptions[CustomOptionCeSel][3] = "CE 4";
    customOptionsDescriptions[CustomOptionReFilter].resize(4);
    customOptionsDescriptions[CustomOptionReFilter][0] = "100 Hz";
    customOptionsDescriptions[CustomOptionReFilter][1] = "500 Hz";
    customOptionsDescriptions[CustomOptionReFilter][2] = "2 kHz";
    customOptionsDescriptions[CustomOptionReFilter][3] = "10 kHz";
    customOptionsDescriptions[CustomOptionClock].resize(4);
    customOptionsDescriptions[CustomOptionClock][0] = "10 MHz";
    customOptionsDescriptions[CustomOptionClock][1] = "5 MHz";
    customOptionsDescriptions[CustomOptionClock][2] = "2.5 MHz";
    customOptionsDescriptions[CustomOptionClock][3] = "1.25 MHz";
    customOptionsDefault.resize(customOptionsNum);
    customOptionsDefault[CustomOptionVmidSel] = 0;
    customOptionsDefault[CustomOptionCeSel] = 0;
    customOptionsDefault[CustomOptionReFilter] = 0;
    customOptionsDefault[CustomOptionClock] = 0;

    customDoublesNum = CustomDoublesNum;
    customDoublesNames.resize(customDoublesNum);
    customDoublesNames[CustomDoubleTemperature] = "Temperature";
    customDoublesRanges.resize(customDoublesNum);
    customDoublesRanges[CustomDoubleTemperature].min = 36.0;
    customDoublesRanges[CustomDoubleTemperature].max = 163.5;
    customDoublesRanges[CustomDoubleTemperature].step = 0.5;
    customDoublesRanges[CustomDoubleTemperature].prefix = UnitPfxNone;
    customDoublesRanges[CustomDoubleTemperature].unit = "°C";
    customDoublesDefault.resize(customDoublesNum);
    customDoublesDefault[CustomDoubleTemperature] = 0.0;

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
    digitalOffsetCompensationFlag = false;
    singleChannelDOCFlag = false;

    /*! Zap */
    zappableDeviceFlag = false;
    singleChannelZapFlag = false;

    /*! Channel off */
    channelOnFlag = false;
    singleChannelOnFlag = false;

    /*! Current range */
    boolConfig.initialByte = 12;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 2;
    currentRangeCoders.resize(1);
    currentRangeCoders[0] = new BoolArrayCoder(boolConfig);

    /*! Voltage range */
    boolConfig.initialByte = 11;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    voltageRangeCoder = new BoolArrayCoder(boolConfig);

    /*! Sampling rate */
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 3;
    samplingRateCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol selection */
    boolConfig.initialByte = 15;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 15;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltages */
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 16;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    protocolVoltageCoders[ProtocolVHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 19;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 22;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    protocolVoltageCoders[ProtocolVStep] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 25;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVInit].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVInit].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVInit].max;
    protocolVoltageCoders[ProtocolVInit] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 28;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVFinal].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVFinal].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVFinal].max;
    protocolVoltageCoders[ProtocolVFinal] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 51;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    protocolVoltageCoders[ProtocolVPk] = new DoubleSignAbsCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 31;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    protocolTimeCoders[ProtocolTHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 35;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    protocolTimeCoders[ProtocolTPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 39;
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
    doubleConfig.initialByte = 51;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    protocolTimeCoders[ProtocolTPe] = new DoubleTwosCompCoder(doubleConfig);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);
    doubleConfig.initialByte = 47;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolN].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolN].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolN].max;
    protocolAdimensionalCoders[ProtocolN] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 49;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolNR].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolNR].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolNR].max;
    protocolAdimensionalCoders[ProtocolNR] = new DoubleTwosCompCoder(doubleConfig);

    boolConfig.initialByte = 13;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 2;
    dacIntFilterCoder = new BoolArrayCoder(boolConfig);

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);
    doubleConfig.initialByte = 53;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        voltageOffsetCoders[channelIdx] = new DoubleTwosCompCoder(doubleConfig);
        doubleConfig.initialByte += 3;
    }

    /*! Insertion pulse */
    doubleConfig.initialByte = 56;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 16;
    doubleConfig.resolution = insertionPulseVoltageRange.step;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    insertionPulseVoltageCoder = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 59;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.resolution = insertionPulseDurationRange.step;
    doubleConfig.minValue = insertionPulseDurationRange.min;
    doubleConfig.maxValue = insertionPulseDurationRange.max;
    insertionPulseDurationCoder = new DoubleTwosCompCoder(doubleConfig);
    boolConfig.initialByte = 15;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    insertionPulseApplyCoder = new BoolArrayCoder(boolConfig);

    /*! Device specific controls */
    customFlagsCoders.resize(customFlagsNum);
    boolConfig.initialByte = 1;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagBgRef3_3VLdoEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlag3_3VLdoEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagVddUcLdoEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlag5VLdoEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagBiasChargePump4VEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagChargePump4VEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagChargePump6VEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagVmidEn] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagHeater1En] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagHeater2En] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagHeater3En] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagReSel1] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagReSel2] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagReSel3] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 4;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagReSel4] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagWeSel1] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagWeSel2] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagWeSel3] = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    customFlagsCoders[CustomFlagWeSel4] = new BoolArrayCoder(boolConfig);

    customOptionsCoders.resize(customOptionsNum);
    boolConfig.initialByte = 3;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 2;
    BoolArrayCoder * vMidSelCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 6;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    BoolArrayCoder * pNCoder = new BoolArrayCoder(boolConfig);
    customOptionsCoders[CustomOptionVmidSel] = new EnsembleCoder();

    static_cast <EnsembleCoder *> (customOptionsCoders[CustomOptionVmidSel])->addCoder(vMidSelCoder);
    static_cast <EnsembleCoder *> (customOptionsCoders[CustomOptionVmidSel])->addCoder(pNCoder);
    static_cast <EnsembleCoder *> (customOptionsCoders[CustomOptionVmidSel])->addMapItem(0x0); // PN = 0b0, VMidSel = 0b00
    static_cast <EnsembleCoder *> (customOptionsCoders[CustomOptionVmidSel])->addMapItem(0x2); // PN = 0b0, VMidSel = 0b10
    static_cast <EnsembleCoder *> (customOptionsCoders[CustomOptionVmidSel])->addMapItem(0x5); // PN = 0b1, VMidSel = 0b01

    boolConfig.initialByte = 4;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 4;
    customOptionsCoders[CustomOptionCeSel] = new BoolOneHotCoder(boolConfig);

    boolConfig.initialByte = 7;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 2;
    customOptionsCoders[CustomOptionReFilter] = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 7;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 2;
    customOptionsCoders[CustomOptionClock] = new BoolArrayCoder(boolConfig);

    customDoublesCoders.resize(customDoublesNum);
    doubleConfig.initialByte = 63;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 8;
    doubleConfig.minValue = customDoublesRanges[CustomDoubleTemperature].min;
    doubleConfig.maxValue = customDoublesRanges[CustomDoubleTemperature].max;
    doubleConfig.resolution = customDoublesRanges[CustomDoubleTemperature].step;
    customDoublesCoders[CustomDoubleTemperature] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*******************\
     * Default status  *
    \*******************/

    txStatus.resize(txDataBytes);

    int txStatusIdx = 0;
    txStatus[txStatusIdx++] = txSyncWord; // HDR
    txStatus[txStatusIdx++] = 0x00; // CFG0
    txStatus[txStatusIdx++] = 0x0A; // CFG1 delta sigma enabled, Vdd uC = 3V
    txStatus[txStatusIdx++] = 0x00; // CFG2
    txStatus[txStatusIdx++] = 0x04; // CFG3 heater circuitry on
    txStatus[txStatusIdx++] = 0x00; // CFG4
    txStatus[txStatusIdx++] = 0x48; // CFG5 WE front end enabled, Stimulus generator enabled
    txStatus[txStatusIdx++] = 0x00; // CFG6
    txStatus[txStatusIdx++] = 0x00; // CFG7
    txStatus[txStatusIdx++] = 0x00; // CFG8
    txStatus[txStatusIdx++] = 0x00; // CFG9
    txStatus[txStatusIdx++] = 0x00; // ranges
    txStatus[txStatusIdx++] = 0x00; // ranges
    txStatus[txStatusIdx++] = 0x00; // filters
    txStatus[txStatusIdx++] = 0x01; // filters 25 kHz anti aliasing filter
    txStatus[txStatusIdx++] = 0x00; // Protocol_cfg
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VStep
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VInit
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VFinal
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // THold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TStep
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TRamp
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // N
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // NR
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Triangular
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VInsPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // TInsPulse
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // Temperature
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_EL10a_TB::~MessageDispatcher_EL10a_TB() {

}

void MessageDispatcher_EL10a_TB::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_EL10a_TB::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold+Vamp\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]))) {
            validFlag = false;
            message = "Vhold-Vamp\nmust be within [-1650,1650]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange2_10ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-1650,1650]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold-Vpulse\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold-Vpulse-Vstep(N-1)\nmust be within [-1650,1650]mV";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vstep(N-1)\nmust be within [-1650,1650]mV";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]))) {
            validFlag = false;
            message = "Vhold+Vpulse\nmust be within [-1650,1650]mV";

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
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-1650,1650]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVFinal]))) {
            validFlag = false;
            message = "Vfinal\nmust be within [-1650,1650]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange1650mV].includes(selectedProtocolVoltage[ProtocolVInit]))) {
            validFlag = false;
            message = "Vinit\nmust be within [-1650,1650]mV";

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
