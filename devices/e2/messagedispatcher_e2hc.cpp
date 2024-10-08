#include "messagedispatcher_e2hc.h"

using namespace std;
using namespace er4CommLib;

MessageDispatcher_e2HC_V00::MessageDispatcher_e2HC_V00(string di) :
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
    currentChannelsNum = 2;
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
    independentCurrentRangesFlag = true;
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200nA].min = -200.0;
    currentRangesArray[CurrentRange200nA].max = 200.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange200nA].unit = "A";
    currentRangesArray[CurrentRange4uA].min = -4.0;
    currentRangesArray[CurrentRange4uA].max = 4.0;
    currentRangesArray[CurrentRange4uA].step = currentRangesArray[CurrentRange4uA].max/SHORT_MAX;
    currentRangesArray[CurrentRange4uA].prefix = UnitPfxMicro;
    currentRangesArray[CurrentRange4uA].unit = "A";
    defaultCurrentRangesIdx.resize(currentChannelsNum);
    defaultCurrentRangesIdx[0] = CurrentRange200nA;
    defaultCurrentRangesIdx[1] = CurrentRange4uA;

    /*! Voltage ranges */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange500mV].min = -500.0;
    voltageRangesArray[VoltageRange500mV].max = 500.0;
    voltageRangesArray[VoltageRange500mV].step = voltageRangesArray[VoltageRange500mV].max/(SHORT_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange500mV;

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate62_5kHz].value = 62.5;
    samplingRatesArray[SamplingRate62_5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate62_5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate250kHz].value = 250.0;
    samplingRatesArray[SamplingRate250kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate250kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate250kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate62_5kHz].value = 62.5;
    realSamplingRatesArray[SamplingRate62_5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate62_5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate250kHz].value = 250.0;
    realSamplingRatesArray[SamplingRate250kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate250kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate62_5kHz].value = 16.0;
    integrationStepArray[SamplingRate62_5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate62_5kHz].unit = "s";
    integrationStepArray[SamplingRate250kHz].value = 4.0;
    integrationStepArray[SamplingRate250kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate250kHz].unit = "s";

    /*! Overampling ratios */
    oversamplingImplemented = false;
    oversamplingRatiosNum = OversamplingRatiosNum;
    oversamplingRatiosArray.resize(oversamplingRatiosNum);
    oversamplingRatiosArray[OversamplingRatioX1] = 1;

    /*! Voltage filters */
    dacIntFilterAvailable = true;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);
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
    protocolVoltageRanges[ProtocolVHold].step = 0.25;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";
    protocolVoltageRanges[ProtocolVPulse].step = 0.25;
    protocolVoltageRanges[ProtocolVPulse].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVPulse].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVPulse].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPulse].unit = "V";
    protocolVoltageRanges[ProtocolVStep].step = 0.25;
    protocolVoltageRanges[ProtocolVStep].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVStep].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVStep].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVStep].unit = "V";
    protocolVoltageRanges[ProtocolVPk].step = 25.0;
    protocolVoltageRanges[ProtocolVPk].min = 25.0;
    protocolVoltageRanges[ProtocolVPk].max = 4.0*protocolVoltageRanges[ProtocolVPk].step;
    protocolVoltageRanges[ProtocolVPk].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVPk].unit = "V";
    protocolVoltageRanges[ProtocolVFinal].step = 0.25;
    protocolVoltageRanges[ProtocolVFinal].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVFinal].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVFinal].unit = "V";
    protocolVoltageRanges[ProtocolVInit].step = 0.25;
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
    voltageOffsetRange.step = 0.25;
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
    insertionPulseVoltageRange.step = 0.25;
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
            "Elements e2HC\n"
            "Channels: 2\n"
            "\n"
            "Data header file\n"
            "\n"
            "Amplifier Setup\n"
            "Range: %currentRange%\n" // 200 pA
            "Sampling frequency (SR): %samplingRate%\n" // 62.5 kHz
            "Final Bandwidth: SR/2 (no filter)\n"
            "\n"
            "Acquisition start time: %dateHour%\n" // 04/11/2020 11:28:55.130
            "\n"
            "Active channels: %activeChannels%\n"; // 2

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
    selectStimulusChannelFlag = true;
    singleChannelSSCFlag = true;

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 2;
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
    boolConfig.bitsNum = 2;
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
    zappableDeviceFlag = false;
    singleChannelZapFlag = false;

//    boolConfig.initialByte = 3;
//    boolConfig.initialBit = 0;
//    boolConfig.bitsNum = 4;
//    zapCoder = new BoolArrayCoder(boolConfig);

//    zapStates.resize(currentChannelsNum);
//    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
//        zapStates[currentIdx] = false;
//    }

    /*! Channel off */
    channelOnFlag = true;
    singleChannelOnFlag = true;

    boolConfig.initialByte = 5;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 2;
    channelOnCoder = new BoolArrayCoder(boolConfig);

    channelOnStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        channelOnStates[currentIdx] = false;
    }

    /*! Current range */
    currentRangeCoders.resize(currentChannelsNum);

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    currentRangeCoders[0] = new BoolRandomArrayCoder(boolConfig);
    currentRangeCoders[0]->addMapItem(1); /*!< 200nA  -> 0b1 */
    currentRangeCoders[0]->addMapItem(0); /*!< 4uA    -> 0b0 */

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    currentRangeCoders[1] = new BoolRandomArrayCoder(boolConfig);
    currentRangeCoders[1]->addMapItem(1); /*!< 200nA  -> 0b1 */
    currentRangeCoders[1]->addMapItem(0); /*!< 4uA    -> 0b0 */

    /*! Voltage range */
    boolConfig.initialByte = 0;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    voltageRangeCoder = new BoolRandomArrayCoder(boolConfig);
    voltageRangeCoder->addMapItem(0); /*!< No controls  -> 0b0 */

    /*! Sampling rate */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 5;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    samplingRateCoder->addMapItem(17); /*!< 62.5kHz      -> 0b10001 */
    samplingRateCoder->addMapItem(8);  /*!< 250kHz       -> 0b01000 */

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
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
    protocolVoltageCoders[ProtocolVHold] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 17;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 21;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
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
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVFinal].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVFinal].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVFinal].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
    protocolVoltageCoders[ProtocolVFinal] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 49;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVInit].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVInit].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVInit].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
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

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolRandomArrayCoder(boolConfig);
    dacIntFilterCoder->addMapItem(1); /*!< 10kHz     -> 0b1 */
    /*! DON'T USE  0b0: the filter must be active because the DAC is a 3rd order DS and this filter accounts for the 4th order filtering */

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        doubleConfig.initialByte = 9+2*channelIdx;
        voltageOffsetCoders[channelIdx] = new DoubleSignAbsCoder(doubleConfig);
    }

    /*! Insertion pulse */
    doubleConfig.initialByte = 19;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    doubleConfig.resolution = insertionPulseVoltageRange.max/(INT14_MAX+1)*stimulusVoltageReference/stimulusVoltageLimit;
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

    /*******************\
     * Default status  *
    \*******************/

    txStatus.resize(txDataBytes);

    int txStatusIdx = 0;
    txStatus[txStatusIdx++] = txSyncWord; // HDR
    txStatus[txStatusIdx++] = 0x3A; // CFG0
    txStatus[txStatusIdx++] = 0x01; // CFG1
    txStatus[txStatusIdx++] = 0x05; // CFG2
    txStatus[txStatusIdx++] = 0x00; // CFG3
    txStatus[txStatusIdx++] = 0x03; // CFG4
    txStatus[txStatusIdx++] = 0x03; // CFG5
    txStatus[txStatusIdx++] = 0x00; // Vhold
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs1
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // VOfs2
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // free
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00; // free
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

MessageDispatcher_e2HC_V00::~MessageDispatcher_e2HC_V00() {

}

void MessageDispatcher_e2HC_V00::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->selectStimulusChannel(currentChannelsNum, true);
    this->digitalOffsetCompensation(currentChannelsNum, false);
    this->switchChannelOn(currentChannelsNum, true, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_e2HC_V00::checkProtocolValidity(string &message) {
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

MessageDispatcher_e2HC_V01::MessageDispatcher_e2HC_V01(string di) :
    MessageDispatcher_e2HC_V00(di) {

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate50kHz].value = 50.0;
    samplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate50kHz].unit = "Hz";
    samplingRatesArray[SamplingRate25kHz].value = 25.0;
    samplingRatesArray[SamplingRate25kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate25kHz].unit = "Hz";
    samplingRatesArray[SamplingRate12_5kHz].value = 12.5;
    samplingRatesArray[SamplingRate12_5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate12_5kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate25kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate50kHz].value = 50.0e3/(8.0*128.0); /*!< 48.8kHz */
    realSamplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate50kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate25kHz].value = 50.0e3/(8.0*256.0); /*!< 24.4kHz */
    realSamplingRatesArray[SamplingRate25kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate25kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate12_5kHz].value = 50.0e3/(8.0*512.0); /*!< 12.2kHz */
    realSamplingRatesArray[SamplingRate12_5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate12_5kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate50kHz].value = 20.48;
    integrationStepArray[SamplingRate50kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate50kHz].unit = "s";
    integrationStepArray[SamplingRate25kHz].value = 40.96;
    integrationStepArray[SamplingRate25kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate25kHz].unit = "s";
    integrationStepArray[SamplingRate12_5kHz].value = 81.92;
    integrationStepArray[SamplingRate12_5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate12_5kHz].unit = "s";

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
    samplingRateCoder->addMapItem(1);  /*!< 50kHz        -> 0b00001 */
    samplingRateCoder->addMapItem(9);  /*!< 25kHz        -> 0b01001 */
    samplingRateCoder->addMapItem(17); /*!< 12.5kHz      -> 0b10001 */
}

MessageDispatcher_e2HC_V01::~MessageDispatcher_e2HC_V01() {

}
