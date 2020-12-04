#include "messagedispatcher_e16n.h"

MessageDispatcher_e16n::MessageDispatcher_e16n(string di) :
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
    currentChannelsNum = 16;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+FTD_RX_INFO_WORD_SIZE+(packetsPerFrame*(int)totalChannelsNum)*(int)FTD_RX_WORD_SIZE;

    infoStructSize = sizeof(InfoStruct_t);
    infoStructPtr = (uint8_t *)&infoStruct;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    txDataBytes = 94;

    /**********************\
     * Available settings *
    \**********************/

    /*! Current ranges */
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

    /*! Default values */
    selectedVoltageRangeIdx = defaultVoltageRangeIdx;
    selectedCurrentRangeIdx = defaultCurrentRangeIdx;
    selectedSamplingRateIdx = defaultSamplingRateIdx;

    currentRange = currentRangesArray[selectedCurrentRangeIdx];
    currentResolution = currentRangesArray[selectedCurrentRangeIdx].step;
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step;
    samplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
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
    protocolVoltageRangesArray[ProtocolVoltageRange2V].min = -2000.0;
    protocolVoltageRangesArray[ProtocolVoltageRange2V].max = 2000.0;
    protocolVoltageRangesArray[ProtocolVoltageRange2V].step = 0.0625;
    protocolVoltageRangesArray[ProtocolVoltageRange2V].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange2V].unit = "V";

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
    protocolTimeRangesArray[ProtocolTimeRange1orMore].max = numeric_limits <double> ::max();
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
//    protocolVoltageNames[ProtocolVExt] = "Dac ext";

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
    protocolVoltageRanges[ProtocolVFinal].step = 1.0/256.0;
    protocolVoltageRanges[ProtocolVFinal].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVFinal].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVFinal].unit = "V";
    protocolVoltageRanges[ProtocolVInit].step = 1.0/256.0;
    protocolVoltageRanges[ProtocolVInit].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVInit].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVInit].unit = "V";
//    protocolVoltageRanges[ProtocolVExt].step = 4096.0/1048576.0;
//    protocolVoltageRanges[ProtocolVExt].min = 0.0;
//    protocolVoltageRanges[ProtocolVExt].max = 4096.0-doubleConfig.resolution;
//    protocolVoltageRanges[ProtocolVExt].prefix = UnitPfxMilli;
//    protocolVoltageRanges[ProtocolVExt].unit = "V";

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
    protocolVoltageDefault[ProtocolVFinal].value = 100.0;
    protocolVoltageDefault[ProtocolVFinal].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVFinal].unit = "V";
    protocolVoltageDefault[ProtocolVInit].value = -100.0;
    protocolVoltageDefault[ProtocolVInit].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVInit].unit = "V";
//    protocolVoltageDefault[ProtocolVExt].value = 2048.0;
//    protocolVoltageDefault[ProtocolVExt].prefix = UnitPfxMilli;
//    protocolVoltageDefault[ProtocolVExt].unit = "V";
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
    protocolTimeRanges[ProtocolTPulse].max = UINT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTPulse].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTPulse].unit = "s";
    protocolTimeRanges[ProtocolTStep].step = 1.0;
    protocolTimeRanges[ProtocolTStep].min = INT28_MIN*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTStep].max = INT28_MAX*protocolTimeRanges[ProtocolTHold].step;
    protocolTimeRanges[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeRanges[ProtocolTStep].unit = "s";
    protocolTimeRanges[ProtocolTRamp].step = 1.0;
    protocolTimeRanges[ProtocolTRamp].min = 0.0;
    protocolTimeRanges[ProtocolTRamp].max = UINT28_MAX*protocolTimeRanges[ProtocolTHold].step;
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
    protocolTimeDefault[ProtocolTStep].value = 100.0;
    protocolTimeDefault[ProtocolTStep].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTStep].unit = "s";
    protocolTimeDefault[ProtocolTRamp].value = 100.0;
    protocolTimeDefault[ProtocolTRamp].prefix = UnitPfxMilli;
    protocolTimeDefault[ProtocolTRamp].unit = "s";
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

    protocolSlopeRanges.resize(ProtocolSlopesNum);

    protocolSlopeDefault.resize(ProtocolSlopesNum);
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

    /****************************\
     * Device specific controls *
    \****************************/

    washerControlFlag = true;

    washerSpeedRange.min = -100.0;
    washerSpeedRange.max = 100.0;
    washerSpeedRange.step = 1.0;
    washerSpeedRange.prefix = UnitPfxNone;
    washerSpeedRange.unit = "rpm";

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

//    boolConfig.initialByte = 93;
//    boolConfig.initialBit = 0;
//    boolConfig.bitsNum = 1;
//    VcSel0Coder = new BoolArrayCoder(boolConfig);
//    boolConfig.initialByte = 93;
//    boolConfig.initialBit = 1;
//    boolConfig.bitsNum = 1;
//    VcSel1Coder = new BoolArrayCoder(boolConfig);

    /*! Current range */
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
    boolConfig.initialByte = 9;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 4;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 9;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltages */
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 16;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    protocolVoltageCoders[ProtocolVHold] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 50;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPulse].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPulse].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPulse].max;
    protocolVoltageCoders[ProtocolVPulse] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 54;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVStep].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVStep].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVStep].max;
    protocolVoltageCoders[ProtocolVStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 74;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 2;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVPk].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVPk].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVPk].max;
    protocolVoltageCoders[ProtocolVPk] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 80;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 20;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVFinal].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVFinal].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVFinal].max;
    protocolVoltageCoders[ProtocolVFinal] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 83;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 20;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVInit].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVInit].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVInit].max;
    protocolVoltageCoders[ProtocolVInit] = new DoubleSignAbsCoder(doubleConfig);
//    doubleConfig.initialByte = 90;
//    doubleConfig.initialBit = 0;
//    doubleConfig.bitsNum = 20;
//    doubleConfig.resolution = protocolVoltageRanges[ProtocolVExt].step;
//    doubleConfig.minValue = protocolVoltageRanges[ProtocolVExt].min;
//    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVExt].max;
//    protocolVoltageCoders[ProtocolVExt] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);
    doubleConfig.initialByte = 56;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTHold].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTHold].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTHold].max;
    protocolTimeCoders[ProtocolTHold] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 60;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPulse].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPulse].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPulse].max;
    protocolTimeCoders[ProtocolTPulse] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 66;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTStep].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTStep].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTStep].max;
    protocolTimeCoders[ProtocolTStep] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 76;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 28;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTRamp].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTRamp].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTRamp].max;
    protocolTimeCoders[ProtocolTRamp] = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 74;
    doubleConfig.initialBit = 2;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolTimeRanges[ProtocolTPe].step;
    doubleConfig.minValue = protocolTimeRanges[ProtocolTPe].min;
    doubleConfig.maxValue = protocolTimeRanges[ProtocolTPe].max;
    protocolTimeCoders[ProtocolTPe] = new DoubleTwosCompCoder(doubleConfig);

    /*! Protocol slope */
    protocolSlopeCoders.resize(ProtocolSlopesNum);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);
    doubleConfig.initialByte = 70;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolN].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolN].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolN].max;
    protocolAdimensionalCoders[ProtocolN] = new DoubleTwosCompCoder(doubleConfig);
    doubleConfig.initialByte = 72;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.resolution = protocolAdimensionalRanges[ProtocolNR].step;
    doubleConfig.minValue = protocolAdimensionalRanges[ProtocolNR].min;
    doubleConfig.maxValue = protocolAdimensionalRanges[ProtocolNR].max;
    protocolAdimensionalCoders[ProtocolNR] = new DoubleTwosCompCoder(doubleConfig);

    boolConfig.initialByte = 1;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolArrayCoder(boolConfig);

    /*! Insertion pulse */
    doubleConfig.initialByte = 52;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = insertionPulseVoltageRange.step;
    doubleConfig.minValue = insertionPulseVoltageRange.min;
    doubleConfig.maxValue = insertionPulseVoltageRange.max;
    insertionPulseVoltageCoder = new DoubleSignAbsCoder(doubleConfig);
    doubleConfig.initialByte = 64;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.resolution = insertionPulseDurationRange.step;
    doubleConfig.minValue = insertionPulseDurationRange.min;
    doubleConfig.maxValue = insertionPulseDurationRange.max;
    insertionPulseDurationCoder = new DoubleTwosCompCoder(doubleConfig);
    boolConfig.initialByte = 9;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    insertionPulseApplyCoder = new BoolArrayCoder(boolConfig);

    /*! Device specific controls */
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 1;
    washerResetCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    washerGetStatusCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    washerGetSpeedsCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 5;
    boolConfig.initialBit = 6;
    boolConfig.bitsNum = 1;
    washerSetSpeedsCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 8;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    washerStartCoder = new BoolArrayCoder(boolConfig);
    boolConfig.initialByte = 8;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 2;
    washerSelectSpeedCoder = new BoolArrayCoder(boolConfig);

    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 8;
    doubleConfig.resolution = washerSpeedRange.step;
    doubleConfig.minValue = washerSpeedRange.min;
    doubleConfig.maxValue = washerSpeedRange.max;
    washerPresetSpeedsCoders.resize(WasherSpeedsNum);
    for (unsigned short speedIdx = 0; speedIdx < WasherSpeedsNum; speedIdx++) {
        doubleConfig.initialByte = 86+speedIdx*2;
        washerPresetSpeedsCoders[speedIdx] = new DoubleTwosCompCoder(doubleConfig);
    }

    /*******************\
     * Default status  *
    \*******************/

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
    txStatus[76] = 0x00; // TRamp
    txStatus[77] = 0x00;
    txStatus[78] = 0x00;
    txStatus[79] = 0x00;
    txStatus[80] = 0x00; // VFinal
    txStatus[81] = 0x00;
    txStatus[82] = 0x00;
    txStatus[83] = 0x00; // VInit
    txStatus[84] = 0x00;
    txStatus[85] = 0x00;
    txStatus[86] = 0x00; // Wash_pre1
    txStatus[87] = 0x00;
    txStatus[88] = 0x00; // Wash_pre2
    txStatus[89] = 0x00;
    txStatus[90] = 0x00; // Wash_pre3
    txStatus[91] = 0x00;
    txStatus[92] = 0x00; // Wash_pre4
    txStatus[93] = 0x00;
}

MessageDispatcher_e16n::~MessageDispatcher_e16n() {

}

ErrorCodes_t MessageDispatcher_e16n::resetWasherError() {
    washerResetCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    washerResetCoder->encode(0, txStatus);
    this_thread::sleep_for(chrono::milliseconds(10));
    this->updateWasherStatus();

    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::setWasherPresetSpeeds(vector <int8_t> speedValues) {
    for (unsigned int idx = 0; idx < 4; idx++) {
        washerPresetSpeedsCoders[idx]->encode(speedValues[idx], txStatus);
    }
    washerSetSpeedsCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    washerSetSpeedsCoder->encode(0, txStatus);
    this_thread::sleep_for(chrono::milliseconds(30));

    /*! After setting the speeds get them, in a separate request to give a bit of time for the eeprom update */
    this->updateWasherSpeeds();

    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::startWasher(uint16_t speedIdx) {
    if (speedIdx < WasherSpeedsNum) {
        washerSelectSpeedCoder->encode(speedIdx, txStatus);
        washerStartCoder->encode(1, txStatus);
        this->stackOutgoingMessage(txStatus);
        washerStartCoder->encode(0, txStatus);
        this_thread::sleep_for(chrono::milliseconds(10));

        return Success;

    } else {
        return ErrorValueOutOfRange;
    }
}

ErrorCodes_t MessageDispatcher_e16n::updateWasherState() {
    this->updateWasherStatus();
    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::updateWasherPresetSpeeds() {
    this->updateWasherSpeeds();
    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::getWasherSpeedRange(RangedMeasurement_t &range) {
    range = washerSpeedRange;
    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::getWasherStatus(WasherStatus_t &status, WasherError_t &error) {
    status = (WasherStatus_t)((infoStruct.state & 0xF0) >> 4);
    error = (WasherError_t)(infoStruct.state & 0x0F);
    return Success;
}

ErrorCodes_t MessageDispatcher_e16n::getWasherPresetSpeeds(vector <int8_t> &speedValue) {
    speedValue.resize(WasherSpeedsNum);
    for (unsigned int idx = 0; idx < WasherSpeedsNum; idx++) {
        speedValue[idx] = infoStruct.presetSpeeds[idx];
    }
    return Success;
}

void MessageDispatcher_e16n::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->digitalOffsetCompensation(currentChannelsNum, false);
    this->switchChannelOn(currentChannelsNum, true, false);

    this->selectVoltageProtocol(defaultProtocol);

    for (unsigned int voltageIdx = 0; voltageIdx < ProtocolVoltagesNum; voltageIdx++) {
        this->setProtocolVoltage(voltageIdx, protocolVoltageDefault[voltageIdx], false);
    }

    for (unsigned int timeIdx = 0; timeIdx < ProtocolTimesNum; timeIdx++) {
        this->setProtocolTime(timeIdx, protocolTimeDefault[timeIdx], false);
    }

    for (unsigned int slopeIdx = 0; slopeIdx < ProtocolSlopesNum; slopeIdx++) {
        this->setProtocolSlope(slopeIdx, protocolSlopeDefault[slopeIdx], false);
    }

    for (unsigned int adimensionalIdx = 0; adimensionalIdx < ProtocolAdimensionalsNum; adimensionalIdx++) {
        this->setProtocolAdimensional(adimensionalIdx, protocolAdimensionalDefault[adimensionalIdx], false);
    }

    this->resetWasherError();
    this->updateWasherSpeeds();

//    this->activateDacIntFilter(false, false);
//    this->activateDacExtFilter(true, false);
}

bool MessageDispatcher_e16n::checkProtocolValidity(string &message) {
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
            message = "Vhold-Vpulse-Vstep(N-1)\nmust be within [-500,500]mV";

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

void MessageDispatcher_e16n::updateWasherStatus() {
    washerGetStatusCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    washerGetStatusCoder->encode(0, txStatus);
    this_thread::sleep_for(chrono::milliseconds(10));
}

void MessageDispatcher_e16n::updateWasherSpeeds() {
    washerGetSpeedsCoder->encode(1, txStatus);
    this->stackOutgoingMessage(txStatus);
    washerGetSpeedsCoder->encode(0, txStatus);
    this_thread::sleep_for(chrono::milliseconds(10));
}

MessageDispatcher_dlp::MessageDispatcher_dlp(string di) :
    MessageDispatcher_e16n(di) {

    rxChannel = 'A';
    txChannel = 'A';
}
