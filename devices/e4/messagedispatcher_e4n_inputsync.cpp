#include "messagedispatcher_e4n_inputsync.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_e4n_SineInputSync::MessageDispatcher_e4n_SineInputSync(string di) :
    MessageDispatcher_e4n_V01(di) {

    /************************\
     * Communication format *
    \************************/

    txDataBytes = 55;

    digInSyncImplementedFlag = true;
    fwLoadedOverrideFlag = true;

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
    protocolsNames[ProtocolSinPlusConstant] = "Constant + Sine";
    protocolsNames[ProtocolSinPlusTriangular] = "Triangular + Sine";
    protocolsNames[ProtocolSinPlusSquareWave] = "Square wave + Sine";
    protocolsNames[ProtocolSinPlusConductance] = "Conductance + Sine";
    protocolsNames[ProtocolSinPlusVariableAmplitude] = "Variable Amplitude + Sine";
    protocolsNames[ProtocolSinPlusVariableDuration] = "Variable Duration + Sine";
    protocolsNames[ProtocolSinPlusRamp] = "Ramp + Sine";
    protocolsNames[ProtocolSinPlusCyclicVoltammetry] = "Cyclic Voltammetry + Sine";
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
    protocolsImages[ProtocolSinPlusConstant] = "holdingVoltage001";
    protocolsImages[ProtocolSinPlusTriangular] = "triangularParametric001";
    protocolsImages[ProtocolSinPlusSquareWave] = "sealTest001";
    protocolsImages[ProtocolSinPlusConductance] = "conductance001";
    protocolsImages[ProtocolSinPlusVariableAmplitude] = "stepVariableAmplitude001";
    protocolsImages[ProtocolSinPlusVariableDuration] = "stepVariableDuration001";
    protocolsImages[ProtocolSinPlusRamp] = "ramp002";
    protocolsImages[ProtocolSinPlusCyclicVoltammetry] = "cyclicVoltammetry002";

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

    protocolsAvailableVoltages[ProtocolSinPlusConstant].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusConstant].push_back(ProtocolVSine);
    protocolsAvailableFrequencies[ProtocolSinPlusConstant].push_back(ProtocolFrequency);

    protocolsAvailableVoltages[ProtocolSinPlusTriangular].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusTriangular].push_back(ProtocolVPk);
    protocolsAvailableVoltages[ProtocolSinPlusTriangular].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusTriangular].push_back(ProtocolTPe);
    protocolsAvailableFrequencies[ProtocolSinPlusTriangular].push_back(ProtocolFrequency);

    protocolsAvailableVoltages[ProtocolSinPlusSquareWave].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusSquareWave].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolSinPlusSquareWave].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusSquareWave].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusSquareWave].push_back(ProtocolTPulse);
    protocolsAvailableFrequencies[ProtocolSinPlusSquareWave].push_back(ProtocolFrequency);

    protocolsAvailableVoltages[ProtocolSinPlusConductance].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusConductance].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolSinPlusConductance].push_back(ProtocolVStep);
    protocolsAvailableVoltages[ProtocolSinPlusConductance].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusConductance].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusConductance].push_back(ProtocolTPulse);
    protocolsAvailableFrequencies[ProtocolSinPlusConductance].push_back(ProtocolFrequency);
    protocolsAvailableAdimensionals[ProtocolSinPlusConductance].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolSinPlusConductance].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolSinPlusVariableAmplitude].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusVariableAmplitude].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolSinPlusVariableAmplitude].push_back(ProtocolVStep);
    protocolsAvailableVoltages[ProtocolSinPlusVariableAmplitude].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusVariableAmplitude].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusVariableAmplitude].push_back(ProtocolTPulse);
    protocolsAvailableFrequencies[ProtocolSinPlusVariableAmplitude].push_back(ProtocolFrequency);
    protocolsAvailableAdimensionals[ProtocolSinPlusVariableAmplitude].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolSinPlusVariableAmplitude].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolSinPlusVariableDuration].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusVariableDuration].push_back(ProtocolVPulse);
    protocolsAvailableVoltages[ProtocolSinPlusVariableDuration].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusVariableDuration].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusVariableDuration].push_back(ProtocolTPulse);
    protocolsAvailableTimes[ProtocolSinPlusVariableDuration].push_back(ProtocolTStep);
    protocolsAvailableFrequencies[ProtocolSinPlusVariableDuration].push_back(ProtocolFrequency);
    protocolsAvailableAdimensionals[ProtocolSinPlusVariableDuration].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolSinPlusVariableDuration].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolSinPlusRamp].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusRamp].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolSinPlusRamp].push_back(ProtocolVInit);
    protocolsAvailableVoltages[ProtocolSinPlusRamp].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusRamp].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusRamp].push_back(ProtocolTPulse);
    protocolsAvailableTimes[ProtocolSinPlusRamp].push_back(ProtocolTRamp);
    protocolsAvailableFrequencies[ProtocolSinPlusRamp].push_back(ProtocolFrequency);
    protocolsAvailableAdimensionals[ProtocolSinPlusRamp].push_back(ProtocolNR);

    protocolsAvailableVoltages[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolVHold);
    protocolsAvailableVoltages[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolVFinal);
    protocolsAvailableVoltages[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolVInit);
    protocolsAvailableVoltages[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolVSine);
    protocolsAvailableTimes[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolTHold);
    protocolsAvailableTimes[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolTRamp);
    protocolsAvailableFrequencies[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolFrequency);
    protocolsAvailableAdimensionals[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolN);
    protocolsAvailableAdimensionals[ProtocolSinPlusCyclicVoltammetry].push_back(ProtocolNR);

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Vhold";
    protocolVoltageNames[ProtocolVPulse] = "Vpulse";
    protocolVoltageNames[ProtocolVStep] = "Vstep";
    protocolVoltageNames[ProtocolVPk] = "Vamp";
    protocolVoltageNames[ProtocolVFinal] = "Vfinal";
    protocolVoltageNames[ProtocolVInit] = "Vinit";
    protocolVoltageNames[ProtocolVSine] = "Vsine";

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
    protocolVoltageRanges[ProtocolVSine].step = 1.0;
    protocolVoltageRanges[ProtocolVSine].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVSine].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVSine].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVSine].unit = "V";

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
    protocolVoltageDefault[ProtocolVSine].value = 100.0;
    protocolVoltageDefault[ProtocolVSine].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVSine].unit = "V";
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

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Device reset */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    deviceResetOverrideCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltages */
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 53;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 11;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVSine].step;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVSine].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVSine].max;
    protocolVoltageCoders[ProtocolVSine] = new DoubleSignAbsCoder(doubleConfig);

    /*! Protocol frequencies */
    protocolFrequencyCoders.resize(ProtocolFrequenciesNum);
    doubleConfig.initialByte = 51;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 14;
    doubleConfig.minValue = protocolFrequencyRanges[ProtocolFrequency].min;
    doubleConfig.maxValue = protocolFrequencyRanges[ProtocolFrequency].max;
    doubleConfig.resolution = protocolFrequencyRanges[ProtocolFrequency].step;
    protocolFrequencyCoders[ProtocolFrequency] = new DoubleOffsetBinaryCoder(doubleConfig);

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
    txStatus[txStatusIdx++] = 0x00; // VSine
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_e4n_SineInputSync::~MessageDispatcher_e4n_SineInputSync() {

}

bool MessageDispatcher_e4n_SineInputSync::checkProtocolValidity(string &message) {
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

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
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

    case ProtocolSinPlusConstant:
        if (!(protocolVoltageRangesArray[selectedVoltageRangeIdx].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[selectedVoltageRangeIdx].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolFrequencyRangesArray[ProtocolFrequencyRange35Hz].includes(selectedProtocolFrequency[ProtocolFrequency]))) {
            validFlag = false;
            message = "Frequency\nmust be within [0,35]Hz";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSinPlusTriangular:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vamp+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPk]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vamp-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vamp+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPk]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vamp-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange2_10ms].includes(selectedProtocolTime[ProtocolTPe]))) {
            validFlag = false;
            message = "TPeriod\nmust be within [1,1000]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSinPlusSquareWave:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vpulse-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_28].includes(selectedProtocolTime[ProtocolTPulse]))) {
            validFlag = false;
            message = "Tpulse\nmust be within [1, 200e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSinPlusConductance:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold-Vpulse+Vsine-Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vpulse-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]-
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold-Vpulse-Vsine-Vstep(N-1)\nmust be within [-500,500]mV";

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

    case ProtocolSinPlusVariableAmplitude:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine+Vstep(N-1)\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]+
                                                                                    selectedProtocolVoltage[ProtocolVStep]*(selectedProtocolAdimensional[ProtocolN].value-1.0)))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine+Vstep(N-1)\nmust be within [-500,500]mV";

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

    case ProtocolSinPlusVariableDuration:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+selectedProtocolVoltage[ProtocolVPulse]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vpulse-Vsine\nmust be within [-500,500]mV";

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

    case ProtocolSinPlusRamp:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vfinal+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vfinal-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vinit+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vinit-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolTimeRangesArray[ProtocolTimeRange1to2_25].includes(selectedProtocolTime[ProtocolTRamp]))) {
            validFlag = false;
            message = "Tramp\nmust be within [1, 30e6]ms";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;

    case ProtocolSinPlusCyclicVoltammetry:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]+
                                                                             selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vhold-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vfinal+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVFinal]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vfinal-Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]+
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vinit+Vsine\nmust be within [-500,500]mV";

        } else if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVInit]-
                                                                                    selectedProtocolVoltage[ProtocolVSine]))) {
            validFlag = false;
            message = "Vinit-Vsine\nmust be within [-500,500]mV";

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
