//  Copyright (C) 2021 Filippo Cona
//
//  This file is part of EDR4.
//
//  EDR4 is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  EDR4 is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with EDR4.  If not, see <http://www.gnu.org/licenses/>.

#ifndef MESSAGEDISPATCHER_ENPR_H
#define MESSAGEDISPATCHER_ENPR_H

#include "messagedispatcher.h"

#include <iostream>

using namespace std;

class MessageDispatcher_eNPR : public MessageDispatcher {
public:
    MessageDispatcher_eNPR(string di);
    virtual ~MessageDispatcher_eNPR();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200pA,
        CurrentRange2nA,
        CurrentRange20nA,
        CurrentRange200nA,
        CurrentRangesNum
    };

    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRate200kHz,
        SamplingRatesNum
    };

    enum OveramplingRatios {
        OversamplingRatioX1,
        OversamplingRatioX4,
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf10kHz,
        VoltageStimulusLpfsNum
    };

    enum VoltageReferenceLpfs {
        VoltageReferenceLpf3Hz,
        VoltageReferenceLpf180kHz,
        VoltageReferenceLpfsNum
    };

    enum ProtocolTimeRanges {
        ProtocolTimeRange2_10ms,
        ProtocolTimeRange0to2_28,
        ProtocolTimeRange1to2_28,
        ProtocolTimeRange1orMore,
        ProtocolTimeRangeSigned2_27,
        ProtocolTimeRange1to2_25,
        ProtocolTimeRangesNum
    };

    enum ProtocolVoltages {
        ProtocolVHold,
        ProtocolVPulse,
        ProtocolVStep,
        ProtocolVPk,
        ProtocolVFinal,
        ProtocolVInit,
        ProtocolVoltagesNum
    };

    enum ProtocolTimes {
        ProtocolTHold,
        ProtocolTPulse,
        ProtocolTStep,
        ProtocolTRamp,
        ProtocolTPe,
        ProtocolTimesNum
    };

    enum ProtocolAdimensionals {
        ProtocolN,
        ProtocolNR,
        ProtocolAdimensionalsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;

private:
    enum VoltageRanges {
        VoltageRange700mV,
        VoltageRange2V,
        VoltageRangesNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange700mV,
        ProtocolVoltageRange2V,
        ProtocolVoltageRangesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };
};

class MessageDispatcher_eNPR_2Channels_V01 : public MessageDispatcher_eNPR {
public:
    MessageDispatcher_eNPR_2Channels_V01(string di);
    virtual ~MessageDispatcher_eNPR_2Channels_V01();

protected:
    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    void updateVoltageReferenceOffsetCalibration();

private:
    enum VoltageRanges {
        VoltageRange500mV,
        VoltageRangesNum
    };

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange500mV,
        ProtocolVoltageRangesNum
    };

    enum ProtocolFrequencyRanges {
        ProtocolFrequencyRange35Hz,
        ProtocolFrequencyRangesNum
    };

    enum ProtocolFrequencies {
        ProtocolFrequency,
        ProtocolFrequenciesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolSinusoid,
        ProtocolsNum
    };

    Measurement_t voltageReferenceOffsetCalibration = {0.0, UnitPfxNone, "V"};
};

class MessageDispatcher_eNPR_2Channels_V02 : public MessageDispatcher_eNPR_2Channels_V01 {
public:
    MessageDispatcher_eNPR_2Channels_V02(string di);
    virtual ~MessageDispatcher_eNPR_2Channels_V02();
};

class MessageDispatcher_eNPR_2Channels_V03 : public MessageDispatcher_eNPR_2Channels_V02 {
public:
    MessageDispatcher_eNPR_2Channels_V03(string di);
    virtual ~MessageDispatcher_eNPR_2Channels_V03();

protected:
    bool checkProtocolValidity(string &message) override;

private:
    enum VoltageRanges {
        VoltageRange500mV,
        VoltageRangesNum
    };

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange500mV,
        ProtocolVoltageRangesNum
    };

    enum ProtocolFrequencyRanges {
        ProtocolFrequencyRangesNum
    };

    enum ProtocolFrequencies {
        ProtocolFrequenciesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };
};

class MessageDispatcher_eNPR_FL_V02 : public MessageDispatcher_eNPR {
public:
    MessageDispatcher_eNPR_FL_V02(string di);
    virtual ~MessageDispatcher_eNPR_FL_V02();

protected:
    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };

    enum Leds {
        LedBlue,
        LedsNum
    };
};

class MessageDispatcher_eNPR_FL_V01 : public MessageDispatcher_eNPR {
public:
    MessageDispatcher_eNPR_FL_V01(string di);
    virtual ~MessageDispatcher_eNPR_FL_V01();

protected:
    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };

    enum Leds {
        LedBlue,
        LedsNum
    };

private:
    enum VoltageRanges {
        VoltageRange700mV,
        VoltageRangesNum
    };

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange700mV,
        ProtocolVoltageRangesNum
    };
};

class MessageDispatcher_eNPR_LegacyEdr3_V04 : public MessageDispatcherLegacyEdr3 {
public:
    MessageDispatcher_eNPR_LegacyEdr3_V04(string id);
    virtual ~MessageDispatcher_eNPR_LegacyEdr3_V04();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200pA,
        CurrentRange2nA,
        CurrentRange20nA,
        CurrentRange200nA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange700mV,
        VoltageRange2V,
        VoltageRangesNum
    };

    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRate200kHz,
        SamplingRatesNum
    };

    enum OveramplingRatios {
        OversamplingRatioX1,
        OversamplingRatioX4,
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf10kHz,
        VoltageStimulusLpfsNum
    };

    enum VoltageReferenceLpfs {
        VoltageReferenceLpf3Hz,
        VoltageReferenceLpf180kHz,
        VoltageReferenceLpfsNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange700mV,
        ProtocolVoltageRange2V,
        ProtocolVoltageRangesNum
    };

    enum ProtocolTimeRanges {
        ProtocolTimeRange2_10ms,
        ProtocolTimeRange0to2_28,
        ProtocolTimeRange1to2_28,
        ProtocolTimeRange1orMore,
        ProtocolTimeRangeSigned2_27,
        ProtocolTimeRange1to2_25,
        ProtocolTimeRangesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };

    enum ProtocolVoltages {
        ProtocolVHold,
        ProtocolVPulse,
        ProtocolVStep,
        ProtocolVPk,
        ProtocolVFinal,
        ProtocolVInit,
        ProtocolVoltagesNum
    };

    enum ProtocolTimes {
        ProtocolTHold,
        ProtocolTPulse,
        ProtocolTStep,
        ProtocolTRamp,
        ProtocolTPe,
        ProtocolTimesNum
    };

    enum ProtocolAdimensionals {
        ProtocolN,
        ProtocolNR,
        ProtocolAdimensionalsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

class MessageDispatcher_eNPR_LegacyEdr3_V03 : public MessageDispatcherLegacyEdr3 {
public:
    MessageDispatcher_eNPR_LegacyEdr3_V03(string id);
    virtual ~MessageDispatcher_eNPR_LegacyEdr3_V03();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200pA,
        CurrentRange2nA,
        CurrentRange20nA,
        CurrentRange200nA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange700mV,
        VoltageRange2V,
        VoltageRangesNum
    };

    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRate200kHz,
        SamplingRatesNum
    };

    enum OveramplingRatios {
        OversamplingRatioX1,
        OversamplingRatioX4,
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf10kHz,
        VoltageStimulusLpfsNum
    };

    enum VoltageReferenceLpfs {
        VoltageReferenceLpf3Hz,
        VoltageReferenceLpf180kHz,
        VoltageReferenceLpfsNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange700mV,
        ProtocolVoltageRange2V,
        ProtocolVoltageRangesNum
    };

    enum ProtocolTimeRanges {
        ProtocolTimeRange2_10ms,
        ProtocolTimeRange0to2_28,
        ProtocolTimeRange1to2_28,
        ProtocolTimeRange1orMore,
        ProtocolTimeRangeSigned2_27,
        ProtocolTimeRange1to2_25,
        ProtocolTimeRangesNum
    };

    enum ProtocolSlopeRanges {
        ProtocolSlopeRange2_10mVms,
        ProtocolSlopeRangesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolConductance,
        ProtocolVariableAmplitude,
        ProtocolVariableDuration,
        ProtocolRamp,
        ProtocolCyclicVoltammetry,
        ProtocolsNum
    };

    enum ProtocolVoltages {
        ProtocolVHold,
        ProtocolVPulse,
        ProtocolVStep,
        ProtocolVPk,
        ProtocolVMax,
        ProtocolVMin,
        ProtocolVoltagesNum
    };

    enum ProtocolTimes {
        ProtocolTHold,
        ProtocolTPulse,
        ProtocolTStep,
        ProtocolTPe,
        ProtocolTimesNum
    };

    enum ProtocolSlopes {
        ProtocolSlope,
        ProtocolSlopesNum
    };

    enum ProtocolAdimensionals {
        ProtocolN,
        ProtocolNR,
        ProtocolAdimensionalsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

#endif // MESSAGEDISPATCHER_ENPR_H
