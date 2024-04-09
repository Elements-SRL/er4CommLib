#ifndef MESSAGEDISPATCHER_E2HC_H
#define MESSAGEDISPATCHER_E2HC_H

#include "messagedispatcher.h"

using namespace std;

class MessageDispatcher_e2HC_V00 : public MessageDispatcher {
public:
    MessageDispatcher_e2HC_V00(string di);
    virtual ~MessageDispatcher_e2HC_V00();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200nA,
        CurrentRange4uA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange500mV,
        VoltageRangesNum
    };

    enum OveramplingRatios {
        OversamplingRatioX1,
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf10kHz,
        VoltageStimulusLpfsNum
    };

    enum VoltageReferenceLpfs {
        VoltageReferenceLpfsNum = 0
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange500mV,
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

    /*! Device specific controls */
    InfoStruct_t infoStruct;

    const double stimulusVoltageLimit = 0.5; /*! max voltage set for stimuli [V] */
    const double stimulusVoltageReference = 1.1; /*! voltage reference for stimuli [V] */

private:
    enum SamplingRates {
        SamplingRate62_5kHz,
        SamplingRate250kHz,
        SamplingRatesNum
    };
};

class MessageDispatcher_e2HC_V01 : public MessageDispatcher_e2HC_V00 {
public:
    MessageDispatcher_e2HC_V01(string di);
    virtual ~MessageDispatcher_e2HC_V01();

private:
    enum SamplingRates {
        SamplingRate50kHz,
        SamplingRate25kHz,
        SamplingRate12_5kHz,
        SamplingRatesNum
    };
};

#endif // MESSAGEDISPATCHER_E2HC_H
