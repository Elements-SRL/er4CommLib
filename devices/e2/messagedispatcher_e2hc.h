#ifndef MESSAGEDISPATCHER_E2HC_H
#define MESSAGEDISPATCHER_E2HC_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e2HC_V00 : public MessageDispatcher {
public:
    MessageDispatcher_e2HC_V00(std::string di);

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

    enum SamplingRates {
        SamplingRate62_5kHz,
        SamplingRate250kHz,
        SamplingRatesNum
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
    bool checkProtocolValidity(std::string &message) override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;

    const double stimulusVoltageLimit = 0.5; /*! max voltage set for stimuli [V] */
    const double stimulusVoltageReference = 1.1; /*! voltage reference for stimuli [V] */
};

class MessageDispatcher_e2HC_V01 : public MessageDispatcher_e2HC_V00 {
public:
    MessageDispatcher_e2HC_V01(std::string di);

protected:
    enum SamplingRates {
        SamplingRate50kHz,
        SamplingRate25kHz,
        SamplingRate12_5kHz,
        SamplingRatesNum
    };
};

class MessageDispatcher_e2HC_V02 : public MessageDispatcher_e2HC_V00 {
public:
    MessageDispatcher_e2HC_V02(std::string di);

protected:
    enum SamplingRates {
        SamplingRate12_5kHz,
        SamplingRate25kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRate200kHz,
        SamplingRatesNum
    };
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E2HC_H
