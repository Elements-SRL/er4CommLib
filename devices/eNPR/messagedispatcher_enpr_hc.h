#ifndef MESSAGEDISPATCHER_ENPR_HC_H
#define MESSAGEDISPATCHER_ENPR_HC_H

#include "messagedispatcher.h"

class MessageDispatcher_eNPR_HC_V01 : public MessageDispatcher {
public:
    MessageDispatcher_eNPR_HC_V01(std::string di);
    virtual ~MessageDispatcher_eNPR_HC_V01();

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
        VoltageRange700mV,
        VoltageRange2V,
        VoltageRangesNum
    };

    enum SamplingRates {
        SamplingRate1_5kHz,
        SamplingRate6_25kHz,
        SamplingRate12_5kHz,
        SamplingRate25kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRatesNum
    };

    enum OveramplingRatios {
        OversamplingRatioX1,
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpfsNum = 0
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
    bool checkProtocolValidity(std::string &message) override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

class MessageDispatcher_eNPR_HC_V02 : public MessageDispatcher_eNPR_HC_V01 {
public:
    MessageDispatcher_eNPR_HC_V02(std::string di);
    virtual ~MessageDispatcher_eNPR_HC_V02();

protected:
    enum SamplingRates {
        SamplingRate1_5kHz,
        SamplingRate6_25kHz,
        SamplingRate12_5kHz,
        SamplingRate25kHz,
        SamplingRate50kHz,
        SamplingRate100kHz,
        SamplingRate200kHz,
        SamplingRatesNum
    };
};

#endif // MESSAGEDISPATCHER_ENPR_HC_H
