#ifndef MESSAGEDISPATCHER_EL_8B_TB_H
#define MESSAGEDISPATCHER_EL_8B_TB_H

#include "messagedispatcher.h"

#include "commandcoder.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_EL08b_TB : public MessageDispatcher {
public:
    MessageDispatcher_EL08b_TB(std::string di);
    virtual ~MessageDispatcher_EL08b_TB();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange1nA,
        CurrentRange10nA,
        CurrentRange100nA,
        CurrentRange1000nA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange500mV,
        VoltageRangesNum
    };

    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate2_5kHz,
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
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf26Hz,
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf5kHz,
        VoltageStimulusLpf20kHz,
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

    enum CustomOptions {
        CustomOptionZapLevel,
        CustomOptionsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(std::string &message) override;

    InfoStruct_t infoStruct;
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_EL_8B_TB_H
