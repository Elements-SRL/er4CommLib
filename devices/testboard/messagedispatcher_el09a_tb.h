#ifndef MESSAGEDISPATCHER_EL_9A_TB_H
#define MESSAGEDISPATCHER_EL_9A_TB_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_EL09a_TB : public MessageDispatcher {
public:
    MessageDispatcher_EL09a_TB(std::string di);
    virtual ~MessageDispatcher_EL09a_TB();

protected:
    typedef struct {
        uint16_t temperature;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange20pA_preamp,
        CurrentRange200pA_preamp,
        CurrentRange2nA_preamp,
        CurrentRange20nA_preamp,
        CurrentRange20nA,
        CurrentRange200nA,
        CurrentRange2uA,
        CurrentRange20uA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange500mV,
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
        OversamplingRatiosNum
    };

    enum VoltageStimulusLpfs {
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf2_5kHz,
        VoltageStimulusLpf10kHz,
        VoltageStimulusLpf25kHz,
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

    enum CustomFlags {
        CustomFlagDac0Cap,
        CustomFlagsNum
    };

    enum CustomOptions {
        CustomOptionDacWriteSelection,
        CustomOptionDacApplySelection,
        CustomOptionClockDivider,
        CustomOptionsNum
    };

    enum CustomDoubles {
        CustomDoubleRShuntCorr,
        CustomDoubleOffset,
        CustomDoublesNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(std::string &message) override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_EL_9A_TB_H
