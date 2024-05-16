#ifndef MESSAGEDISPATCHER_EL06D_EL06E_H
#define MESSAGEDISPATCHER_EL06D_EL06E_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_EL06d_EL06e : public MessageDispatcher {
public:
    MessageDispatcher_EL06d_EL06e(std::string di);
    virtual ~MessageDispatcher_EL06d_EL06e();

    ErrorCodes_t setProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false) override;

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

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
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

private:
    enum SamplingRates {
        SamplingRate100kHz,
        SamplingRatesNum
    };
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_EL06D_EL06E_H
