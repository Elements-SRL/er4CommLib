#ifndef MESSAGEDISPATCHER_E1B_EL_3C_LEGACYEDR3_PCBV_2_H
#define MESSAGEDISPATCHER_E1B_EL_3C_LEGACYEDR3_PCBV_2_H

#include "messagedispatcher.h"

#include "commandcoder.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02 : public MessageDispatcherLegacyEdr3 {
public:
    MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02(std::string id);
    virtual ~MessageDispatcher_e1b_El03c_LegacyEdr3_PCBV02_FWV02();

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
        VoltageRange2000mV,
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
        VoltageStimulusLpfsNum = 0
    };

    enum VoltageReferenceLpfs {
        VoltageReferenceLpf3Hz,
        VoltageReferenceLpf180kHz,
        VoltageReferenceLpfsNum
    };

    enum ProtocolVoltageRanges {
        ProtocolVoltageRange500mV,
        ProtocolVoltageRange1650mV,
        ProtocolVoltageRange2000mV,
        ProtocolVoltageRangesNum
    };

    enum ProtocolTimeRanges {
        ProtocolTimeRange1_1000ms,
        ProtocolTimeRange0to2_20,
        ProtocolTimeRange1to2_20,
        ProtocolTimeRange1orMore,
        ProtocolTimeRangeSigned2_20,
        ProtocolTimeRangesNum
    };

    enum ProtocolSlopeRanges {
        ProtocolSlopeRange1_1000mVms,
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
    bool checkProtocolValidity(std::string &message) override;
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E1B_EL_3C_LEGACYEDR3_PCBV_2_H
