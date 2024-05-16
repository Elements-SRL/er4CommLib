#ifndef MESSAGEDISPATCHER_E1HC_H
#define MESSAGEDISPATCHER_E1HC_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e1Hc_El03f_LegacyEdr3_V00 : public MessageDispatcherLegacyEdr3 {

public:
    MessageDispatcher_e1Hc_El03f_LegacyEdr3_V00(std::string id);
    virtual ~MessageDispatcher_e1Hc_El03f_LegacyEdr3_V00();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange2uA,
        CurrentRange20uA,
        CurrentRange200uA,
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
        ProtocolVoltageRange2000mV,
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
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

class MessageDispatcher_e1Hc_El03c_LegacyEdr3_V00 : public MessageDispatcher_e1Hc_El03f_LegacyEdr3_V00 {
public:
    MessageDispatcher_e1Hc_El03c_LegacyEdr3_V00(std::string id);
    ~MessageDispatcher_e1Hc_El03c_LegacyEdr3_V00();
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E1HC_H
