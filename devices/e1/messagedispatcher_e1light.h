#ifndef MESSAGEDISPATCHER_E1LIGHT_H
#define MESSAGEDISPATCHER_E1LIGHT_H

#include "messagedispatcher.h"

class MessageDispatcher_e1Light_El03f_LegacyEdr3_V01 : public MessageDispatcherLegacyEdr3 {
public:
    MessageDispatcher_e1Light_El03f_LegacyEdr3_V01(std::string id);
    virtual ~MessageDispatcher_e1Light_El03f_LegacyEdr3_V01();

protected:
    typedef struct {
        uint8_t unused;
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200pA,
        CurrentRange20nA,
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
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

class MessageDispatcher_e1Light_El03c_LegacyEdr3_V01 : public MessageDispatcher_e1Light_El03f_LegacyEdr3_V01 {
public:
    MessageDispatcher_e1Light_El03c_LegacyEdr3_V01(std::string id);
    ~MessageDispatcher_e1Light_El03c_LegacyEdr3_V01();
};
#endif // MESSAGEDISPATCHER_E1LIGHT_H
