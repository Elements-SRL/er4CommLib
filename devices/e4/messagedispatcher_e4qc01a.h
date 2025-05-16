#ifndef MESSAGEDISPATCHER_E4QC01A_H
#define MESSAGEDISPATCHER_E4QC01A_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e4qc01a_V01 : public MessageDispatcher {
public:
    MessageDispatcher_e4qc01a_V01(std::string di);

    ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;

protected:
    typedef struct {
        int16_t offset[4];
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange312_5pA,
        CurrentRange1_25nA,
        CurrentRange10nA,
        CurrentRange100nA,
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
        VoltageStimulusLpf1kHz,
        VoltageStimulusLpf10kHz,
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

    void initializeDevice() override;
    bool checkProtocolValidity(std::string &message) override;
    ErrorCodes_t updateVoltageOffsetCompensations(std::vector <Measurement_t> &offsets) override;

    /*! Device specific controls */
    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;

    /*! Device specific controls */
    InfoStruct_t infoStruct;
};

class MessageDispatcher_e4qc01a_V02 : public MessageDispatcher_e4qc01a_V01 {
public:
    MessageDispatcher_e4qc01a_V02(std::string di);

protected:
    enum CurrentRanges {
        CurrentRange5nA,
        CurrentRangesNum
    };
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E4QC01A_H
