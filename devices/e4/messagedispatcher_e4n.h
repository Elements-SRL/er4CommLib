#ifndef MESSAGEDISPATCHER_E4N_H
#define MESSAGEDISPATCHER_E4N_H

#include "messagedispatcher.h"
#include "messagedispatcher_e4e.h"

class MessageDispatcher_e4n_V01 : public MessageDispatcher {
public:
    MessageDispatcher_e4n_V01(string di);
    virtual ~MessageDispatcher_e4n_V01();

    ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;

protected:
    typedef struct {
        int16_t offset[4];
    } InfoStruct_t;

    enum CurrentRanges {
        CurrentRange200pA,
        CurrentRange2nA,
        CurrentRange20nA,
        CurrentRange200nA,
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
        VoltageStimulusLpf100Hz,
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
    virtual void setFerdParameters() override;
    ErrorCodes_t updateVoltageOffsetCompensations(vector <Measurement_t> &offsets) override;

    /*! Device specific controls */
    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;

    InfoStruct_t infoStruct;

private:
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
};

class MessageDispatcher_e4n_sine_V01 : public MessageDispatcher_e4n_V01 {
public:
    MessageDispatcher_e4n_sine_V01(string di);
    virtual ~MessageDispatcher_e4n_sine_V01();

protected:
    bool checkProtocolValidity(string &message) override;

private:
    enum ProtocolFrequencyRanges {
        ProtocolFrequencyRange35Hz,
        ProtocolFrequencyRangesNum
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
        ProtocolSinusoid,
        ProtocolsNum
    };

    enum ProtocolFrequencies {
        ProtocolFrequency,
        ProtocolFrequenciesNum
    };
};

class MessageDispatcher_e4n_El03c_LegacyEdr3_V04 : public MessageDispatcherLegacyEdr3 {
public:
    MessageDispatcher_e4n_El03c_LegacyEdr3_V04(std::string id);
    virtual ~MessageDispatcher_e4n_El03c_LegacyEdr3_V04();

    ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;

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
        VoltageStimulusLpf100Hz,
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

    enum ProtocolSlopeRanges {
        ProtocolSlopeRange2_10mVms,
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
    bool checkProtocolValidity(string &message) override;

    /*! Device specific controls */
    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;

    InfoStruct_t infoStruct;
};

class MessageDispatcher_e4n_El03c_LegacyEdr3_V05 : public MessageDispatcher_e4e_El03c_LegacyEdr3_V05 {
public:
    MessageDispatcher_e4n_El03c_LegacyEdr3_V05(std::string id);
    ~MessageDispatcher_e4n_El03c_LegacyEdr3_V05();

    ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;

protected:
    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;
};

#endif // MESSAGEDISPATCHER_E4N_H
