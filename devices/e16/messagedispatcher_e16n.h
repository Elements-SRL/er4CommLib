#ifndef MESSAGEDISPATCHER_E16N_H
#define MESSAGEDISPATCHER_E16N_H

#include "messagedispatcher.h"

class MessageDispatcher_e16n_V01 : public MessageDispatcher {
public:
    MessageDispatcher_e16n_V01(std::string di);
    virtual ~MessageDispatcher_e16n_V01();

    er4cl::ErrorCodes_t resetWasherError() override;
    er4cl::ErrorCodes_t setWasherPresetSpeeds(std::vector <int8_t> speedValues) override;
    er4cl::ErrorCodes_t startWasher(uint16_t speedIdx) override;
    er4cl::ErrorCodes_t updateWasherState() override;
    er4cl::ErrorCodes_t updateWasherPresetSpeeds() override;

    er4cl::ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;
    er4cl::ErrorCodes_t getWasherSpeedRange(er4cl::RangedMeasurement_t &range) override;
    er4cl::ErrorCodes_t getWasherStatus(er4cl::WasherStatus_t &status, er4cl::WasherError_t &error) override;
    er4cl::ErrorCodes_t getWasherPresetSpeeds(std::vector <int8_t> &speedValue) override;

protected:
    enum {
        WasherSpeedsNum = 4
    };

    typedef struct {
        uint8_t state;
        int8_t presetSpeeds[WasherSpeedsNum];
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
//        ProtocolVExt,
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
    void updateWasherStatus();
    void updateWasherSpeeds();

    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;

    InfoStruct_t infoStruct;
    er4cl::RangedMeasurement_t washerSpeedRange;
    std::vector <int8_t> washerSpeeds;

    BoolArrayCoder * washerResetCoder;
    BoolArrayCoder * washerGetStatusCoder;
    BoolArrayCoder * washerGetSpeedsCoder;
    BoolArrayCoder * washerSetSpeedsCoder;
    BoolArrayCoder * washerStartCoder;
    BoolArrayCoder * washerSelectSpeedCoder;
    std::vector <DoubleTwosCompCoder *> washerPresetSpeedsCoders;
};

class MessageDispatcher_e16n_sine_V01 : public MessageDispatcher_e16n_V01 {
public:
    MessageDispatcher_e16n_sine_V01(std::string di);
    virtual ~MessageDispatcher_e16n_sine_V01();

protected:
    bool checkProtocolValidity(std::string &message) override;

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


class MessageDispatcher_dlp : public MessageDispatcher_e16n_V01 {
public:
    MessageDispatcher_dlp(std::string di);
};

#endif // MESSAGEDISPATCHER_E16N_H
