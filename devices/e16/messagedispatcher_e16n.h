#ifndef MESSAGEDISPATCHER_E16N_H
#define MESSAGEDISPATCHER_E16N_H

#include "messagedispatcher.h"

#include <iostream>

using namespace std;

class MessageDispatcher_e16n : public MessageDispatcher {
public:
    MessageDispatcher_e16n(string di);
    virtual ~MessageDispatcher_e16n();

    ErrorCodes_t resetWasherError() override;
    ErrorCodes_t setWasherPresetSpeeds(vector <int8_t> speedValues) override;
    ErrorCodes_t startWasher(uint16_t speedIdx) override;
    ErrorCodes_t updateWasherState() override;
    ErrorCodes_t updateWasherPresetSpeeds() override;

    ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature) override;
    ErrorCodes_t getWasherSpeedRange(RangedMeasurement_t &range) override;
    ErrorCodes_t getWasherStatus(WasherStatus_t &status, WasherError_t &error) override;
    ErrorCodes_t getWasherPresetSpeeds(vector <int8_t> &speedValue) override;

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

    enum ProtocolSlopes {
        ProtocolSlopesNum
    };

    enum ProtocolAdimensionals {
        ProtocolN,
        ProtocolNR,
        ProtocolAdimensionalsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    virtual void setFerdParameters() override;

    /*! Device specific controls */
    void updateWasherStatus();
    void updateWasherSpeeds();

    int minControllerTemperature = -10;
    int maxControllerTemperature = 60;

    InfoStruct_t infoStruct;
    RangedMeasurement_t washerSpeedRange;
    vector <int8_t> washerSpeeds;

    BoolArrayCoder * washerResetCoder;
    BoolArrayCoder * washerGetStatusCoder;
    BoolArrayCoder * washerGetSpeedsCoder;
    BoolArrayCoder * washerSetSpeedsCoder;
    BoolArrayCoder * washerStartCoder;
    BoolArrayCoder * washerSelectSpeedCoder;
    vector <DoubleTwosCompCoder *> washerPresetSpeedsCoders;
};

class MessageDispatcher_dlp : public MessageDispatcher_e16n {
public:
    MessageDispatcher_dlp(string di);
};

#endif // MESSAGEDISPATCHER_E16N_H
