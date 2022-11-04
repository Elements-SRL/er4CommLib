#ifndef MESSAGEDISPATCHER_E16HC_H
#define MESSAGEDISPATCHER_E16HC_H

#include "messagedispatcher.h"

class MessageDispatcher_e16HC_V02 : public MessageDispatcher {
public:
    MessageDispatcher_e16HC_V02(string id);
    virtual ~MessageDispatcher_e16HC_V02();

protected:
    typedef struct {
        int16_t offset[16];
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

    enum ProtocolSlopes {
        ProtocolSlopesNum = 0
    };

    enum ProtocolAdimensionals {
        ProtocolN,
        ProtocolNR,
        ProtocolAdimensionalsNum
    };

    enum Leds {
        LedGreen,
        LedsNum
    };

    void initializeDevice() override;
    bool checkProtocolValidity(string &message) override;
    ErrorCodes_t updateVoltageOffsetCompensations(vector <Measurement_t> &offsets) override;
    void updateVoltageReferenceOffsetCalibration();

    /*! Device specific controls */
    InfoStruct_t infoStruct;

    const double stimulusVoltageLimit = 0.5; /*! max voltage set for stimuli [V] */
    const double stimulusVoltageReference = 1.1; /*! voltage reference for stimuli [V] */

private:
    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRange15V,
        VoltageReferenceRangesNum
    };

    Measurement_t voltageReferenceOffsetCalibration = {0.0, UnitPfxNone, "V"}; /*! \todo FCON aggiungere metodo per leggere il valore dalla eeprom FTDI */
};

class MessageDispatcher_e16HC_V01 : public MessageDispatcher_e16HC_V02 {
public:
    MessageDispatcher_e16HC_V01(string id);
    virtual ~MessageDispatcher_e16HC_V01();

private:
    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
    };
};

#endif // MESSAGEDISPATCHER_E16HC_H
