#ifndef MESSAGEDISPATCHER_E16HC_H
#define MESSAGEDISPATCHER_E16HC_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e16HC_V03 : public MessageDispatcher {
public:
    MessageDispatcher_e16HC_V03(std::string id);
    virtual ~MessageDispatcher_e16HC_V03();

    virtual ErrorCodes_t setGpRange(uint16_t gpRangeIdx, uint16_t channelIdx, bool applyFlag = true) override;
    virtual ErrorCodes_t getVoltageReferenceRanges(std::vector <RangedMeasurement_t> &ranges, uint16_t &defaultOption) override;

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

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRange15V,
        VoltageReferenceRangesNum
    };

    enum GpChannels {
        GpChannelVoltageReference,
        GpChannelsNum
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
    bool checkProtocolValidity(std::string &message) override;
    ErrorCodes_t updateVoltageOffsetCompensations(std::vector <Measurement_t> &offsets) override;
    void updateVoltageReferenceOffsetCalibration();

    /*! Device specific controls */
    InfoStruct_t infoStruct;

    Measurement_t voltageReferenceOffsetCalibration = {0.0, UnitPfxNone, "V"};
};

class MessageDispatcher_e16HC_V02 : public MessageDispatcher_e16HC_V03 {
public:
    MessageDispatcher_e16HC_V02(std::string id);
    virtual ~MessageDispatcher_e16HC_V02();

    ErrorCodes_t setGpRange(uint16_t gpRangeIdx, uint16_t channelIdx, bool applyFlag = true) override;
    virtual ErrorCodes_t getVoltageReferenceRanges(std::vector <RangedMeasurement_t> &ranges, uint16_t &defaultOption) override;
};

class MessageDispatcher_e16HC_V01 : public MessageDispatcher_e16HC_V02 {
public:
    MessageDispatcher_e16HC_V01(std::string id);
    virtual ~MessageDispatcher_e16HC_V01();

    enum VoltageReferenceRanges {
        VoltageReferenceRange2V,
        VoltageReferenceRangesNum
    };
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E16HC_H
