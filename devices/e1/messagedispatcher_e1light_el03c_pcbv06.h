#ifndef MESSAGEDISPATCHER_E1LIGHT_EL_3C_PCBV_6_H
#define MESSAGEDISPATCHER_E1LIGHT_EL_3C_PCBV_6_H

#include "messagedispatcher.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

    class MessageDispatcher_e1Light_EL03c_PCBV06 : public MessageDispatcher {
    public:
        MessageDispatcher_e1Light_EL03c_PCBV06(std::string di);
        virtual ~MessageDispatcher_e1Light_EL03c_PCBV06();

    protected:
        typedef struct {
            int16_t offset;
        } InfoStruct_t;

        enum CurrentRanges {
            CurrentRange200pA,
            CurrentRange20nA,
            CurrentRangesNum
        };

        enum VoltageRanges {
            VoltageRange500mV,
            VoltageRange50mV,
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
        ErrorCodes_t updateVoltageOffsetCompensations(std::vector <Measurement_t> &offsets) override;

        /*! Device specific controls */
        InfoStruct_t infoStruct;
    };

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E1LIGHT_EL_3C_PCBV_6_H
