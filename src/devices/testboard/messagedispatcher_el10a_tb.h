// #ifndef MESSAGEDISPATCHER_EL1_A_TB_H
// #define MESSAGEDISPATCHER_EL1_A_TB_H

// #include "messagedispatcher.h"

// #ifndef ER4COMMLIB_LABVIEW_WRAPPER
// namespace er4CommLib {
// #endif

// class MessageDispatcher_EL10a_TB : public MessageDispatcher {
// public:
//     MessageDispatcher_EL10a_TB(std::string di);
//     virtual ~MessageDispatcher_EL10a_TB();

// protected:
//     typedef struct {
//         uint8_t unused;
//     } InfoStruct_t;

//     enum CurrentRanges {
//         CurrentRange25nA,
//         CurrentRange50nA,
//         CurrentRange100nA,
//         CurrentRange200nA,
//         CurrentRangesNum
//     };

//     enum VoltageRanges {
//         VoltageRange1650mV,
//         VoltageRangesNum
//     };

//     enum SamplingRates {
//         SamplingRate1_25kHz,
//         SamplingRate2_5kHz,
//         SamplingRate5kHz,
//         SamplingRate10kHz,
//         SamplingRate20kHz,
//         SamplingRate50kHz,
//         SamplingRate100kHz,
//         SamplingRate200kHz,
//         SamplingRatesNum
//     };

//     enum OveramplingRatios {
//         OversamplingRatioX1,
//         OversamplingRatiosNum
//     };

//     enum VoltageStimulusLpfs {
//         VoltageStimulusLpfsNum = 0
//     };

//     enum VoltageReferenceLpfs {
//         VoltageReferenceLpfsNum = 0
//     };

//     enum ProtocolVoltageRanges {
//         ProtocolVoltageRange1650mV,
//         ProtocolVoltageRangesNum
//     };

//     enum ProtocolTimeRanges {
//         ProtocolTimeRange2_10ms,
//         ProtocolTimeRange0to2_28,
//         ProtocolTimeRange1to2_28,
//         ProtocolTimeRange1orMore,
//         ProtocolTimeRangeSigned2_27,
//         ProtocolTimeRange1to2_25,
//         ProtocolTimeRangesNum
//     };

//     enum Protocols {
//         ProtocolConstant,
//         ProtocolTriangular,
//         ProtocolSquareWave,
//         ProtocolConductance,
//         ProtocolVariableAmplitude,
//         ProtocolVariableDuration,
//         ProtocolRamp,
//         ProtocolCyclicVoltammetry,
//         ProtocolsNum
//     };

//     enum ProtocolVoltages {
//         ProtocolVHold,
//         ProtocolVPulse,
//         ProtocolVStep,
//         ProtocolVPk,
//         ProtocolVFinal,
//         ProtocolVInit,
//         ProtocolVoltagesNum
//     };

//     enum ProtocolTimes {
//         ProtocolTHold,
//         ProtocolTPulse,
//         ProtocolTStep,
//         ProtocolTRamp,
//         ProtocolTPe,
//         ProtocolTimesNum
//     };

//     enum ProtocolAdimensionals {
//         ProtocolN,
//         ProtocolNR,
//         ProtocolAdimensionalsNum
//     };

//     enum CustomFlags {
//         CustomFlagDac0Cap,
//         CustomFlagsNum
//     };

//     enum CustomOptions {
//         CustomOptionDacWriteSelection,
//         CustomOptionDacApplySelection,
//         CustomOptionClockDivider,
//         CustomOptionsNum
//     };

//     enum CustomDoubles {
//         CustomDoubleRShuntCorr,
//         CustomDoubleOffset,
//         CustomDoublesNum
//     };

//     void initializeDevice() override;
//     bool checkProtocolValidity(std::string &message) override;

//     /*! Device specific controls */
//     InfoStruct_t infoStruct;
// };

// #ifndef ER4COMMLIB_LABVIEW_WRAPPER
// };
// #endif

// #endif // MESSAGEDISPATCHER_EL1_A_TB_H
