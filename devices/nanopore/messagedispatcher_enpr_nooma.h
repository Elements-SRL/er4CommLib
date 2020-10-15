#ifndef MESSAGEDISPATCHER_ENPR_NOOMA_H
#define MESSAGEDISPATCHER_ENPR_NOOMA_H

#include "messagedispatcher.h"

#include <iostream>

using namespace std;

class MessageDispatcher_eNPR_Nooma : public MessageDispatcher {
public:
    MessageDispatcher_eNPR_Nooma(string di);
    virtual ~MessageDispatcher_eNPR_Nooma();

protected:
    enum CurrentRanges {
        CurrentRange200nA,
        CurrentRange4uA,
        CurrentRangesNum
    };

    enum VoltageRanges {
        VoltageRange800mV,
        VoltageRange500mV,
        VoltageRange15V,
        VoltageRangesNum
    };

    enum SamplingRates {
        SamplingRate200kHz,
        SamplingRate312_5kHz,
        SamplingRate320kHz,
        SamplingRatesNum
    };

    enum Protocols {
        ProtocolConstant,
        ProtocolTriangular,
        ProtocolSquareWave,
        ProtocolsNum
    };

    enum ProtocolVoltages {
        ProtocolVHold,
        ProtocolVPulse,
        ProtocolVoltagesNum
    };

    enum ProtocolTimes {
        ProtocolTHold,
        ProtocolTPulse,
        ProtocolTimesNum
    };

    enum FsmFlags {
        FsmRun,
        FsmEnhance,
        FsmEnableReScan,
        FsmEnableP2Exit,
        FsmFlagsNum
    };

    enum FsmVoltages {
        FsmV1PreCapture,
        FsmV1L2R,
        FsmV1R2L,
        FsmV1PrePause,
        FsmV1Capture,
        FsmV1Pause,
        FsmV2Reverse,
        FsmV2Capture,
        FsmV12Push,
        FsmV12Pull,
        FsmVoltagesNum
    };

    enum FsmCurrents {
        FsmP1PreCaptureThreshold,
        FsmP1CaptureThreshold,
        FsmP2CaptureThreshold,
        FsmP2TagThreshold,
        FsmP2TagThresholdHigh,
        FsmP2ExitThreshold,
        FsmP2FoldThreshold,
        FsmPullCaptureThreshold,
        FsmCurrentsNum
    };

    enum FsmTimes {
        FsmTransientSettleTime,
        FsmTofHoldTime,
        FsmPauseFilterResetTime,
        FsmPreCaptureLowLimitTime,
        FsmPreCaptureHighLimitTime,
        FsmL2RCalibrationTime,
        FsmR2LCalibrationTime,
        FsmPrePauseWaitTime,
        FsmP1CaptureTimeout,
        FsmP1CaptureWaitTime,
        FsmP2CaptureTimeout,
        FsmP2CaptureWaitTime,
        FsmP2ReverseTimeout,
        FsmP2ExitWaitTime,
        FsmTagDurationMaxTime,
        FsmTagDurationMinTime,
        FsmTagWaitTime,
        FsmTagHoldTime,
        FsmBaselineSettleTime,
        FsmPushTime,
        FsmPullTimeout,
        FsmPullInDelayTime,
        FsmRecaptureTimeout,
        FsmTimesNum
    };

    enum FsmIntegers {
        FsmInitialTagMax,
        FsmTagCountStop,
        FsmScanCountMax,
        FsmRecaptureTagCount,
        FsmIntegersNum
    };

    enum ConditioningVoltages {
        ConditioningVChargePos,
        ConditioningVChargeNeg,
        ConditioningVObservePos,
        ConditioningVObserveNeg,
        ConditioningVoltagesNum
    };

    enum CheckingVoltages {
        CheckingVHold,
        CheckingVoltagesNum
    };

    enum ConditioningTimes {
        ConditioningTCharge,
        ConditioningTObserve,
        ConditioningTimesNum
    };

    void initializeDevice() override;
};

#endif // MESSAGEDISPATCHER_ENPR_NOOMA_H
