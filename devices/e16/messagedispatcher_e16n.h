#ifndef MESSAGEDISPATCHER_E16N_H
#define MESSAGEDISPATCHER_E16N_H

#include "messagedispatcher.h"

#include <iostream>

using namespace std;

class MessageDispatcher_e16n : public MessageDispatcher {
public:
    MessageDispatcher_e16n(string di);
    virtual ~MessageDispatcher_e16n();

protected:
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

    enum ProtocolIntegers {
        ProtocolN,
        ProtocolNR,
        ProtocolIntegersNum
    };

    void initializeDevice() override;
};

#endif // MESSAGEDISPATCHER_E16N_H
