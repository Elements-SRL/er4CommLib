#ifndef MESSAGEDISPATCHER_E4N_INPUTSYNC_H
#define MESSAGEDISPATCHER_E4N_INPUTSYNC_H

#include "messagedispatcher_e4n.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e4n_SineInputSync : public MessageDispatcher_e4n_V01 {
public:
    MessageDispatcher_e4n_SineInputSync(std::string di);
    virtual ~MessageDispatcher_e4n_SineInputSync();

protected:
    bool checkProtocolValidity(std::string &message) override;
    void remapProtocolParameters() override;

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
        ProtocolSinPlusConstant,
        ProtocolSinPlusTriangular,
        ProtocolSinPlusSquareWave,
        ProtocolSinPlusConductance,
        ProtocolSinPlusVariableAmplitude,
        ProtocolSinPlusVariableDuration,
        ProtocolSinPlusRamp,
        ProtocolSinPlusCyclicVoltammetry,
        ProtocolsNum
    };

    enum ProtocolVoltages {
        ProtocolVHold,
        ProtocolVPulse,
        ProtocolVStep,
        ProtocolVPk,
        ProtocolVFinal,
        ProtocolVInit,
        ProtocolVSine,
        ProtocolVoltagesNum
    };

    enum ProtocolFrequencies {
        ProtocolFrequency,
        ProtocolFrequenciesNum
    };
};

class MessageDispatcher_e4n_SineInputSync_FwV02 : public MessageDispatcher_e4n_SineInputSync {
public:
    enum TriggerRanges {
        TriggerRange3_3V,
        TriggerRangesNum
    };

    enum GpChannels {
        GpChannelTrigger,
        GpChannelsNum
    };

    MessageDispatcher_e4n_SineInputSync_FwV02(std::string di);
    virtual ~MessageDispatcher_e4n_SineInputSync_FwV02();
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E4N_INPUTSYNC_H
