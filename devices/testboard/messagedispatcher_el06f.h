#ifndef MESSAGEDISPATCHER_EL06F_H
#define MESSAGEDISPATCHER_EL06F_H

#include "messagedispatcher_el06d_el06e.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_EL06f : public MessageDispatcher_EL06d_EL06e {
public:
    MessageDispatcher_EL06f(std::string di);
    virtual ~MessageDispatcher_EL06f();

    enum SamplingRates {
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRate40kHz,
        SamplingRate80kHz,
        SamplingRate160kHz,
        SamplingRatesNum
    };

    enum CustomOptions {
        CustomOptionClockDivider,
        CustomOptionsNum
    };
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_EL06F_H
