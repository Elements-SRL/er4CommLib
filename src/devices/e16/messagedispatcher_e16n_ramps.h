#ifndef MESSAGEDISPATCHER_E16N_RAMPS_H
#define MESSAGEDISPATCHER_E16N_RAMPS_H

#include "messagedispatcher_e16n.h"

#include "commandcoder.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e16n_ramps_V01 : public MessageDispatcher_e16n_V01 {
public:
    MessageDispatcher_e16n_ramps_V01(std::string di);
    virtual ~MessageDispatcher_e16n_ramps_V01();
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E16N_RAMPS_H
