#ifndef MESSAGEDISPATCHER_E4N_INPUTSYNC_H
#define MESSAGEDISPATCHER_E4N_INPUTSYNC_H

#include "messagedispatcher_e4n.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e4n_InputSync : public MessageDispatcher_e4n_V01 {
public:
    MessageDispatcher_e4n_InputSync(std::string di);
    virtual ~MessageDispatcher_e4n_InputSync();
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E4N_INPUTSYNC_H
