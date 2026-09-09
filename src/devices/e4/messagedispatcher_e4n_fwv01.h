#ifndef MESSAGEDISPATCHER_E4N_FWV01_H
#define MESSAGEDISPATCHER_E4N_FWV01_H

#include "messagedispatcher_e4n.h"

#include "commandcoder.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class MessageDispatcher_e4n_FWV01 : public MessageDispatcher_e4n_V01 {
public:
    MessageDispatcher_e4n_FWV01(std::string di);
    virtual ~MessageDispatcher_e4n_FWV01();
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // MESSAGEDISPATCHER_E4N_FWV01_H
