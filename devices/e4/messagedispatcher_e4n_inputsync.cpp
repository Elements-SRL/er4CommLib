#include "messagedispatcher_e4n_inputsync.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_e4n_InputSync::MessageDispatcher_e4n_InputSync(string di) :
    MessageDispatcher_e4n_V01(di) {

    digInSyncImplementedFlag = true;
    fwLoadedOverrideFlag = true;

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;

    /*! Device reset */
    boolConfig.initialByte = 2;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    deviceResetOverrideCoder = new BoolArrayCoder(boolConfig);
}

MessageDispatcher_e4n_InputSync::~MessageDispatcher_e4n_InputSync() {

}
