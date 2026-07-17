#include "messagedispatcher_e4n_fwv01.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_e4n_FWV01::MessageDispatcher_e4n_FWV01(string di) :
    MessageDispatcher_e4n_V01(di) {

    selectableDOCAutostopFlag = true;

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;

    boolConfig.initialByte = 6;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    digitalOffsetCompensationAutostopCoder = new BoolArrayCoder(boolConfig);
}

MessageDispatcher_e4n_FWV01::~MessageDispatcher_e4n_FWV01() {

}
