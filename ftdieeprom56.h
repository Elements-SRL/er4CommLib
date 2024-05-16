#ifndef FTDIEEPROM56_H
#define FTDIEEPROM56_H

#include "ftdieeprom.h"

#define E56_DEVICE_VER_ADDR 0x074
#define E56_FW_VER_ADDR 0x0075
#define E56_VC_OFFSET_ADDR 0x07B

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class FtdiEeprom56 : public FtdiEeprom {
public:
    FtdiEeprom56(std::string deviceId);
    ~FtdiEeprom56();

protected:
    /*! FtdiEeprom interface */
    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif


#endif // FTDIEEPROM56_H
