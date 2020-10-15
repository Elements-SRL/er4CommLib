#ifndef FTDIEEPROM56_H
#define FTDIEEPROM56_H

#include "ftdieeprom.h"

#define E56_DEVICE_VER_ADDR 0x074
#define E56_FW_VER_ADDR 0x0075
#define E56_VC_OFFSET_ADDR 0x07B

class FtdiEeprom56 : public FtdiEeprom {
public:
    FtdiEeprom56(string deviceId);
    ~FtdiEeprom56();

protected:
    /*! FtdiEeprom interface */
    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#endif // FTDIEEPROM56_H
