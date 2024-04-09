#ifndef FTDIEEPROMDEMO_H
#define FTDIEEPROMDEMO_H

#include "ftdieeprom.h"

class FtdiEepromDemo : public FtdiEeprom {
public:
    FtdiEepromDemo(string deviceId);
    ~FtdiEepromDemo();

    ErrorCodes_t openConnection(char channel = 'A') override;
    ErrorCodes_t closeConnection() override;

protected:
    /*! FtdiEeprom interface */
    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#endif // FTDIEEPROMDEMO_H
