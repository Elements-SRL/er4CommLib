#ifndef FTDIEEPROMDEMO_H
#define FTDIEEPROMDEMO_H

#include "ftdieeprom.h"

class FtdiEepromDemo : public FtdiEeprom {
public:
    FtdiEepromDemo(std::string deviceId);
    ~FtdiEepromDemo();

    er4cl::ErrorCodes_t openConnection(char channel = 'A') override;
    er4cl::ErrorCodes_t closeConnection() override;

protected:
    /*! FtdiEeprom interface */
    er4cl::ErrorCodes_t loadData() override;
    er4cl::ErrorCodes_t loadDeviceTuple() override;
    er4cl::ErrorCodes_t loadVcOffset() override;
};

#endif // FTDIEEPROMDEMO_H
