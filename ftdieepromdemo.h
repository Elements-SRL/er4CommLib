#ifndef FTDIEEPROMDEMO_H
#define FTDIEEPROMDEMO_H

#include "ftdieeprom.h"

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class FtdiEepromDemo : public FtdiEeprom {
public:
    FtdiEepromDemo(std::string deviceId);
    ~FtdiEepromDemo();

    ErrorCodes_t openConnection(char channel = 'A') override;
    ErrorCodes_t closeConnection() override;

protected:
    /*! FtdiEeprom interface */
    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // FTDIEEPROMDEMO_H
