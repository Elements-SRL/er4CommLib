#ifndef FTDIEEPROM_H
#define FTDIEEPROM_H

#include <string>

#ifdef _WIN32 /*! _WIN32 isolates both 32 and 64 bit windows systems, _WIN64 isolates only 64 */
#include <windows.h>
#include "ftd2xx_win.h"

#elif __APPLE__ /*! __APPLE__ isolates macOS X systems */
/*! libftdi must be properly set in system folders (/usr/local/lib ... ecc) */
#include "ftd2xx.h"
#endif

#include "er4commlib.h"

typedef enum {
    FtdiEepromId56,
    FtdiEepromIdDemo
} FtdiEepromId_t;

typedef enum {
    DeviceVersionE16 = 0x03,
    DeviceVersionE4 = 0x04,
    DeviceVersionDlp = 0x06,
    DeviceVersionDemo = 0xFD,
    DeviceVersionPrototype = 0xFE,
    DeviceVersionUndefined = 0xFF
} DeviceVersion_t;

typedef enum {
    /*! Subversions used for ver = 03 */
    DeviceSubversionE16n = 5,

    /*! Subversions used for ver = 04 */
    DeviceSubversionE4e = 8,

    /*! Subversions used for ver = 06 */
    DeviceSubversionDlp = 3,

    /*! Subversions used for ver = FD */
    DeviceSubversionDemo = 1,

    /*! Subversions used for ver = FE */

    /*! Subversions used for ver = 0xFF */
    DeviceSubversionUndefined = 0xFF
} DeviceSubversion_t ;

typedef struct {
    DeviceVersion_t version = DeviceVersionUndefined;
    DeviceSubversion_t subversion = DeviceSubversionUndefined;
    uint32_t fwVersion = 0;
} DeviceTuple_t;

typedef enum {
    DateDayIdx = 0,
    DateMonthIdx = 1,
    DateYearIdx = 2,
} DateIndices_t;

inline bool operator == (const DeviceTuple_t &a, const DeviceTuple_t &b) {
    return ((a.version == b.version) && (a.subversion == b.subversion) && (a.fwVersion == b.fwVersion));
}

inline bool operator != (const DeviceTuple_t &a, const DeviceTuple_t &b) {
    return !(a == b);
}

using namespace std;
using namespace er4CommLib;

class FtdiEeprom {
public:
    FtdiEeprom(string deviceId);
    virtual ~FtdiEeprom();

    virtual ErrorCodes_t openConnection(char channel = 'A');
    virtual ErrorCodes_t closeConnection();
    DeviceTuple_t getDeviceTuple();
    uint16_t getVcOffset();
    ErrorCodes_t readEepromWord(DWORD address, LPWORD result);

protected:
    string deviceId;
    char communicationChannel;
    string communicationSerial;
    DeviceTuple_t deviceTuple;
    uint16_t vcOffset;
    bool connectionOpened = false;
    FT_HANDLE handler;

    virtual ErrorCodes_t loadData() = 0;
    virtual ErrorCodes_t loadDeviceTuple() = 0;
    virtual ErrorCodes_t loadVcOffset() = 0;
};

#endif // FTDIEEPROM_H
