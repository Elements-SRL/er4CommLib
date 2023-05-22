//  Copyright (C) 2021 Filippo Cona
//
//  This file is part of EDR4.
//
//  EDR4 is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  EDR4 is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with EDR4.  If not, see <http://www.gnu.org/licenses/>.

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
    DeviceVersionENPR = 0x08,
    DeviceVersionE1 = 0x09,
    DeviceVersionE2 = 0x0B,
    DeviceVersionDlp = 0x06,
    DeviceVersionDemo = 0xFD,
    DeviceVersionPrototype = 0xFE,
    DeviceVersionUndefined = 0xFF
} DeviceVersion_t;

typedef enum {
    /*! Subversions used for ver = 03 */
    DeviceSubversionE16FastPulses = 4,
    DeviceSubversionE16n = 5,
    DeviceSubversionE16e = 8,
    DeviceSubversionE16eth = 9,
    DeviceSubversionE16HC = 10,

    /*! Subversions used for ver = 04 */
    DeviceSubversionE4n = 3,
    DeviceSubversionE4e = 8,

    /*! Subversions used for ver = 08 */
    DeviceSubversionENPR = 2,
    DeviceSubversionENPRHC = 8,

    /*! Subversions used for ver = 09 */
    DeviceSubversionE1bEL03C = 2,
    DeviceSubversionE1LightEL03C = 4,
    DeviceSubversionE1PlusEL03C = 5,
    DeviceSubversionE1HcEL03C = 6,
    DeviceSubversionE1LightEL03F = 7,
    DeviceSubversionE1PlusEL03F = 8,
    DeviceSubversionE1HcEL03F = 9,

    /*! Subversions used for ver = 0B */
    DeviceSubversionE2HC = 1,

    /*! Subversions used for ver = 06 */
    DeviceSubversionDlp = 3,
    DeviceSubversionEL06b = 5,
    DeviceSubversionEL06c = 6,
    DeviceSubversionEL06d = 7,
    DeviceSubversionEL06e = 8,

    /*! Subversions used for ver = FD */
    DeviceSubversionDemo = 1,

    /*! Subversions used for ver = FE */
    DeviceSubversionE2HCExtAdc = 14,
    DeviceSubversionE2HCIntAdc = 15,
    DeviceSubversionENPRFairyLight = 16,
    DeviceSubversionENPR2Channels = 17,
    DeviceSubversionOrbitMiniSineWave = 18,

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
