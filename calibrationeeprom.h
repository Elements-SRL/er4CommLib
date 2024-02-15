//  Copyright (C) 2021-2024 Filippo Cona
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

#ifndef CALIBRATIONEEPROM_H
#define CALIBRATIONEEPROM_H

#include <windows.h>

#include "ftd2xx_win.h"
#include "libMPSSE_spi.h"
#include "er4commlib_errorcodes.h"

/*! EEPROM constants */
#define CEE_XCBUS_DIR 0x09 /*! Set high output pins: FPGA reset 0x01 + SPI program 0x08 */
#define CEE_SPI_PROG_ENABLE 0x00
#define CEE_SPI_PROG_DISABLE 0x08
#define CEE_FPGA_RESET_ENABLE 0x00
#define CEE_FPGA_RESET_DISABLE 0x01
#define CEE_WRITE_DONE 0x01
#define CEE_EEPROM_SIZE 2048

/*! Eeprom commands */
#define CEE_PROGRAM_CMD 0x02
#define CEE_READ_CMD 0x03
#define CEE_GET_STATUS_CMD 0x05
#define CEE_WRITE_ENABLE_CMD 0x06

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

class CalibrationEeprom {
public:
    CalibrationEeprom(uint32 channel);
    ~CalibrationEeprom();

    ErrorCodes_t openConnection();
    ErrorCodes_t closeConnection();
    ErrorCodes_t enableFPGA(bool flag);
    ErrorCodes_t enableWrite();
    ErrorCodes_t writeBytes(unsigned char * values, unsigned int addr, unsigned int size);
    ErrorCodes_t pollWriteDone();
    ErrorCodes_t readBytes(unsigned char * values, unsigned int addr, unsigned int size);
    ErrorCodes_t readByte(unsigned char * value, unsigned int addr, bool start = true, bool end = true);
    ErrorCodes_t getStatus(unsigned char &eepromStatus);

    static unsigned int getMemorySize();

private:
    uint32 channelIdx;
    ChannelConfig config;
    FT_HANDLE handle;
    bool connectionOpened = false;

    /*! Buffers */
    unsigned char writeBuffer[256];
    unsigned char readBuffer[256];
};

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
};
#endif

#endif // CALIBRATIONEEPROM_H
