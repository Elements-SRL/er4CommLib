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

#include "calibrationeeprom.h"

CalibrationEeprom::CalibrationEeprom(uint32 channel) :
    channelIdx(channel) {

    config.ClockRate = 3000000; /*! 3MHz */
    config.LatencyTimer = 1; /*! 1ms */
    config.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS5 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    config.Pin = 0x00002323;
}

CalibrationEeprom::~CalibrationEeprom() {

}

ErrorCodes_t CalibrationEeprom::openConnection() {
    if (!connectionOpened) {
        connectionOpened = true;
        FT_STATUS status;
        Init_libMPSSE();

        status = SPI_OpenChannel(channelIdx, &handle);
        if (status != FT_OK) {
            connectionOpened = false;
            return ErrorEepromConnectionFailed;
        }

        status = SPI_InitChannel(handle, &config);
        if (status != FT_OK) {
            connectionOpened = false;
            return ErrorEepromConnectionFailed;
        }

        status = this->enableFPGA(false);
        if (status != FT_OK) {
            connectionOpened = false;
            return ErrorEepromConnectionFailed;
        }

        return Success;

    } else {
        return ErrorEepromAlreadyConnected;
    }
}

ErrorCodes_t CalibrationEeprom::closeConnection() {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    FT_STATUS status;
    status = this->enableFPGA(true);
    if (status != FT_OK) {
        return ErrorEepromDisconnectionFailed;
    }

    status = SPI_CloseChannel(handle);
    if (status != FT_OK) {
        return ErrorEepromDisconnectionFailed;
    }

    Cleanup_libMPSSE();

    connectionOpened = false;
    return Success;
}

ErrorCodes_t CalibrationEeprom::enableFPGA(bool flag) {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    FT_STATUS status;
    if (flag) {
        status = FT_WriteGPIO(handle, CEE_XCBUS_DIR, CEE_SPI_PROG_DISABLE | CEE_FPGA_RESET_DISABLE);

    } else {
        status = FT_WriteGPIO(handle, CEE_XCBUS_DIR, CEE_SPI_PROG_ENABLE | CEE_FPGA_RESET_ENABLE);
    }

    if (status != FT_OK) {
        return ErrorEepromWriteFailed;

    } else {
        return Success;
    }
}

ErrorCodes_t CalibrationEeprom::enableWrite() {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    FT_STATUS status;

    uint32 bytesWritten = 0;
    int bytesToWrite = 0;
    writeBuffer[bytesToWrite++] = CEE_WRITE_ENABLE_CMD;
    status = SPI_Write(handle, writeBuffer, bytesToWrite, &bytesWritten, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
    if (status != FT_OK) {
        return ErrorEepromWriteFailed;
    }

    return Success;
}

ErrorCodes_t CalibrationEeprom::writeBytes(unsigned char * values, unsigned int addr, unsigned int size) {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    if (addr+size > CEE_EEPROM_SIZE) {
        return ErrorEepromInvalidAddress;
    }

    ErrorCodes_t ret;
    FT_STATUS status;

    ret = this->enableWrite();
    /*! Write is disabled by successful write */
    if (ret != Success) {
        return ret;
    }

    uint32 bytesWritten = 0;
    int bytesToWrite = 0;
    writeBuffer[bytesToWrite++] = CEE_PROGRAM_CMD;
    writeBuffer[bytesToWrite++] = (unsigned char)((addr & 0xFF00) >> 8);
    writeBuffer[bytesToWrite++] = (unsigned char)(addr & 0x00FF);
    status = SPI_Write(handle, writeBuffer, bytesToWrite, &bytesWritten, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);
    if (status != FT_OK) {
        return ErrorEepromWriteFailed;
    }

    status = SPI_Write(handle, values, size, &bytesWritten, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
    if (status != FT_OK) {
        return ErrorEepromWriteFailed;
    }
    return this->pollWriteDone();
}

ErrorCodes_t CalibrationEeprom::readBytes(unsigned char * values, unsigned int addr, unsigned int size) {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    ErrorCodes_t ret;
    bool start;
    bool end;
    for (unsigned int idx = 0; idx < size; idx++) {
        start = (idx == 0 ? true : false);
        end = (idx == size-1 ? true : false);
        if ((ret = this->readByte(values+idx, addr+idx, start, end)) != Success) {
            break;
        }
    }
    return ret;
}

ErrorCodes_t CalibrationEeprom::readByte(unsigned char * value, unsigned int addr, bool start, bool end) {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    if (addr >= CEE_EEPROM_SIZE) {
        return ErrorEepromInvalidAddress;
    }

    FT_STATUS status;
    uint32 bytesWritten = 0;
    uint32 bytesRead = 0;

    if (start) {
        int bytesToWrite = 0;
        writeBuffer[bytesToWrite++] = CEE_READ_CMD;
        writeBuffer[bytesToWrite++] = (unsigned char)((addr & 0xFF00) >> 8);
        writeBuffer[bytesToWrite++] = (unsigned char)(addr & 0x00FF);
        status = SPI_Write(handle, writeBuffer, bytesToWrite, &bytesWritten, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);
        if (status != FT_OK) {
            return ErrorEepromWriteFailed;
        }
    }

    if (end) {
        status = SPI_Read(handle, value, 1, &bytesRead, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);

    } else {
        status = SPI_Read(handle, value, 1, &bytesRead, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
    }
    if (status != FT_OK) {
        return ErrorEepromReadFailed;

    } else {
        return Success;
    }
}

ErrorCodes_t CalibrationEeprom::pollWriteDone() {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    unsigned char eepromStatus = CEE_WRITE_DONE;
    while (eepromStatus & CEE_WRITE_DONE) { /*! \todo FCON magari mettere un timeout */
        this->getStatus(eepromStatus);
    }
    return Success;
}

ErrorCodes_t CalibrationEeprom::getStatus(unsigned char &eepromStatus) {
    if (!connectionOpened) {
        return ErrorEepromNotConnected;
    }

    int bytesToWrite = 0;
    int bytesToRead = 0;
    uint32 bytesWritten = 0;
    uint32 bytesRead = 0;

    writeBuffer[bytesToWrite++] = CEE_GET_STATUS_CMD;
    FT_STATUS status = SPI_Write(handle, writeBuffer, bytesToWrite, &bytesWritten, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);
    if (status != FT_OK) {
        return ErrorEepromWriteFailed;
    }

    bytesToRead = 1;
    status = SPI_Read(handle, readBuffer, bytesToRead, &bytesRead, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
    if (status != FT_OK) {
        return ErrorEepromWriteFailed;
    }

    eepromStatus = readBuffer[0];
    return Success;
}

unsigned int CalibrationEeprom::getMemorySize() {
    return CEE_EEPROM_SIZE;
}
