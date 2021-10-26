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

#include "ftdieeprom.h"

FtdiEeprom::FtdiEeprom(string deviceId) :
    deviceId(deviceId) {

}

FtdiEeprom::~FtdiEeprom() {

}

ErrorCodes_t FtdiEeprom::openConnection(char channel) {
    ErrorCodes_t ret;
    if (!connectionOpened) {
        /*! Appends the channel to the serial */
        communicationSerial = deviceId+channel;

        /*! Opens the connection with the handler */
        FT_STATUS result = FT_OpenEx((PVOID)communicationSerial.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &handler);
        connectionOpened = (result == FT_OK);

        if (connectionOpened) {
            ret = Success;

        } else {
            ret = ErrorEepromConnectionFailed;
        }

    } else {
        ret = ErrorEepromAlreadyConnected;
    }
    return ret;
}

ErrorCodes_t FtdiEeprom::closeConnection() {
    ErrorCodes_t ret;
    if (connectionOpened) {
        connectionOpened = (FT_Close(handler) != FT_OK);

        if (connectionOpened) {
            ret = ErrorEepromDisconnectionFailed;

        } else {
            ret = Success;
        }

    } else {
        ret = ErrorEepromNotConnected;
    }
    return ret;
}

ErrorCodes_t FtdiEeprom::readEepromWord(DWORD address, LPWORD result) {
    ErrorCodes_t ret;
    if (connectionOpened) {
        FT_STATUS ftRet = FT_ReadEE(handler, address, result);
        if (ftRet != FT_OK) {
            ret = ErrorEepromReadFailed;

        } else {
            ret = Success;
        }

    } else {
        ret = ErrorEepromNotConnected;
    }
    return ret;
}

DeviceTuple_t FtdiEeprom::getDeviceTuple() {
    return deviceTuple;
}

uint16_t FtdiEeprom::getVcOffset() {
    return vcOffset;
}
