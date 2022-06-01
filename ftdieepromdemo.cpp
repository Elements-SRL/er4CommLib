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

#include "ftdieepromdemo.h"

FtdiEepromDemo::FtdiEepromDemo(string deviceId) :
    FtdiEeprom(deviceId) {

    this->loadData();
}

FtdiEepromDemo::~FtdiEepromDemo() {

}

ErrorCodes_t FtdiEepromDemo::openConnection(char channel) {
    ErrorCodes_t ret;
    if (!connectionOpened) {
        /*! Appends the channel to the serial */
        communicationSerial = deviceId+channel;

        /*! Fake connection */
        connectionOpened = true;

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

ErrorCodes_t FtdiEepromDemo::closeConnection() {
    ErrorCodes_t ret;
    if (connectionOpened) {
        /*! Fake disconnection */
        connectionOpened = false;

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

ErrorCodes_t FtdiEepromDemo::setVcOffset(uint16_t) {
    return Success;
}

ErrorCodes_t FtdiEepromDemo::loadData() {
    ErrorCodes_t ret;
    ret = this->openConnection();
    if ((ret != Success) && (ret != ErrorEepromAlreadyConnected)) {
        return ret;
    }

    ret = this->loadDeviceTuple();
    if (ret != Success) {
        return ret;
    }

    ret = this->closeConnection();
    if (ret != Success) {
        return ret;
    }

    return ret;
}

ErrorCodes_t FtdiEepromDemo::loadDeviceTuple() {
    ErrorCodes_t ret = Success;

    deviceTuple.version = DeviceVersionDemo;
    deviceTuple.subversion = DeviceSubversionDemo;
    deviceTuple.fwVersion = 129;

    return ret;
}

ErrorCodes_t FtdiEepromDemo::loadVcOffset() {
    ErrorCodes_t ret = Success;

    vcOffset = 1650.0;

    return ret;
}
