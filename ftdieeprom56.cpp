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

#include "ftdieeprom56.h"

FtdiEeprom56::FtdiEeprom56(string deviceId) :
    FtdiEeprom(deviceId) {

    this->loadData();
}

FtdiEeprom56::~FtdiEeprom56() {

}

ErrorCodes_t FtdiEeprom56::loadData() {
    ErrorCodes_t ret;
    ret = this->openConnection();
    if ((ret != Success) && (ret != ErrorEepromAlreadyConnected)) {
        return ret;
    }

    ret = this->loadDeviceTuple();
    if (ret != Success) {
        return ret;
    }

    ret = this->loadVcOffset();
    if (ret != Success) {
        return ret;
    }

    ret = this->closeConnection();
    if (ret != Success) {
        return ret;
    }

    return ret;
}

ErrorCodes_t FtdiEeprom56::loadDeviceTuple() {
    ErrorCodes_t ret;
    WORD firmware;
    WORD versionSubversion;

    ret = this->readEepromWord(E56_DEVICE_VER_ADDR, &versionSubversion);
    if (ret != Success) {
        return ret;
    }

    ret = this->readEepromWord(E56_FW_VER_ADDR, &firmware);
    if (ret != Success) {
        return ret;
    }

    deviceTuple.version = (DeviceVersion_t)((versionSubversion >> 8) & 0xFF);
    deviceTuple.subversion = (DeviceSubversion_t)(versionSubversion & 0xFF);
    deviceTuple.fwVersion = firmware;

    return ret;
}

ErrorCodes_t FtdiEeprom56::loadVcOffset() {
    ErrorCodes_t ret;

    ret = this->readEepromWord(E56_VC_OFFSET_ADDR, &vcOffset);
    if (ret != Success) {
        return ret;
    }

    return ret;
}
