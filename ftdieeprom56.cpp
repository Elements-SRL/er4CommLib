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
