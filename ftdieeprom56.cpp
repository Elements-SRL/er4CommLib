#include "ftdieeprom56.h"

FtdiEeprom56::FtdiEeprom56(std::string deviceId) :
    FtdiEeprom(deviceId) {

    this->loadData();
}

FtdiEeprom56::~FtdiEeprom56() {

}

er4cl::ErrorCodes_t FtdiEeprom56::loadData() {
    er4cl::ErrorCodes_t ret;
    ret = this->openConnection();
    if ((ret != er4cl::Success) && (ret != er4cl::ErrorEepromAlreadyConnected)) {
        return ret;
    }

    ret = this->loadDeviceTuple();
    if (ret != er4cl::Success) {
        return ret;
    }

    ret = this->loadVcOffset();
    if (ret != er4cl::Success) {
        return ret;
    }

    ret = this->closeConnection();
    if (ret != er4cl::Success) {
        return ret;
    }

    return ret;
}

er4cl::ErrorCodes_t FtdiEeprom56::loadDeviceTuple() {
    er4cl::ErrorCodes_t ret;
    WORD firmware;
    WORD versionSubversion;

    ret = this->readEepromWord(E56_DEVICE_VER_ADDR, &versionSubversion);
    if (ret != er4cl::Success) {
        return ret;
    }

    ret = this->readEepromWord(E56_FW_VER_ADDR, &firmware);
    if (ret != er4cl::Success) {
        return ret;
    }

    deviceTuple.version = (DeviceVersion_t)((versionSubversion >> 8) & 0xFF);
    deviceTuple.subversion = (DeviceSubversion_t)(versionSubversion & 0xFF);
    deviceTuple.fwVersion = firmware;

    return ret;
}

er4cl::ErrorCodes_t FtdiEeprom56::loadVcOffset() {
    er4cl::ErrorCodes_t ret;

    ret = this->readEepromWord(E56_VC_OFFSET_ADDR, &vcOffset);
    if (ret != er4cl::Success) {
        return ret;
    }

    return ret;
}
