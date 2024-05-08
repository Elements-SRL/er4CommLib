#include "ftdieepromdemo.h"

FtdiEepromDemo::FtdiEepromDemo(std::string deviceId) :
    FtdiEeprom(deviceId) {

    this->loadData();
}

FtdiEepromDemo::~FtdiEepromDemo() {

}

er4cl::ErrorCodes_t FtdiEepromDemo::openConnection(char channel) {
    er4cl::ErrorCodes_t ret;
    if (!connectionOpened) {
        /*! Appends the channel to the serial */
        communicationSerial = deviceId+channel;

        /*! Fake connection */
        connectionOpened = true;

        if (connectionOpened) {
            ret = er4cl::Success;

        } else {
            ret = er4cl::ErrorEepromConnectionFailed;
        }

    } else {
        ret = er4cl::ErrorEepromAlreadyConnected;
    }
    return ret;
}

er4cl::ErrorCodes_t FtdiEepromDemo::closeConnection() {
    er4cl::ErrorCodes_t ret;
    if (connectionOpened) {
        /*! Fake disconnection */
        connectionOpened = false;

        if (connectionOpened) {
            ret = er4cl::ErrorEepromDisconnectionFailed;

        } else {
            ret = er4cl::Success;
        }

    } else {
        ret = er4cl::ErrorEepromNotConnected;
    }
    return ret;
}

er4cl::ErrorCodes_t FtdiEepromDemo::loadData() {
    er4cl::ErrorCodes_t ret;
    ret = this->openConnection();
    if ((ret != er4cl::Success) && (ret != er4cl::ErrorEepromAlreadyConnected)) {
        return ret;
    }

    ret = this->loadDeviceTuple();
    if (ret != er4cl::Success) {
        return ret;
    }

    ret = this->closeConnection();
    if (ret != er4cl::Success) {
        return ret;
    }

    return ret;
}

er4cl::ErrorCodes_t FtdiEepromDemo::loadDeviceTuple() {
    er4cl::ErrorCodes_t ret = er4cl::Success;

    deviceTuple.version = DeviceVersionDemo;
    deviceTuple.subversion = DeviceSubversionDemo;
    deviceTuple.fwVersion = 129;

    return ret;
}

er4cl::ErrorCodes_t FtdiEepromDemo::loadVcOffset() {
    er4cl::ErrorCodes_t ret = er4cl::Success;

    vcOffset = 1650.0;

    return ret;
}
