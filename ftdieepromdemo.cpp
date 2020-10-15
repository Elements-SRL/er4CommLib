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
