#include "ftdieeprom.h"

FtdiEeprom::FtdiEeprom(std::string deviceId) :
    deviceId(deviceId) {

}

FtdiEeprom::~FtdiEeprom() {

}

er4cl::ErrorCodes_t FtdiEeprom::openConnection(char channel) {
    er4cl::ErrorCodes_t ret;
    if (!connectionOpened) {
        /*! Appends the channel to the serial */
        communicationSerial = deviceId+channel;

        /*! Opens the connection with the handler */
        FT_STATUS result = FT_OpenEx((PVOID)communicationSerial.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &handler);
        connectionOpened = (result == FT_OK);

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

er4cl::ErrorCodes_t FtdiEeprom::closeConnection() {
    er4cl::ErrorCodes_t ret;
    if (connectionOpened) {
        connectionOpened = (FT_Close(handler) != FT_OK);

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

er4cl::ErrorCodes_t FtdiEeprom::readEepromWord(DWORD address, LPWORD result) {
    er4cl::ErrorCodes_t ret;
    if (connectionOpened) {
        FT_STATUS ftRet = FT_ReadEE(handler, address, result);
        if (ftRet != FT_OK) {
            ret = er4cl::ErrorEepromReadFailed;

        } else {
            ret = er4cl::Success;
        }

    } else {
        ret = er4cl::ErrorEepromNotConnected;
    }
    return ret;
}

DeviceTuple_t FtdiEeprom::getDeviceTuple() {
    return deviceTuple;
}

uint16_t FtdiEeprom::getVcOffset() {
    return vcOffset;
}
