/*! \file e4ocommlib_errorcodes.h
 * \brief Defines error codes returned by methods in e4oCommLib namespace.
 */
#ifndef E4OCOMMLIB_ERRORCODES_H
#define E4OCOMMLIB_ERRORCODES_H

namespace e4oCommLib {

/*! \enum ErrorGroups_t
 *  \brief Enumerated error groups.
 */
typedef enum {
    ErrorGroupDeviceDetection =     0x00010000, /*!< Error codes related to device detection errors. */
    ErrorGroupEepromCommunication = 0x00020000, /*!< Error codes related to eeprom communication errors. */
    ErrorGroupDeviceConnection =    0x00030000, /*!< Error codes related to connection errors. */
    ErrorGroupDeviceCommands =      0x00040000, /*!< Error codes related to failed commands to the device. */
    ErrorGroupDeviceFeatures =      0x00050000, /*!< Error codes related to wrongly used features. */
    ErrorGroupDeviceIssues =        0x00060000  /*!< Error codes related to issues with the device. */
} ErrorGroups_t;

/*! \enum ErrorCodes_t
 *  \brief Enumerated error codes.
 */
typedef enum {
    Success =                                                           0x00000000, /*!< Method returns with no errors. */

    ErrorNoDeviceFound =                ErrorGroupDeviceDetection +     0x00000001, /*!< Error returned when no devices are detected. */
    ErrorListDeviceFailed =             ErrorGroupDeviceDetection +     0x00000002, /*!< Error returned when device detection fails.
                                                                                     *   This error may be due to problems with FTDI driver installation too. */

    ErrorEepromAlreadyConnected =       ErrorGroupEepromCommunication +  0x00000001, /*!< Error returned when trying to connect to a device eeprom which is already connected. */
    ErrorEepromConnectionFailed =       ErrorGroupEepromCommunication +  0x00000002, /*!< Error returned when connection to a device eeprom fails. */
    ErrorEepromDisconnectionFailed =    ErrorGroupEepromCommunication +  0x00000003, /*!< Error returned when disconnection from a device eeprom fails. */
    ErrorEepromNotConnected =           ErrorGroupEepromCommunication +  0x00000004, /*!< Error returned when trying to communicate with a device eeprom if none is connected. */
    ErrorEepromReadFailed =             ErrorGroupEepromCommunication +  0x00000005, /*!< Error returned when reading from a device eeprom fails. */
    ErrorEepromNotRecognized =          ErrorGroupEepromCommunication +  0x00000006, /*!< Error returned when the eeprom is not recognized. */

    ErrorInitializationFailed =         ErrorGroupDeviceConnection +    0x00000001, /*!< Error returned if the communication library intitialization fails. */
    ErrorDeviceTypeNotRecognized =      ErrorGroupDeviceConnection +    0x00000002, /*!< Error returned when the device type is not recognized. */
    ErrorDeviceAlreadyConnected =       ErrorGroupDeviceConnection +    0x00000003, /*!< Error returned when trying to connect to a device which is already connected. */
    ErrorDeviceNotConnected =           ErrorGroupDeviceConnection +    0x00000004, /*!< Error returned when trying to communicate with a device if none is connected. */
    ErrorDeviceConnectionFailed =       ErrorGroupDeviceConnection +    0x00000005, /*!< Error returned when connection to a device fails. */
    ErrorFtdiConfigurationFailed =      ErrorGroupDeviceConnection +    0x00000006, /*!< Error returned when FTDI communication channel configuration fails. */
    ErrorDeviceDisconnectionFailed =    ErrorGroupDeviceConnection +    0x00000007, /*!< Error returned when disconnection from a device fails. */

    ErrorSendMessageFailed =            ErrorGroupDeviceCommands +      0x00000001, /*!< Error returned when sending a message to a device fails. */
    ErrorCommandNotImplemented =        ErrorGroupDeviceCommands +      0x00000002, /*!< Error returned when trying to use a command that is not implemented for the current device. */
    ErrorValueOutOfRange =              ErrorGroupDeviceCommands +      0x00000003, /*!< Error returned when trying to set a value that is out of range for the current device,
                                                                                     *   e.g. a current range index bigger than the total number of current ranges available. */
    WarningNotEnoughAvailable =         ErrorGroupDeviceCommands +      0x00000004, /*!< Warning returned when trying to read data but there's not enough. */

    ErrorFeatureNotImplemented =        ErrorGroupDeviceFeatures +      0x00000000, /*!< Error returned when trying to use a feature that is not implemented for the current device. */
    ErrorUpgradesNotAvailable =         ErrorGroupDeviceFeatures +      0x00000001, /*!< Error returned when there are no upgrades available for the current device. */

    ErrorExpiredDevice =                ErrorGroupDeviceIssues +        0x00000000, /*!< Error returned when the device has expired */

    ErrorUnknown =                                                      0xFFFFFFFF  /*!< Undefined error. */
} ErrorCodes_t;

} // e4oCommLib

#endif // E4OCOMMLIB_ERRORCODES_H
