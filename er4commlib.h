/*! \file e4ocommlib.h
 * \brief Declares class CommLib.
 */
#ifndef E4OCOMMLIB_H
#define E4OCOMMLIB_H

#include <vector>
#include <string>

#include "e4ocommlib_global.h"
#include "e4ocommlib_errorcodes.h"

class MessageDispatcher;

namespace e4oCommLib {

/*! \struct QueueStatus_t
 * \brief Struct that contains information on the device status.
 * Returned by getQueueStatus.
 */
typedef struct {
    unsigned int availableDataPackets; /*!< Number of data packets available for read.
                                        * Each data packets consists of 1 sample per channel.
                                        * Successful calls to readData reduce this number. */
    bool bufferOverflowFlag; /*!< This flag is true if the internal buffer has been filled and old data has been overwritten.
                              * This flag is reset after a call to getQueueStatus. */
    bool lostDataFlag; /*!< This flag is true if the device has sent too much data and some has been lost.
                        * This flag is reset after a call to getQueueStatus. */
} QueueStatus_t;

/*******************\
 *  Init / Deinit  *
\*******************/

/*! \brief Initialize the communication library.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t init(
        E4OCL_ARGVOID);

/*! \brief Deinitialize the communication library.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t deinit(
            E4OCL_ARGVOID);

/************************\
 *  Connection methods  *
\************************/

/*! \brief Detects plugged in devices.
 *
 * \param deviceIds [out] List of plugged in devices IDs.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t detectDevices(
        E4OCL_ARGOUT std::vector <std::string> &deviceIds);

/*! \brief Connects to a specific device
 * Calling this method if a device is already connected will return an error code.
 *
 * \param deviceId [in] Device ID of the device to connect to.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t connect(
        E4OCL_ARGIN std::string deviceId);

/*! \brief Disconnects from connected device.
 * Calling this method if no device is connected will return an error code.
 *
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t disconnect(
        E4OCL_ARGVOID);

/****************\
 *  Tx methods  *
\****************/

/*! \brief Send buffered commands.
 *
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t sendCommands(
        E4OCL_ARGVOID);

/*! \brief Apply voltage on the external DAC.
 *
 * \param value [in] Voltage applied on the external DAC.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t applyDacExt(
        E4OCL_ARGIN Measurement_t voltage);

/*! \brief Defines if the interposer is inserted.
 *
 * \param flag [in] true if the intersposer is inserted, false otherwise.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setInterposerInserted(
        E4OCL_ARGIN bool flag);

/*! \brief When the interposer is inserted defines we are in conditioning or checking modality.
 *
 * \param flag [in] true if we are performing checking, false if we are performing conditioning.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setConditioningCheck(
        E4OCL_ARGIN bool flag);

/*! \brief When the interposer is inserted defines which channel we are working with.
 *
 * \param channelIdx [in] index of the channel being conditioned.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setConditioningChannel(
        E4OCL_ARGIN uint16_t channelIdx);

/*! \brief Select a protocol.
 *
 * \param idx [in] Index of the protocol selected.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t selectVoltageProtocol(
        E4OCL_ARGIN unsigned int idx);

/*! \brief Apply a protocol.
 *
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t applyVoltageProtocol(
        E4OCL_ARGVOID);

/*! \brief Set a protocol voltage value.
 *
 * \param idx [in] Index of the voltage set.
 * \param value [in] Value of the voltage set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolVoltage(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t voltage);

/*! \brief Set triangular voltage amplitude.
 *
 * \param value [in] Triangular voltage amplitude.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setTriangularVoltage(
        E4OCL_ARGIN Measurement_t voltage);

/*! \brief Set a protocol time value.
 *
 * \param idx [in] Index of the time set.
 * \param value [in] Value of the time set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolTime(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t time);

/*! \brief Set triangular period.
 *
 * \param value [in] Triangular period.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setTriangularTime(
        E4OCL_ARGIN Measurement_t time);

/*! \brief Set a FSM flag.
 *
 * \param idx [in] Index of the flag set.
 * \param value [in] Value of the flag set.
 * \param applyFlag [in] true to apply immediately, false to buffer and apply on next command.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setFsmFlag(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN bool flag,
        E4OCL_ARGIN bool applyFlag = true);

/*! \brief Set a FSM voltage value.
 *
 * \param idx [in] Index of the voltage set.
 * \param value [in] Value of the voltage set.
 * \param applyFlag [in] true to apply immediately, false to buffer and apply on next command.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setFsmVoltage(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t voltage,
        E4OCL_ARGIN bool applyFlag = true);

/*! \brief Set a FSM threshold current value.
 *
 * \param idx [in] Index of the threshold current set.
 * \param value [in] Value of the threshold current set.
 * \param applyFlag [in] true to apply immediately, false to buffer and apply on next command.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setFsmThresholdCurrent(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t current,
        E4OCL_ARGIN bool applyFlag = true);

/*! \brief Set a FSM time value.
 *
 * \param idx [in] Index of the time set.
 * \param value [in] Value of the time set.
 * \param applyFlag [in] true to apply immediately, false to buffer and apply on next command.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setFsmTime(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t time,
        E4OCL_ARGIN bool applyFlag = true);

/*! \brief Set a FSM integer value.
 *
 * \param idx [in] Index of the integer set.
 * \param value [in] Value of the integer set.
 * \param applyFlag [in] true to apply immediately, false to buffer and apply on next command.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setFsmInteger(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN unsigned int value,
        E4OCL_ARGIN bool applyFlag = true);

/*! \brief Set the moving average filter duration.
 *
 * \param duration [in] Duration of the moving average filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setMovingAverageFilterDuration(
        E4OCL_ARGIN Measurement_t duration);

/*! \brief Set the 4th order filter cut off frequency.
 *
 * \param cFrequency [in] Cut off frequency of the 4th order filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t set4thOrderFilterCutoffFrequency(
        E4OCL_ARGIN Measurement_t cFrequency);

/*! \brief Set the raw data filter cut off frequency.
 *
 * \param cFrequency [in] Cut off frequency of the raw data filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setRawDataFilterCutoffFrequency(
        E4OCL_ARGIN Measurement_t cFrequency);

/*! \brief Set a conditioning protocol voltage value.
 *
 * \param idx [in] Index of the voltage set.
 * \param value [in] Value of the voltage set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setConditioningProtocolVoltage(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t voltage);

/*! \brief Set a checking protocol voltage value.
 *
 * \param idx [in] Index of the voltage set.
 * \param value [in] Value of the voltage set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setCheckingProtocolVoltage(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t voltage);

/*! \brief Set a conditioning protocol time value.
 *
 * \param idx [in] Index of the time set.
 * \param value [in] Value of the time set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setConditioningProtocolTime(
        E4OCL_ARGIN unsigned int idx,
        E4OCL_ARGIN Measurement_t time);

/*! \brief Activate the front end reset denoiser.
 *
 * \param flag [in] False: de-activate the denoiser; True: activate the denoiser.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t activateFEResetDenoiser(
        E4OCL_ARGIN bool flag);

/*! \brief Activate the internal dac filter.
 *
 * \param flag [in] False: de-activate the internal dac filter; True: activate the internal dac filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t activateDacIntFilter(
        E4OCL_ARGIN bool flag);

/*! \brief Activate the external dac filter.
 *
 * \param flag [in] False: de-activate the external dac filter; True: activate the external dac filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t activateDacExtFilter(
        E4OCL_ARGIN bool flag);

/*! \brief Set the state of the VcmForce.
 *
 * \param flag [in] False: de-activate the VcmForce and enable VcInt; True: activate the VcmForce and disable VcInt.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setVcmForce(
        E4OCL_ARGIN bool flag);

/*! \brief Set the sampling rate.
 *
 * \param samplingRateIdx [in] Index of the sampling rate to be set.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t setSamplingRate(
        E4OCL_ARGIN uint16_t samplingRateIdx);

/*! \brief Execute digital offset compensation.
 * Digital offset compensation tunes the offset of the applied voltage so that the
 * acquired current is 0.
 *
 * \param on [in] False disables the digital offset compensation, true enables it.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t digitalOffsetCompensation(
        E4OCL_ARGIN bool on);

/*! \brief Zap.
 * A big voltage is applied in order to break the membrane.
 *
 * \param channelIdx [in] Index of the channel to zap.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t zap(
        E4OCL_ARGIN uint16_t channelIdx);

/*! \brief Reset the device.
 *
 * \param reset [in] False sets the device in normal operation state, true sets in reset state.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t resetDevice(
        E4OCL_ARGIN bool reset);

///*! \brief Reset the device's digital offset compensation.
// *
// * \param reset [in] False sets the digital offset compensation in normal operation state, true sets in reset state.
// * \return Error code.
// */
//ErrorCodes_t resetDigitalOffsetCompensation(
//        E4OCL_ARGIN bool reset);

/****************\
 *  Rx methods  *
\****************/

///*! \brief Get notification of possible upgrades for the connected device.
// * Returns Success if there are upgrades available.
// *
// * \param upgradeNotes [out] Notes of the available upgrades.
// * \return Error code.
// */
//ErrorCodes_t isDeviceUpgradable(
//        E4OCL_ARGOUT std::string &upgradeNotes,
//        E4OCL_ARGOUT std::string &notificationTag);

///*! \brief Get the device identification information (to be used when the device is already connected).
// *
// * \param deviceVersion [out] Device version.
// * \param deviceSubversion [out] Device subversion.
// * \param firmwareVersion [out] Firmware version.
// * \return Error code.
// */
//ErrorCodes_t getDeviceInfo(
//        E4OCL_ARGOUT uint8_t &deviceVersion,
//        E4OCL_ARGOUT uint8_t &deviceSubversion,
//        E4OCL_ARGOUT uint32_t &firmwareVersion);

///*! \brief Get the device identification information (to be used when the device is not connected yet).
// *
// * \param deviceId [in] Device identification code (S/N).
// * \param deviceVersion [out] Device version.
// * \param deviceSubversion [out] Device subversion.
// * \param firmwareVersion [out] Firmware version.
// * \return Error code.
// */
//static ErrorCodes_t getDeviceInfo(
//        E4OCL_ARGOUT std::string deviceId,
//        E4OCL_ARGOUT uint8_t &deviceVersion,
//        E4OCL_ARGOUT uint8_t &deviceSubversion,
//        E4OCL_ARGOUT uint32_t &firmwareVersion);

/*! \brief Returns information on the queue status.
 * Calling this method if no device is connected will return an error code.
 *
 * \param status [out] Struct containing queue information.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getQueueStatus(
        E4OCL_ARGOUT QueueStatus_t &status);

/*! \brief Reads data packets from the device into a buffer.
 * The returned buffer contains \a dataRead valid data packets of with one sample for each channel.
 * \a buffer is allocated by method init() with #E4OCL_DATA_ARRAY_SIZE items and freed by method deinit().\n
 * Calling this method with \a dataRead greater than the actual number of available data packets will return an error code,
 * but the returned \a buffer will still contain all the available data packets, and \a dataRead will be smaller than
 * \a dataToRead. \n
 * Calling this method if no device is connected will return an error code.
 *
 * \param dataToRead [in] Number of data packets to read.
 * \param dataRead [out] Number of data packets actually read (may be lower than dataToRead).
 * \param buffer [out] Buffer of data read. Each data packet consists of 1 sample per channel.
 * In each data packet the first sample is the voltage command;
 * the following samples are the measured currents, 1 sample per current channel.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t readData(E4OCL_ARGIN unsigned int dataToRead,
                      E4OCL_ARGOUT unsigned int &dataRead,
                      E4OCL_ARGOUT uint16_t * &buffer);

/*! \brief Converts an integer number to the corresponding voltage value.
 * The converted voltage's unit depends on the device configuration.
 *
 * \param intValue [in] Integer value to be converted.
 * \param fltValue [out] Floating point voltage converted value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t convertVoltageValue(E4OCL_ARGIN uint16_t intValue,
                                 E4OCL_ARGOUT double &fltValue);

/*! \brief Converts an integer number to the corresponding current value.
 * The converted current's unit depends on the device configuration.
 *
 * \param intValue [in] Integer value to be converted.
 * \param fltValue [out] Floating point current converted value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t convertCurrentValue(E4OCL_ARGIN uint16_t intValue,
                                 E4OCL_ARGOUT double &fltValue);

/*! \brief Purges data read from the device.
 * This command is useful to get rid of data acquired during the device configuration (e.g. during setting of sampling rate or digital offset compensation).
 * Calling this method if no device is connected will return an error code.
 *
 * \return #EdlErrorCode_t Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t purgeData(E4OCL_ARGVOID);

/*! \brief Get the number of channels for the device.
 *
 * \param fsmStatesChannelsNum [out] Number of channels for the state of the FSM.
 * \param voltageChannelsNum [out] Number of voltage channels.
 * \param currentChannelsNum [out] Number of current channels (each current channel is reutrned raw, filtered and averaged).
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getChannelsNumber(
        E4OCL_ARGOUT uint32_t &fsmStatesChannelsNum,
        E4OCL_ARGOUT uint32_t &voltageChannelsNum,
        E4OCL_ARGOUT uint32_t &currentChannelsNum);

/*! \brief Get the current ranges available in voltage clamp for the device.
 *
 * \param currentRanges [out] Array containing all the available current ranges in voltage clamp.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getCurrentRanges(
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &currentRanges);

/*! \brief Get the current range currently applied for the sequencing algorithm.
 *
 * \param currentRange [out] Current range currently applied for the sequencing algorithm.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getCurrentRange(
        E4OCL_ARGOUT RangedMeasurement_t &currentRange);

/*! \brief Get the current range currently applied for the nanopore checking.
 *
 * \param currentRange [out] Current range currently applied for the nanopore checking.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getCheckingCurrentRange(
        E4OCL_ARGOUT RangedMeasurement_t &currentRange);

/*! \brief Get the current range currently applied for the nanopore conditioning.
 *
 * \param currentRange [out] Current range currently applied for the nanopore conditioning.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getConditioningCurrentRange(
        E4OCL_ARGOUT RangedMeasurement_t &currentRange);

/*! \brief Get the voltage ranges available in voltage clamp for the device.
 *
 * \param voltageRanges [out] Array containing all the available voltage ranges in voltage clamp.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageRanges(
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &voltageRanges);

/*! \brief Get the voltage range currently applied for the sequencing algorithm.
 *
 * \param voltageRange [out] Voltage range currently applied for the sequencing algorithm.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getCheckingVoltageRange(
        E4OCL_ARGOUT RangedMeasurement_t &voltageRange);

/*! \brief Get the voltage range currently applied for the nanopore checking.
 *
 * \param voltageRange [out] Voltage range currently applied for the nanopore checking.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getConditioningVoltageRange(
        E4OCL_ARGOUT RangedMeasurement_t &voltageRange);

/*! \brief Get the voltage range currently applied for the nanopore conditioning.
 *
 * \param voltageRange [out] Voltage range currently applied for the nanopore conditioning.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageRange(
        E4OCL_ARGOUT RangedMeasurement_t &voltageRange);

/*! \brief Get the sampling rates available for the device.
 *
 * \param samplingRates [out] Array containing all the available sampling rates.
 * \param defaultValue [out] Default option.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getSamplingRates(
        E4OCL_ARGOUT std::vector <Measurement_t> &samplingRates,
        E4OCL_ARGOUT unsigned int &defaultOption);

/*! \brief Get the real sampling rates available for the device.
 *
 * \param samplingRates [out] Array containing all the available real sampling rates
 *                            (may slightly differ from displayed sampling rates).
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getRealSamplingRates(
        E4OCL_ARGOUT std::vector <Measurement_t> &samplingRates);

/*! \brief Get the available options for the voltage stimulus low pass filter.
 *
 * \param opened [out] Available options for the voltage stimulus low pass filter.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageStimulusLpfs(
        E4OCL_ARGOUT std::vector <std::string> &filterOptions);

/*! \brief Get the specifications of the control for the liquid junction.
 *
 * \param control [out] Specifications of the control for the liquid junction.
 * \return Success if the device implements liquid junction control.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getLiquidJunctionControl(
        E4OCL_ARGOUT CompensationControl_t &control);

/*! \brief Get appliable voltage range on the external DAC.
 *
 * \param value [out] Appliable voltage range on the external DAC.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getDacExtRange(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get protocol list.
 *
 * \param protocolsNames [out] Names of available protocols.
 * \param triangularIdx [out] Index of triangular protocol.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolList(
        E4OCL_ARGOUT std::vector <std::string> &protocolsNames,
        E4OCL_ARGOUT unsigned int &triangularIdx);

/*! \brief Get protocol appliable voltage range.
 *
 * \param voltageNames [out] Names of available voltages.
 * \param ranges [out] Ranges of appliable voltage in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolVoltage(
        E4OCL_ARGOUT std::vector <std::string> &voltageNames,
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &ranges,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get triangular appliable voltage range.
 *
 * \param range [out] Range of appliable voltage amplitude in triangular protocol.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getTriangularVoltage(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get protocol appliable time range.
 *
 * \param timeNames [out] Names of available times.
 * \param range [out] Range of appliable time in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolTime(
        E4OCL_ARGOUT std::vector <std::string> &timeNames,
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get triangular appliable period range.
 *
 * \param range [out] Range of appliable period in triangular protocol.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getTriangularTime(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get FSM flags.
 *
 * \param integerNames [out] Names of available flags.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getFsmFlag(
        E4OCL_ARGOUT std::vector <std::string> &flagNames,
        E4OCL_ARGOUT std::vector <bool> &defaultValues);

/*! \brief Get FSM voltage appliable range.
 *
 * \param voltageNames [out] Names of available voltages.
 * \param range [out] Range of appliable voltage in the FSM.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getFsmVoltage(
        E4OCL_ARGOUT std::vector <std::string> &voltageNames,
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get FSM threshold current appliable range.
 *
 * \param currentNames [out] Names of available currents.
 * \param range [out] Range of appliable threshold current in the FSM.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getFsmThresholdCurrent(
        E4OCL_ARGOUT std::vector <std::string> &currentNames,
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get FSM time appliable range.
 *
 * \param timeNames [out] Names of available times.
 * \param range [out] Range of appliable time in the FSM.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getFsmTime(
        E4OCL_ARGOUT std::vector <std::string> &timeNames,
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get FSM integer appliable range.
 *
 * \param integerNames [out] Names of available integers.
 * \param range [out] Range of appliable integers in the FSM.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getFsmInteger(
        E4OCL_ARGOUT std::vector <std::string> &integerNames,
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get the moving average filter appliable duration range.
 *
 * \param range [out] Range of appliable duration of the moving average filter.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getMovingAverageFilterDuration(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get the 4th order filter appliable cut off frequency range.
 *
 * \param range [out] Range of appliable cut off frequency of the 4th order filter.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t get4thOrderFilterCutoffFrequency(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get the raw data filter appliable cut off frequency range.
 *
 * \param range [out] Range of appliable cut off frequency of the raw data filter.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getRawDataFilterCutoffFrequency(
        E4OCL_ARGOUT RangedMeasurement_t &range,
        E4OCL_ARGOUT Measurement_t &defaultValue);

/*! \brief Get conditioning protocol appliable voltage range.
 *
 * \param voltageNames [out] Names of available voltages.
 * \param ranges [out] Ranges of appliable voltage in conditioning protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getConditioningProtocolVoltage(
        E4OCL_ARGOUT std::vector <std::string> &voltageNames,
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &ranges,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get checking protocol appliable voltage range.
 *
 * \param voltageNames [out] Names of available voltages.
 * \param ranges [out] Ranges of appliable voltage in checking protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getCheckingProtocolVoltage(
        E4OCL_ARGOUT std::vector <std::string> &voltageNames,
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &ranges,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);

/*! \brief Get conditioning protocol appliable time range.
 *
 * \param timeNames [out] Names of available times.
 * \param ranges [out] Ranges of appliable time in conditioning protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
E4OCOMMLIBSHARED_EXPORT
ErrorCodes_t getConditioningProtocolTime(
        E4OCL_ARGOUT std::vector <std::string> &timeNames,
        E4OCL_ARGOUT std::vector <RangedMeasurement_t> &ranges,
        E4OCL_ARGOUT std::vector <Measurement_t> &defaultValues);
}

#endif // E4OCOMMLIB_H
