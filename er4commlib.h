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

/*! \file er4commlib.h
 * \brief Declares class CommLib.
 */
#ifndef ER4COMMLIB_H
#define ER4COMMLIB_H


#include "er4commlib_global.h"
#include "er4commlib_errorcodes.h"

class MessageDispatcher;

/*! \struct QueueStatus_t
 * \brief Struct that contains information on the device status.
 * Returned by getQueueStatus.
 */
typedef struct QueueStatus{
    unsigned int availableDataPackets; /*!< Number of data packets available for read.
                                        * Each data packets consists of 1 sample per channel.
                                        * Successful calls to readData reduce this number. */
    bool bufferOverflowFlag; /*!< This flag is true if the internal buffer has been filled and old data has been overwritten.
                              *   This flag is reset after a call to getQueueStatus or to purgeData. */
    bool lostDataFlag; /*!< This flag is true if the device has sent too much data and some has been lost.
                        *   This flag is reset after a call to getQueueStatus or to purgeData. */
    bool saturationFlag; /*!< This flag is true if some data saturates the front end range.
                          *   This flag is reset after a call to getQueueStatus or to purgeData. */
    bool currentRangeIncreaseFlag; /*!< This flag is true if any current channel is above the threshold that suggests an increase of front end current range.
                                    *   This flag is reset after a call to getQueueStatus or to purgeData. */
    bool currentRangeDecreaseFlag; /*!< This flag is true if all current channels are below the threshold that suggests a decrease of front end current range.
                                    *   This flag is reset after a call to getQueueStatus or to purgeData. */
    bool communicationErrorFlag; /*!< This flag is true after a communication error with the device.
                                  *   This flag is reset if the communication restarts successfully. */
} QueueStatus_t;

/*******************\
 *  Init / Deinit  *
\*******************/

/*! \brief Initialize the communication library.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t init(
        ER4CL_ARGVOID);

/*! \brief Deinitialize the communication library.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t deinit(
            ER4CL_ARGVOID);

/************************\
 *  Connection methods  *
\************************/

/*! \brief Detects plugged in devices.
 *
 * \param deviceIds [out] List of plugged in devices IDs.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t detectDevices(
        ER4CL_ARGOUT char * deviceIds);

/*! \brief Connects to a specific device
 * Calling this method if a device is already connected will return an error code.
 *
 * \param deviceId [in] Device ID of the device to connect to.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t connectDevice(
        ER4CL_ARGIN char * deviceId);

/*! \brief Disconnects from connected device.
 * Calling this method if no device is connected will return an error code.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t disconnect(
        ER4CL_ARGVOID);

/****************\
 *  Tx methods  *
\****************/

/*! \brief Send buffered commands.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t sendCommands(
        ER4CL_ARGVOID);

/*! \brief Select a protocol.
 *
 * \param idx [in] Index of the protocol selected.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t selectVoltageProtocol(
        ER4CL_ARGIN unsigned int idx);

/*! \brief Apply a protocol.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t applyVoltageProtocol(
        ER4CL_ARGVOID);

/*! \brief Set a protocol voltage value.
 *
 * \param idx [in] Index of the voltage set.
 * \param voltage [in] Value of the voltage set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolVoltage(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage);

/*! \brief Set a protocol time value.
 *
 * \param idx [in] Index of the time set.
 * \param time [in] Value of the time set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolTime(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t time);

/*! \brief Set a protocol slope value.
 *
 * \param idx [in] Index of the slope set.
 * \param value [in] Value of the slope set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolSlope(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t slope);

/*! \brief Set a protocol adimensional value.
 *
 * \param idx [in] Index of the adimensional set.
 * \param adimensional [in] Value of the adimensional set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setProtocolAdimensional(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t adimensional);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the protocol to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkSelectedProtocol(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN char * message);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the voltage parameter to be checked.
 * \param voltage [in] Value of the voltage parameter to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkProtocolVoltage(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage,
        ER4CL_ARGIN char * message);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the time parameter to be checked.
 * \param time [in] Value of the time parameter to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkProtocolTime(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t time,
        ER4CL_ARGIN char * message);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the slope parameter to be checked.
 * \param slope [in] Value of the slope parameter to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkProtocolSlope(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t slope,
        ER4CL_ARGIN char * message);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the adimensional parameter to be checked.
 * \param adimensional [in] Value of the adimensional parameter to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkProtocolAdimensional(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t adimensional,
        ER4CL_ARGIN char * message);

/*! \brief Set a channel voltage offset.
 *
 * \param idx [in] Index of the channel.
 * \param voltage [in] Value of the voltage offset.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setVoltageOffset(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage);

/*! \brief Check if the protocol parameters are valid.
 *
 * \param idx [in] Index of the channel voltage offset to be checked.
 * \param voltage [in] Value of the voltage offset to be checked.
 * \param message [in] Error message in case the parameters set is invalid.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t checkVoltageOffset(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage,
        ER4CL_ARGIN char * message);

/*! \brief Apply the insertion pulse if available.
 *
 * \param voltage [in] Voltage of the insertion pulse to be applied.
 * \param duration [in] Duration of the insertion pulse to be applied.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t applyInsertionPulse(
        ER4CL_ARGIN MeasurementReduced_t voltage,
        ER4CL_ARGIN MeasurementReduced_t duration);

/*! \brief Apply the reference pulse if available.
 *
 * \param voltage [in] Voltage of the reference pulse to be applied.
 * \param duration [in] Duration of the reference pulse to be applied.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t applyReferencePulse(
        ER4CL_ARGIN MeasurementReduced_t voltage,
        ER4CL_ARGIN MeasurementReduced_t duration);

/*! \brief Override the voltage reference switch.
 *
 * \param applyFlag [in] true: apply the override of voltage reference.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t overrideReferencePulse(
        ER4CL_ARGIN bool flag);

/*! \brief Set the raw data filter cut off frequency and type.
 *
 * \param cutoffFrequency [in] Cut off frequency of the raw data filter.
 * \param lowPassFlag [in] true: set a low pass filter; false: set a high pass filter.
 * \param activeFlag [in] true: enable the filter; false: disable the filter.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setRawDataFilter(
        ER4CL_ARGIN MeasurementReduced_t cutoffFrequency,
        ER4CL_ARGIN bool lowPassFlag,
        ER4CL_ARGIN bool activeFlag);

/*! \brief Apply voltage on the external DAC.
 *
 * \param value [in] Voltage applied on the external DAC.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t applyDacExt(
        ER4CL_ARGIN MeasurementReduced_t voltage);

/*! \brief Reset the error status for the Orbit washer.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t resetWasherError(
        ER4CL_ARGVOID);

/*! \brief Set the Orbit washer preset speed values.
 *
 * \param speedValues [in] Vector of preset speed values.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setWasherPresetSpeeds(
        ER4CL_ARGIN int8_t * speedValues);

/*! \brief Start the Orbit washer at a given preset speed.
 *
 * \param speedIdx [in] Index of the preset speed to use.
 * \return Error code.
 */

ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t startWasher(
        ER4CL_ARGIN uint16_t speedIdx);

/*! \brief Forces an update of the washer status, otherwise the update is done automatically after a reset.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t updateWasherState(
        ER4CL_ARGVOID);

/*! \brief Forces an update of the washer preset speeds, otherwise the update is done automatically after the preset speeds are set.
 *
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t updateWasherPresetSpeeds(
        ER4CL_ARGVOID);

/*! \brief Set the current range on a given channel.
 *
 * \param currentRangeIdx [in] Index of the current range to be set.
 * \param channelIdx [in] Index of the channel to set the current range for; set equal to the number of channels to set them all.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setCurrentRange(
        ER4CL_ARGIN uint16_t currentRangeIdx,
        ER4CL_ARGIN uint16_t channelIdx);

/*! \brief Set the voltage range.
 *
 * \param voltageRangeIdx [in] Index of the voltage range to be set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setVoltageRange(
        ER4CL_ARGIN uint16_t voltageRangeIdx);

/*! \brief Set the voltage range for the reference.
 *
 * \param voltageRangeIdx [in] Index of the voltage range to be set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setVoltageReferenceRange(
        ER4CL_ARGIN uint16_t voltageRangeIdx);

/*! \brief Set the sampling rate.
 *
 * \param samplingRateIdx [in] Index of the sampling rate to be set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setSamplingRate(
        ER4CL_ARGIN uint16_t samplingRateIdx);

/*! \brief Set the oversampling ratio.
 *
 * \param oversamplingRatioIdx [in] Index of the oversampling ratio to be set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setOversamplingRatio(
        ER4CL_ARGIN uint16_t oversamplingRatioIdx);

/*! \brief Sets the low pass filter on the voltage stimulus.
 *
 * \param opened [in] Index of the filter setting (get available settings with method getVoltageStimulusLpfs).
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setVoltageStimulusLpf(
        ER4CL_ARGIN uint16_t filterIdx);

/*! \brief Sets the low pass filter on the voltage reference.
 *
 * \param opened [in] Index of the filter setting (get available settings with method getVoltageReferenceLpfs).
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setVoltageReferenceLpf(
        ER4CL_ARGIN uint16_t filterIdx);

/*! \brief Select channels for voltage stimulation.
 * On multi channel devices it is possible to apply the voltage stimulus only on a subset
 * of the available channels.
 *
 * \param channelIdx [in] Index of the channel to select/deselect fot voltage stimualtion.
 * Set equal to the number of current channels to select/deselect them all.
 * \param on [in] False deselect the channel for voltage stimulation, true selects it.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t selectStimulusChannel(
        ER4CL_ARGIN uint16_t channelIdx,
        ER4CL_ARGIN bool on);

/*! \brief Execute digital offset compensation.
 * Digital offset compensation tunes the offset of the applied voltage so that the
 * acquired current is 0.
 *
 * \param channelIdx [in] Index of the channel to apply the digital offset compensation to.
 * Set equal to the number of current channels to apply to all channels.
 * \param on [in] False disables the digital offset compensation, true enables it.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t digitalOffsetCompensation(
        ER4CL_ARGIN uint16_t channelIdx,
        ER4CL_ARGIN bool on);

/*! \brief Activate autostop feature for digital offset compensation.
 *
 * \param on [in] False disables the digital offset compensation's autostop, true enables it.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t digitalOffsetCompensationAutostop(
        ER4CL_ARGIN bool on);

/*! \brief Zap.
 * A big voltage is applied in order to break the membrane.
 *
 * \param channelIdx [in] Index of the channel to zap.
 * Set equal to the number of current channels to apply to all channels.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t zap(
        ER4CL_ARGIN uint16_t channelIdx);

/*! \brief Channel on.
 * Switch on the channel. Switched off channel do not send data and are constantly compensated to reduce current offset.
 *
 * \param channelIdx [in] Index of the channel to switch on.
 * Set equal to the number of current channels to apply to all channels.
 * \param on [in] False switches off the channel, true switches it on.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t switchChannelOn(
        ER4CL_ARGIN uint16_t channelIdx,
        ER4CL_ARGIN bool on);

/*! \brief Set the voltage of a waveform 1 item for the fast pulses protocol.
 *
 * \param idx [in] Index of the time set.
 * \param time [in] Value of the time set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage);

/*! \brief Set the duration of a waveform 1 item for the fast pulses protocol.
 *
 * \param idx [in] Index of the voltage set.
 * \param voltage [in] Value of the voltage set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setFastReferencePulseProtocolWave1Time(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t time);

/*! \brief Set the voltage of a waveform 2 item for the fast pulses protocol.
 *
 * \param idx [in] Index of the time set.
 * \param time [in] Value of the time set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t voltage);

/*! \brief Set the waiting time of a waveform 2 item for the fast pulses protocol.
 *
 * \param idx [in] Index of the time set.
 * \param time [in] Value of the time set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setFastReferencePulseProtocolWave2Time(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t time);

/*! \brief Set the duration of a waveform 2 item for the fast pulses protocol.
 *
 * \param idx [in] Index of the time set.
 * \param time [in] Value of the time set.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setFastReferencePulseProtocolWave2Duration(
        ER4CL_ARGIN unsigned int idx,
        ER4CL_ARGIN MeasurementReduced_t time);

ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t switchVcSel0(
        ER4CL_ARGIN bool on);

ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t switchVcSel1(
        ER4CL_ARGIN bool on);

/*! \brief Turn on the digital output.
 *
 * \param on [in] True to turn on the digital output, false to turn it off.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t turnOnDigitalOutput(
        ER4CL_ARGIN bool on);

/*! \brief Turn on/off a specific LED.
 *
 * \param ledIndex [in] Index of the LED to turn on/off.
 *        See the device documentation for an enumeration of the single LEDs.
 * \param on [in] True to turn the LED on, false to turn it off.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t turnLedOn(
        ER4CL_ARGIN uint16_t ledIndex,
        ER4CL_ARGIN bool on);

/*! \brief Enable the front end reset denoiser.
 *
 * \param on [in] True to enable, false to disable.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t enableFrontEndResetDenoiser(
        ER4CL_ARGIN bool on);

/*! \brief Reset the device.
 *
 * \param reset [in] False sets the device in normal operation state, true sets in reset state.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t resetDevice(
        ER4CL_ARGVOID);

/*! \brief Select the channel for compesantions settings.
 *  Call this method before other compensations control methods, in order to select which channel
 *  those methods should apply to.
 *
 * \param channelIdx [in] Channel index that compensations methods will be applied to.
 * \return Error code.
 */
ErrorCodes_t setCompensationsChannel(
        ER4CL_ARGIN uint16_t channelIdx);

/*! \brief Turn on/off cFast compensation.
 *
 * \param on [in] True to turn the cFast compensation on, false to turn it off.
 * \return Error code.
 */
ErrorCodes_t turnCFastCompensationOn(
        ER4CL_ARGIN bool on);

/*! \brief Set options for cFast compensation (voltage clamp).
 *
 * \param optionIdx [in] Option index.
 * \return Error code.
 */
ErrorCodes_t setCFastCompensationOptions(
        ER4CL_ARGIN uint16_t optionIdx);

/*! \brief Sets the value of the cFast capacitance.
 *
 * \param value [in] Value of the cFast capacitance.
 * \return Error code.
 */
ErrorCodes_t setCFastCapacitance(
        ER4CL_ARGIN MeasurementReduced_t value);

///*! \brief Reset the device's digital offset compensation.
// *
// * \param reset [in] False sets the digital offset compensation in normal operation state, true sets in reset state.
// * \return Error code.
// */
//ErrorCodes_t resetDigitalOffsetCompensation(
//        ER4CL_ARGIN bool reset);

/*! \brief Set a bit of the communication protocol for debug purposes.
 *
 * \param byteOffset [in] Offset of the byte the bit belongs to.
 * \param bitOffset [in] Offset of the bit within the byte.
 * \param status [in] Status of the bit.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setDebugBit(
        ER4CL_ARGIN uint16_t byteOffset,
        ER4CL_ARGIN uint16_t bitOffset,
        ER4CL_ARGIN bool status);

/*! \brief Set a byte of the communication protocol for debug purposes.
 *
 * \param byteOffset [in] Offset of the byte to set.
 * \param byteValue [in] Value of the byte.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t setDebugByte(
        ER4CL_ARGIN uint16_t byteOffset,
        ER4CL_ARGIN uint16_t byteValue);

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
//        ER4CL_ARGOUT std::string &upgradeNotes,
//        ER4CL_ARGOUT std::string &notificationTag);

/*! \brief Get the device identification information (to be used when the device is already connected).
 *
 * \param deviceVersion [out] Device version.
 * \param deviceSubversion [out] Device subversion.
 * \param firmwareVersion [out] Firmware version.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getDeviceInfoAlreadyConnect(
        ER4CL_ARGOUT uint8_t &deviceVersion,
        ER4CL_ARGOUT uint8_t &deviceSubversion,
        ER4CL_ARGOUT uint32_t &firmwareVersion);

/*! \brief Get the device identification information (to be used when the device is not connected yet).
 *
 * \param deviceId [in] Device identification code (S/N).
 * \param deviceVersion [out] Device version.
 * \param deviceSubversion [out] Device subversion.
 * \param firmwareVersion [out] Firmware version.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getDeviceInfo(
        ER4CL_ARGOUT char * deviceId,
        ER4CL_ARGOUT uint8_t &deviceVersion,
        ER4CL_ARGOUT uint8_t &deviceSubversion,
        ER4CL_ARGOUT uint32_t &firmwareVersion);

/*! \brief Returns information on the queue status.
 * Calling this method if no device is connected will return an error code.
 *
 * \param status [out] Struct containing queue information.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getQueueStatus(
        ER4CL_ARGOUT QueueStatus_t &status);

/*! \brief Reads data packets from the device into a buffer.
 * The returned buffer contains \a dataRead valid data packets of with one sample for each channel.
 * \a buffer is allocated by method init() with #ER4CL_DATA_ARRAY_SIZE items and freed by method deinit().\n
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
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t readData(
        ER4CL_ARGIN unsigned int dataToRead,
        ER4CL_ARGOUT unsigned int &dataRead,
        ER4CL_ARGOUT int16_t buffer[]);

/*! \brief Converts an integer number to the corresponding voltage value.
 * The converted voltage's unit depends on the device configuration.
 *
 * \param intValue [in] Integer value to be converted.
 * \param fltValue [out] Floating point voltage converted value.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t convertVoltageValue(
        ER4CL_ARGIN uint16_t intValue,
        ER4CL_ARGOUT double &fltValue);

/*! \brief Converts an integer number to the corresponding current value.
 * The converted current's unit depends on the device configuration.
 *
 * \param intValue [in] Integer value to be converted.
 * \param channelIdx [in] Index of the channel to be converted.
 * \param fltValue [out] Floating point current converted value.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t convertCurrentValue(
        ER4CL_ARGIN uint16_t intValue,
        ER4CL_ARGIN uint16_t channelIdx,
        ER4CL_ARGOUT double &fltValue);

/*! \brief Purges data read from the device.
 * This command is useful to get rid of data acquired during the device configuration (e.g. during setting of sampling rate or digital offset compensation).
 * Calling this method if no device is connected will return an error code.
 *
 * \return #EdlErrorCode_t Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t purgeData(ER4CL_ARGVOID);

/*! \brief Get the number of channels for the device.
 *
 * \param voltageChannelsNum [out] Number of voltage channels.
 * \param currentChannelsNum [out] Number of current channels (each current channel is reutrned raw, filtered and averaged).
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getChannelsNumber(
        ER4CL_ARGOUT uint32_t &voltageChannelsNum,
        ER4CL_ARGOUT uint32_t &currentChannelsNum);

/*! \brief Get the current ranges available in voltage clamp for the device.
 *
 * \param currentRanges [out] Array containing all the available current ranges in voltage clamp.
 * \param defaultValue [out] Default options.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getCurrentRanges(
        ER4CL_ARGOUT RangedMeasurementReduced_t currentRanges[],
        ER4CL_ARGOUT uint16_t defaultOptions[]);

/*! \brief Get the current range currently applied on a given channel.
 *
 * \param currentRange [out] Current range currently applied.
 * \param channelIdx [in] Channel index.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getCurrentRange(
        ER4CL_ARGOUT RangedMeasurementReduced_t &currentRange,
        ER4CL_ARGIN uint16_t channelIdx);

/*! \brief Check if the device can set the current range independently on each channel.
 *
 * \return Success if the device can set independently the current ranges for each channel.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasIndependentCurrentRanges(
        ER4CL_ARGVOID);

/*! \brief Get the voltage ranges available in voltage clamp for the device.
 *
 * \param voltageRanges [out] Array containing all the available voltage ranges in voltage clamp.
 * \param defaultValue [out] Default option.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageRanges(
        ER4CL_ARGOUT RangedMeasurementReduced_t voltageRanges[],
        ER4CL_ARGOUT uint16_t &defaultOption);

/*! \brief Get the voltage range currently applied.
 *
 * \param voltageRange [out] Voltage range currently.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageRange(
        ER4CL_ARGOUT RangedMeasurementReduced_t &voltageRange);

/*! \brief Get the voltage ranges available for the reference.
 *
 * \param voltageRanges [out] Array containing all the available voltage ranges for the reference.
 * \param defaultValue [out] Default option.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageReferenceRanges(
        ER4CL_ARGOUT RangedMeasurementReduced_t ranges[],
        ER4CL_ARGOUT uint16_t &defaultOption);

/*! \brief Get the sampling rates available for the device.
 *
 * \param samplingRates [out] Array containing all the available sampling rates.
 * \param defaultValue [out] Default option.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getSamplingRates(
        ER4CL_ARGOUT MeasurementReduced_t * &samplingRates,
        ER4CL_ARGOUT uint16_t &defaultOption);

/*! \brief Get the sampling rate currently applied.
 *
 * \param samplingRate [out] Sampling rate currently applied.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getSamplingRate(
        ER4CL_ARGOUT MeasurementReduced_t &samplingRate);

/*! \brief Get the real sampling rates available for the device.
 *
 * \param samplingRates [out] Array containing all the available real sampling rates
 *                            (may slightly differ from displayed sampling rates).
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getRealSamplingRates(
        ER4CL_ARGOUT MeasurementReduced_t * &samplingRates);

/*! \brief Get the real sampling rate currently applied.
 *
 * \param samplingRate [out] Real sampling rate currently applied.
 *                           (may slightly differ from displayed sampling rate).
 * \return Error code.
 */
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getRealSamplingRate(
        ER4CL_ARGOUT Measurement_t &samplingRate);

/*! \brief Get the oversampling ratios available for the device.
 *
 * \param oversamplingRatios [out] Array containing all the available oversampling ratios.
 * \return Success only if at least one ratio other than 1 is available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getOversamplingRatios(
        ER4CL_ARGOUT uint16_t &oversamplingRatios);

/*! \brief Get the oversampling ratio currently applied.
 *
 * \param oversamplingRatio [out] Oversampling ratio currently applied.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getOversamplingRatio(
        ER4CL_ARGOUT uint16_t &oversamplingRatio);

/*! \brief Get the available options for the voltage stimulus low pass filter.
 *
 * \param filterOptions [out] Available options for the voltage stimulus low pass filter.
 * \param defaultOption [out] Option selected by default.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageStimulusLpfs(
        ER4CL_ARGOUT MeasurementReduced_t * &filterOptions,
        ER4CL_ARGOUT uint16_t &defaultOption);

/*! \brief Get the available options for the voltage reference low pass filter.
 *
 * \param filterOptions [out] Available options for the voltage reference low pass filter.
 * \param defaultOption [out] Option selected by default.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageReferenceLpfs(
        ER4CL_ARGOUT MeasurementReduced_t * &filterOptions,
        ER4CL_ARGOUT uint16_t &defaultOption);

/*! \brief Get the select channels for voltage stimulation feature availability.
 *
 * \param selectStimulusChannelFlag [out] True if the device has the ability to apply voltage stimulation to selected channels.
 * \param singleChannelSSCFlag [out] True if the device can select single channels independently for voltage stimulation.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasSelectStimulusChannel(
        ER4CL_ARGOUT bool &selectStimulusChannelFlag,
        ER4CL_ARGOUT bool &singleChannelSSCFlag);

/*! \brief Get the digital offset compensation feature availability.
 *
 * \param digitalOffsetCompensationFlag [out] True if the device has the digital offset compensation feature.
 * \param singleChannelDOCFlag [out] True if the device can apply digital offset compesantion to single channels independently.
 * \param selectableDOCAutostop [out] True if the device can select the autostop feature for the digital offset compesantion.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasDigitalOffsetCompensation(
        ER4CL_ARGOUT bool &digitalOffsetCompensationFlag,
        ER4CL_ARGOUT bool &singleChannelDOCFlag,
        ER4CL_ARGOUT bool &selectableDOCAutostopFlag);

/*! \brief Get the zap feature availability.
 *
 * \param zappableDeviceFlag [out] True if the device has the zap feature.
 * \param singleChannelZapFlag [out] True if the device can zap single channels independently.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasZap(
        ER4CL_ARGOUT bool &zappableDeviceFlag,
        ER4CL_ARGOUT bool &singleChannelZapFlag);

/*! \brief Get the switch on feature availability.
 *
 * \param channelOnFlag [out] True if the device has the switch channel on feature.
 * \param singleChannelOnFlag [out] True if the device can switch on single channels independently.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasChannelOn(
        ER4CL_ARGOUT bool &channelOnFlag,
        ER4CL_ARGOUT bool &singleChannelOnFlag);

/*! \brief Get the digital output availability.
 *
 * \return Return an error code if the feature is not available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasDigitalOutput(
        ER4CL_ARGVOID);

/*! \brief Get the front end reset denoiser feature availability.
 *
 * \return Return an error code if the feature is not available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasFrontEndResetDenoiser(
        ER4CL_ARGVOID);

/*! \brief Get protocols list.
 *
 * \param names [out] Names of available protocols.
 * \param images [out] Strings to use in order to load protocols images.
 * \param voltages [out] Indexes of available voltage controls for each protocol.
 * \param times [out] Indexes of available time controls for each protocol.
 * \param slopes [out] Indexes of available slope controls for each protocol.
 * \param adimensionals [out] Indexes of available adimensional controls for each protocol.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolList(
        ER4CL_ARGOUT char names[],
        ER4CL_ARGOUT char images[],
        ER4CL_ARGOUT int16_t voltages[],
        ER4CL_ARGOUT int16_t times[],
        ER4CL_ARGOUT int16_t slopes[],
        ER4CL_ARGOUT int16_t adimensionals[]);

/*! \brief Get triangular protocol index.
 *
 * \param idx [out] Index of the triangular protocol.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getTriangularProtocolIdx(
        ER4CL_ARGOUT uint16_t &idx);

/*! \brief Get seal test protocol index.
 *
 * \param idx [out] Index of the seal test protocol.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getSealTestProtocolIdx(
        ER4CL_ARGOUT uint16_t &idx);

/*! \brief Get protocol applicable voltage range.
 *
 * \param voltageNames [out] Names of available voltages.
 * \param ranges [out] Ranges of applicable voltage in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolVoltage(
        ER4CL_ARGOUT char voltageNames[],
        ER4CL_ARGOUT RangedMeasurementReduced_t ranges[],
        ER4CL_ARGOUT MeasurementReduced_t  defaultValues[]);

/*! \brief Get protocol applicable time range.
 *
 * \param timeNames [out] Names of available times.
 * \param ranges [out] Ranges of applicable time in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolTime(
        ER4CL_ARGOUT char* timeNames[],
        ER4CL_ARGOUT RangedMeasurementReduced_t ranges[],
        ER4CL_ARGOUT MeasurementReduced_t defaultValues[]);

/*! \brief Get protocol applicable slope range.
 *
 * \param slopeNames [out] Names of available slopes.
 * \param ranges [out] Ranges of applicable slope in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolSlope(
        ER4CL_ARGOUT char slopeNames[],
        ER4CL_ARGOUT RangedMeasurementReduced_t ranges[],
        ER4CL_ARGOUT MeasurementReduced_t defaultValues[]);

/*! \brief Get protocol applicable adimensional range.
 *
 * \param adimensionalNames [out] Names of available adimensionals.
 * \param ranges [out] Ranges of applicable adimensional in protocols.
 * \param defaultValues [out] Default values.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getProtocolAdimensional(
        ER4CL_ARGOUT char adimensionalNames[],
        ER4CL_ARGOUT RangedMeasurementReduced_t ranges[],
        ER4CL_ARGOUT MeasurementReduced_t defaultValues[]);

/*! \brief Availability of single channels voltage offset controls.
 *
 * \param voltageRange [out] Range of applicable voltage offset.
 * \return Success if the voltage offsets of single channels can be controlled.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageOffsetControls(
        ER4CL_ARGOUT RangedMeasurementReduced_t * &voltageRange);

/*! \brief Get insertion pulse controls definition.
 *
 * \param voltageRange [out] Range of applicable pulse voltage.
 * \param durationRange [out] Ranges of applicable pulse duration.
 * \return Success if the device has the insertion pulse feature.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getInsertionPulseControls(
        ER4CL_ARGOUT RangedMeasurementReduced_t &voltageRange,
        ER4CL_ARGOUT RangedMeasurementReduced_t &durationRange);

/*! \brief Get reference pulse controls definition.
 *
 * \param voltageRange [out] Range of applicable pulse voltage.
 * \param durationRange [out] Ranges of applicable pulse duration.
 * \return Success if the device has the reference pulse feature.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getReferencePulseControls(
        ER4CL_ARGOUT RangedMeasurementReduced_t &voltageRange,
        ER4CL_ARGOUT RangedMeasurementReduced_t &durationRange);

/*! \brief Get reference pulse controls definition.
 *
 * \param voltageRange [out] Range of applicable pulse voltage.
 * \param durationRange [out] Ranges of applicable pulse duration.
 * \return Success if the device has the reference pulse feature.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasReferencePulseControls(
        ER4CL_ARGOUT bool referencePulseImplemented,
        ER4CL_ARGOUT bool overrideReferencePulseImplemented);


/*! \brief Get data header format.
 *
 * \param format [out] Format of the data header.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getEdhFormat(
        ER4CL_ARGOUT char* &format);

/*! \brief Get the raw data filter applicable cut off frequency range.
 *
 * \param range [out] Range of applicable cut off frequency of the raw data filter.
 * \param defaultValue [out] Default value.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getRawDataFilterCutoffFrequency(
        ER4CL_ARGOUT RangedMeasurementReduced_t &range,
        ER4CL_ARGOUT MeasurementReduced_t &defaultValue);

/*! \brief Get the number of LEDs for the device.
 *
 * \param ledsNum [out] Number of LEDs.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getLedsNumber(
        ER4CL_ARGOUT uint16_t &ledsNum);

/*! \brief Get the LEDs colors for the device.
 *
 * \param ledsColors [out] Array containing the colors of the LEDs.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getLedsColors(
        ER4CL_ARGOUT uint32_t ledsColors);

/*! \brief Get the range currently applied and the number of pulse.
 *
 * \param voltageRange [out] Applicable voltage range.
 * \param timeRange [out] Applicable voltage range.
 * \param nPulse [out] number of pulse.
 * \return Success if external DAC control is available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getFastReferencePulseProtocolWave1Range(
        ER4CL_ARGOUT RangedMeasurementReduced_t &voltageRange,
        ER4CL_ARGOUT RangedMeasurementReduced_t &timeRange,
        ER4CL_ARGOUT uint16_t &nPulse);

/*! \brief Get the range currently applied and the number of pulse.
 *
 * \param voltageRange [out] Applicable voltage range.
 * \param timeRange [out] Applicable time range.
 * \param durationRange [out] Applicable duration range.
 * \param nPulse [out] number of pulse.
 * \return Success if external DAC control is available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getFastReferencePulseProtocolWave2Range(
        ER4CL_ARGOUT RangedMeasurementReduced_t &voltageRange,
        ER4CL_ARGOUT RangedMeasurementReduced_t &timeRange,
        ER4CL_ARGOUT RangedMeasurementReduced_t &durationRange,
        ER4CL_ARGOUT uint16_t &nPulse);

/*! \brief Availability of Nanion's temperature controller.
 *
 * \return Success if the Nanion's temperature controller is available with this device.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasNanionTemperatureController(
        ER4CL_ARGVOID);

/*! \brief Get Nanion's temperature controller temeprature range.
 *
 * \param minTemperature [out] Minimum applicable temperatures in °C.
 * \param maxTemperature [out] Minimum applicable temperatures in °C.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getTemperatureControllerRange(
        ER4CL_ARGOUT int &minTemperature,
        ER4CL_ARGOUT int &maxTemperature);

/*! \brief Availability of Orbit washer controls.
 *
 * \return Success if the Orbit washer control is available.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t hasWasherControls(
        ER4CL_ARGVOID);

/*! \brief Get Orbit washer's speed range.
 *
 * \param range [out] Range of applicable cut off frequency of the raw data filter.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getWasherSpeedRange(
        ER4CL_ARGOUT RangedMeasurementReduced_t &range);

/*! \brief Get Orbit washer's status.
 *
 * \param status [out] Status code.
 * \param error [out] Error code.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getWasherStatus(
        ER4CL_ARGOUT WasherStatus_t &status,
        ER4CL_ARGOUT WasherError_t &error);

/*! \brief Get Orbit washer's preset speeds.
 *
 * \param speedValue [out] vector of preset speeds.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getWasherPresetSpeeds(
        ER4CL_ARGOUT int8_t* &speedValue);

/*! \brief Tell if the device implements CFast compensation.
 *
 * \return Success if the device implements CFast compensation.
 */
ErrorCodes_t hasCFastCompensation(
        ER4CL_ARGVOID);

/*! \brief Get options for the CFast compensation.
 *
 * \param option [out]: vector of strings of the available options.
 * \return Success if the device has options for CFast compensation.
 */
ErrorCodes_t getCFastCompensationOptions(
        ER4CL_ARGOUT char* &options);

/*! \brief Get the specifications of the control for the CFast capacitance.
 *
 * \param control [in] Specifications of the control for the CFast capacitance.
 * \return Success if the device implements CFast capacitance control.
 */
ErrorCodes_t getCFastCapacitanceControl(
        ER4CL_ARGOUT CompensationControl_t &control);

/*! \brief Get the voltage offset compensated with the digital compensation.
 *
 * \param offsets [out] Vector of applied offsets.
 * \return Error code.
 */
ER4COMMLIB_NAME_MANGLING
ER4COMMLIBSHARED_EXPORT
ErrorCodes_t getVoltageOffsetCompensations(
        ER4CL_ARGOUT MeasurementReduced_t* &offsets);


#endif // ER4COMMLIB_H
