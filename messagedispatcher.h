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

#ifndef MESSAGEDISPATCHER_H
#define MESSAGEDISPATCHER_H


#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <unordered_map>

#include "er4commlib_errorcodes.h"
#include "er4commlib_global.h"
#include "ftdieeprom.h"
#include "ftdieeprom56.h"
#include "ftdieepromdemo.h"
#include "commandcoder.h"

using namespace std;

//#define DEBUG_PRINT

#ifdef DEBUG_PRINT
//#define DEBUG_RAW_BIT_RATE_PRINT
#endif

#define SHORT_OFFSET_BINARY (static_cast <double> (0x8000))
#define SHORT_MAX (static_cast <double> (0x7FFF))
#define SHORT_MIN (-SHORT_MAX-1.0)
#define USHORT_MAX (static_cast <double> (0xFFFF))
#define UINT10_MAX (static_cast <double> (0x3FF))
#define INT14_MAX (static_cast <double> (0x1FFF))
#define INT18_MAX (static_cast <double> (0x1FFFF))
#define UINT28_MAX (static_cast <double> (0xFFFFFFF))
#define INT28_MAX (static_cast <double> (0x7FFFFFF))
#define INT28_MIN (-INT28_MAX-1.0)

#define UINT16_POSITIVE_SATURATION 0x7FFE
#define UINT16_NEGATIVE_SATURATION 0x8000 /*! \todo FCON dovrebbe essere 0x8001 */
#define UINT16_CURRENT_RANGE_INCREASE_MINIMUM_THRESHOLD 0x7800 /*!< Equivalent to 15/16 of range */
#define UINT16_CURRENT_RANGE_INCREASE_MAXIMUM_THRESHOLD 0x8800 /*!< Equivalent to -15/16 of range */
#define UINT16_CURRENT_RANGE_DECREASE_MINIMUM_THRESHOLD 0xF800 /*!< Equivalent to 1/16 of range */
#define UINT16_CURRENT_RANGE_DECREASE_MAXIMUM_THRESHOLD 0x0800 /*!< Equivalent to -1/16 of range */

#define LSB_NOISE_ARRAY_SIZE 0x40000 /*! \todo FCON valutare che questo numero sia adeguato */ // ~250k
#define LSB_NOISE_ARRAY_MASK (LSB_NOISE_ARRAY_SIZE-1) // 0b11...1 for all bits of the array indexes

#define IIR_ORD 2
#define IIR_2_SIN_PI_4 (2.0*sin(M_PI/4.0))
#define IIR_2_COS_PI_4 (2.0*cos(M_PI/4.0))
#define IIR_2_COS_PI_4_2 (IIR_2_COS_PI_4*IIR_2_COS_PI_4)

#define FTD_MAX_RESEND_TRIES 10
#define FTD_MAX_FPGA_RESET_TRIES 10
#define FTD_MAX_WRITE_TRIES 10

#define FTD_RX_WORD_SIZE (sizeof(uint16_t)) // 16 bit word
#define FTD_RX_SYNC_WORD_SIZE (static_cast <int> (2*FTD_RX_WORD_SIZE))
#define FTD_RX_INFO_WORD_SIZE (static_cast <int> (FTD_RX_WORD_SIZE))
#define FTD_RX_BUFFER_SIZE 0x100000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
#define FTD_RX_BUFFER_MASK (FTD_RX_BUFFER_SIZE-1)
#define FTD_DEFAULT_MIN_READ_FRAME_NUMBER 10
#define FTD_DEFAULT_MIN_STORE_FRAME_NUMBER 10
#define FTD_DEFAULT_FEW_FRAME_SLEEP 2
#define FTD_FEW_PACKET_COEFF 0.05 /*!< = 50.0/1000.0: 50.0 because I want to get data once every 50ms, 1000 to convert sampling rate from Hz to kHz */
#define FTD_MAX_BYTES_TO_WAIT_FOR 2048 /*! Max FTDI buffer size = 4k, so wait no more than half full to read it */

#define ER4CL_OUTPUT_BUFFER_SIZE 0x100000 /*!< Always use a power of 2 for efficient circular buffer management through index masking */
#define ER4CL_OUTPUT_BUFFER_MASK (ER4CL_OUTPUT_BUFFER_SIZE-1)

#define FTD_TX_WORD_SIZE (sizeof(uint8_t)) // 1 bite word
#define FTD_TX_SYNC_WORD_SIZE (FTD_TX_WORD_SIZE) // could be different size
#define FTD_TX_RAW_BUFFER_SIZE 0x1000 /*! \todo FCON valutare che questo numero sia adeguato */ // ~4k
#define FTD_TX_RAW_BUFFER_MASK (FTD_TX_RAW_BUFFER_SIZE-1) // 0b11...1 for all bits of the buffer indexes
#define FTD_TX_MSG_BUFFER_SIZE 0x100 /*! \todo FCON valutare che questo numero sia adeguato */ // 256
#define FTD_TX_MSG_BUFFER_MASK (FTD_TX_MSG_BUFFER_SIZE-1)

typedef struct Measurement{
    double value; /*!< Numerical value. */
    UnitPfx_t prefix; /*!< Unit prefix in the range [femto, Peta]. */
    char * unit; /*!< Unit. \note Can be any string, the library is not aware of real units meaning. */

    /*! \brief Returns the value without prefix.
     *
     * \return Value without prefix, e.g. for 1.5nA returns 1.5e-9.
     */
    double getNoPrefixValue(ER4CL_ARGVOID) {
        return value*multiplier();
    }

    /*! \brief Returns the string corresponding to the prefix.
     *
     * \return String corresponding to the prefix.
     */
    char getPrefix(ER4CL_ARGVOID) {
        return unitPrefixes[prefix];
    }

    /*! \brief Returns the prefix multiplier.
     *
     * \return Prefix multiplier, e.g. 10^-6 for micro.
     */
    double multiplier(ER4CL_ARGVOID) {
        unsigned int delta;
        bool deltaPositive;

        if (prefix > UnitPfxNone) {
            delta = prefix-UnitPfxNone;
            deltaPositive = true;

        } else {
            delta = UnitPfxNone-prefix;
            deltaPositive = false;
        }

        if (deltaPositive) {
            return powersOf1000[delta];

        } else {
            return 1.0/powersOf1000[delta];
        }
    }

    /*! \brief Converts #value given the input unit prefix.
     *
     * \param newPrefix [in] Desired unit prefix.
     */
    void convertValue(ER4CL_ARGIN UnitPfx_t newPrefix) {
        unsigned int delta;
        bool deltaPositive;

        if (prefix > newPrefix) {
            delta = prefix-newPrefix;
            deltaPositive = true;

        } else {
            delta = newPrefix-prefix;
            deltaPositive = false;
        }

        if (deltaPositive) {
            value *= powersOf1000[delta];

        } else {
            value /= powersOf1000[delta];
        }
        prefix = newPrefix;
    }

    /*! \brief Converts #value given the input unit multiplier.
     *
     * \param newMultiplier [in] Desired unit multiplier.
     */
    void convertValueMult(ER4CL_ARGIN double newMultiplier) {
        double multiplier = this->multiplier();
        double gain;
        bool gainPositive;

        if (multiplier > newMultiplier) {
            gain = multiplier/newMultiplier;
            gainPositive = true;

        } else {
            gain = newMultiplier/multiplier;
            gainPositive = false;
        }

        double diff;
        double minDiff = (std::numeric_limits <double>::max)();
        unsigned int minDelta = 0;
        for (unsigned int delta = 0; delta < UnitPfxNum; delta++) {
            diff = fabs(powersOf1000[delta]-gain);
            if (diff < minDiff) {
                minDiff = diff;
                minDelta = delta;
            }
        }

        if (gainPositive) {
            value *= powersOf1000[minDelta];
            decrementUnit(prefix, (int)minDelta);

        } else {
            value /= powersOf1000[minDelta];
            incrementUnit(prefix, (int)minDelta);
        }
    }


} Measurement_t;

/*! \brief Overloaded equality check for #Measurement_t. \note No conversion is performed, cause the multiplication can introduce rounding errors.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a and \p b have same value, prefix and unit.
*/
inline bool operator == (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
    return ((a.value == b.value) && (a.prefix == b.prefix) /*&& (a.unit == b.unit)*/);
}

/*! \brief Overloaded inequality check for #Measurement_t. \note No conversion is performed, cause the multiplication can introduce rounding errors.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a and \p b have different value, prefix or unit.
*/
inline bool operator != (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
    return !(a == b);
}

/*! \brief Overloaded inequality check for #Measurement_t.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a is smaller than \p b after they have been converted to have the same prefix. \note Returns false if \p a and \p b have different units.
*/
inline bool operator < (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
    if (a.unit != b.unit) {
        return false;

    } else {
        /*! Not using convertValue method to avoid changing the input structures */
        if (a.prefix < b.prefix) {
            return a.value < (b.value*powersOf1000[b.prefix-a.prefix]);

        } else {
            return (a.value*powersOf1000[a.prefix-b.prefix]) < b.value;
        }
    }
}

/*! \brief Overloaded inequality check for #Measurement_t.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a is smaller than or equal to \p b after they have been converted to have the same prefix. \note Returns false if \p a and \p b have different units.
*/
inline bool operator <= (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
        /*! Not using convertValue method to avoid changing the input structures */
        if (a.prefix < b.prefix) {
            return a.value <= (b.value*powersOf1000[b.prefix-a.prefix]);

        } else {
            return (a.value*powersOf1000[a.prefix-b.prefix]) <= b.value;
        }
    }


/*! \brief Overloaded inequality check for #Measurement_t.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a is greater than \p b after they have been converted to have the same prefix. \note Returns false if \p a and \p b have different units.
*/
inline bool operator > (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {

        /*! Not using convertValue method to avoid changing the input structures */
        if (a.prefix < b.prefix) {
            return a.value > (b.value*powersOf1000[b.prefix-a.prefix]);

        } else {
            return (a.value*powersOf1000[a.prefix-b.prefix]) > b.value;
        }
    }


/*! \brief Overloaded inequality check for #Measurement_t.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a is greater than or equal to \p b after they have been converted to have the same prefix. \note Returns false if \p a and \p b have different units.
*/
inline bool operator >= (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
        /*! Not using convertValue method to avoid changing the input structures */
        if (a.prefix < b.prefix) {
            return a.value >= (b.value*powersOf1000[b.prefix-a.prefix]);

        } else {
            return (a.value*powersOf1000[a.prefix-b.prefix]) >= b.value;
        }
    }


/*! \brief Overloaded sum for #Measurement_t.
 *
 * \param a [in] First operand.
 * \param b [in] Second operand.
 * \return A #Measurement_t whose value is the sum of the values of the operands converted to the prefix of the first operand, and the unit equals the unit of the second operand.
 * \note This method assumes the units are compatible and won't check for the sake of speed.
*/
inline Measurement_t operator + (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
    Measurement_t c = b;
    c.convertValue(a.prefix);
    c.value += a.value;
    return c;
}

/*! \brief Overloaded subtraction for #Measurement_t.
 *
 * \param a [in] First operand.
 * \param b [in] Second operand.
 * \return A #Measurement_t whose value is the difference of the values of the operands converted to the prefix of the first operand, and the unit equals the unit of the second operand.
 * \note This method assumes the units are compatible and won't check for the sake of speed.
*/
inline Measurement_t operator - (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const Measurement_t &b) {
    Measurement_t c = b;
    c.convertValue(a.prefix);
    c.value = a.value-c.value;
    return c;
}

/*! \brief Overloaded multiplication between #Measurement_t and a constant.
 *
 * \param a [in] First operand.
 * \param b [in] Second operand.
 * \return A #Measurement_t whose value is the product of the values and the unit equals the unit of the first operand.
*/
template <class T>
inline Measurement_t operator * (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const T &b) {
    Measurement_t c = a;
    c.value *= (double)b;
    return c;
}

/*! \brief Overloaded multiplication between #Measurement_t and a constant.
 *
 * \param a [in] First operand.
 * \param b [in] Second operand.
 * \return A #Measurement_t whose value is the product of the values and the unit equals the unit of the second operand.
*/
template <class T>
inline Measurement_t operator * (ER4CL_ARGIN const T &a, ER4CL_ARGIN const Measurement_t &b) {
    return b*a;
}

/*! \brief Overloaded division between #Measurement_t and a constant.
 *
 * \param a [in] First operand.
 * \param b [in] Second operand.
 * \return A #Measurement_t whose value is the ratio of the values and the unit equals the unit of the first operand.
*/
template <class T>
inline Measurement_t operator / (ER4CL_ARGIN const Measurement_t &a, ER4CL_ARGIN const T &b) {
    Measurement_t c = a;
    c.value /= (double)b;
    return c;
}

/*! \struct RangedMeasurement_t
 * \brief Structure used manage physical ranges that define a range with its unit and unit prefix.
 */
typedef struct RangedMeasurement{
    double min; /*!< Minimum value. */
    double max; /*!< Maximum value. */
    double step; /*!< Resolution. */
    UnitPfx_t prefix = UnitPfxNone; /*!< Unit prefix in the range [femto, Peta]. */

    /*! \brief Returns the string corresponding to the prefix.
     *
     * \return String corresponding to the prefix.
     */
    char getPrefix(ER4CL_ARGVOID) {
        return unitPrefixes[prefix];
    }

    /*! \brief Returns the prefix multiplier.
     *
     * \return Prefix multiplier, e.g. 10^-6 for micro.
     */
    double multiplier(ER4CL_ARGVOID) {
        unsigned int delta;
        bool deltaPositive;

        if (prefix > UnitPfxNone) {
            delta = prefix-UnitPfxNone;
            deltaPositive = true;

        } else {
            delta = UnitPfxNone-prefix;
            deltaPositive = false;
        }

        if (deltaPositive) {
            return powersOf1000[delta];

        } else {
            return 1.0/powersOf1000[delta];
        }
    }

    /*! \brief Converts #min, #max and #step given the input unit prefix.
     *
     * \param newPrefix [in] Desired unit prefix.
     */
    void convertValues(ER4CL_ARGIN UnitPfx_t newPrefix) {
        unsigned int delta;
        bool deltaPositive;

        if (prefix > newPrefix) {
            delta = prefix-newPrefix;
            deltaPositive = true;

        } else {
            delta = newPrefix-prefix;
            deltaPositive = false;
        }

        if (deltaPositive) {
            min *= powersOf1000[delta];
            max *= powersOf1000[delta];
            step *= powersOf1000[delta];

        } else {
            min /= powersOf1000[delta];
            max /= powersOf1000[delta];
            step /= powersOf1000[delta];
        }
        prefix = newPrefix;
    }

    /*! \brief Converts #min, #max and #step given the input unit multiplier.
     *
     * \param newMultiplier [in] Desired unit multiplier.
     */
    void convertValuesMult(ER4CL_ARGIN double newMultiplier) {
        double multiplier = this->multiplier();
        double gain;
        bool gainPositive;

        if (multiplier > newMultiplier) {
            gain = multiplier/newMultiplier;
            gainPositive = true;

        } else {
            gain = newMultiplier/multiplier;
            gainPositive = false;
        }

        double diff;
        double minDiff = (std::numeric_limits <double>::max)();
        unsigned int minDelta = 0;
        for (unsigned int delta = 0; delta < UnitPfxNum; delta++) {
            diff = fabs(powersOf1000[delta]-gain);
            if (diff < minDiff) {
                minDiff = diff;
                minDelta = delta;
            }
        }

        if (gainPositive) {
            min *= powersOf1000[minDelta];
            max *= powersOf1000[minDelta];
            step *= powersOf1000[minDelta];
            decrementUnit(prefix, (int)minDelta);

        } else {
            min /= powersOf1000[minDelta];
            max /= powersOf1000[minDelta];
            step /= powersOf1000[minDelta];
            incrementUnit(prefix, (int)minDelta);
        }
    }

    /*! \brief Returns the range width.
     *
     * \return Difference between max and min.
     */
    double delta(ER4CL_ARGVOID) {
        return max-min;
    }

    /*! \brief Returns a reasonable amount of decimals to represent the values in the range.
     *
     * \return A reasonable amount of decimals to represent the values in the range.
     */
    int decimals(ER4CL_ARGVOID) {
        int decimals = 0;
        double temp = step;
        while ((fabs(temp-round(temp)) > 0.05 || temp < 1.0) &&
               decimals < 3) { /*!< \todo Rather than asking 3 decimals better asking for 3 digits */
            decimals++;
            temp *= 10.0;
        }
        return decimals;
    }

    /*! \brief Returns a Measurement_t equivalent to the max value of the range.
     *
     * \return Measurement_t equivalent to the max value of the range.
     */
    Measurement_t getMax(ER4CL_ARGVOID) {
        Measurement_t extreme;
        extreme.value = max;
        extreme.prefix = prefix;
        return extreme;
    }

    /*! \brief Returns a Measurement_t equivalent to the min value of the range.
     *
     * \return Measurement_t equivalent to the min value of the range.
     */
    Measurement_t getMin(ER4CL_ARGVOID) {
        Measurement_t extreme;
        extreme.value = min;
        extreme.prefix = prefix;
        return extreme;
    }

    /*! \brief Checks if a Measurement_t is within the range.
     *
     * \return true if the Measurement_t is within the range, false otherwise.
     */
    bool includes(ER4CL_ARGIN const Measurement_t value) {
        if (value >= getMin() && value <= getMax()) {
            return true;

        } else {
            return false;
        }
    }
} RangedMeasurement_t;

/*! \brief Overloaded equality check for #RangedMeasurement_t. \note No conversion is performed, cause the multiplication can introduce rounding errors.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a and \p b have same value, prefix and unit.
*/
inline bool operator == (const RangedMeasurement_t &a, const RangedMeasurement_t &b) {
    return ((a.min == b.min) && (a.max == b.max) && (a.step == b.step) && (a.prefix == b.prefix) /*&& (a.unit == b.unit)*/);
}

/*! \brief Overloaded inequality check for #RangedMeasurement_t. \note No conversion is performed, cause the multiplication can introduce rounding errors.
 *
 * \param a [in] First item of the comparison.
 * \param b [in] Second item of the comparison.
 * \return true if \p a and \p b have different value, prefix or unit.
*/
inline bool operator != (const RangedMeasurement_t &a, const RangedMeasurement_t &b) {
    return !(a == b);
}

MeasurementReduced_t toReduceMeasurement(Measurement_t);
RangedMeasurementReduced_t toReduceRangedMeasurement(RangedMeasurement_t);
Measurement_t fromReduceMeasurement(MeasurementReduced_t);
RangedMeasurement_t fromReduceRangedMeasurement(RangedMeasurementReduced_t);


/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcher                                        *
 *                                                                                          *
\********************************************************************************************/

class MessageDispatcher {
public:

    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher(string deviceId);
    virtual ~MessageDispatcher();

    /************************\
     *  Connection methods  *
    \************************/

    ErrorCodes_t init();
    ErrorCodes_t deinit();
    virtual ErrorCodes_t connect(FtdiEeprom * ftdiEeprom);
    virtual ErrorCodes_t disconnect();
    virtual ErrorCodes_t pauseConnection(bool pauseFlag);
    void readDataFromDevice();
    void sendCommandsToDevice();

    static ErrorCodes_t getDeviceType(DeviceTuple_t tuple, DeviceTypes_t &type);

    /****************\
     *  Tx methods  *
    \****************/

    ErrorCodes_t turnOnLsbNoise(bool flag);
    ErrorCodes_t convertVoltageValue(uint16_t uintValue, double &fltValue);
    ErrorCodes_t convertCurrentValue(uint16_t uintValue, uint16_t channelIdx, double &fltValue);
    ErrorCodes_t setVoltageRange(uint16_t voltageRangeIdx, bool applyFlag = true);
    ErrorCodes_t setVoltageReferenceRange(uint16_t voltageRangeIdx, bool applyFlag = true);
    virtual ErrorCodes_t setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool applyFlag = true);
    virtual ErrorCodes_t setSamplingRate(uint16_t samplingRateIdx, bool applyFlag = true);
    virtual ErrorCodes_t setOversamplingRatio(uint16_t oversamplingRatioIdx, bool applyFlag = true);

    ErrorCodes_t setVoltageStimulusLpf(uint16_t filterIdx, bool applyFlag = true);
    ErrorCodes_t setVoltageReferenceLpf(uint16_t filterIdx, bool applyFlag = true);

    virtual ErrorCodes_t selectStimulusChannel(uint16_t channelIdx, bool on, bool applyFlag = true);
    virtual ErrorCodes_t digitalOffsetCompensation(uint16_t channelIdx, bool on, bool applyFlag = true);
    virtual ErrorCodes_t digitalOffsetCompensationAutostop(bool on, bool applyFlag = true);
    ErrorCodes_t zap(uint16_t channelIdx, bool applyFlag = true);
    ErrorCodes_t switchChannelOn(uint16_t channelIdx, bool on, bool applyFlag = true);
    ErrorCodes_t switchVcSel0(bool on);
    ErrorCodes_t switchVcSel1(bool on);

    ErrorCodes_t turnOnDigitalOutput(bool on);
    ErrorCodes_t turnLedOn(uint16_t ledIndex, bool on);
    ErrorCodes_t enableFrontEndResetDenoiser(bool on);

    ErrorCodes_t resetDevice();
    ErrorCodes_t resetCalib();
    ErrorCodes_t resetDigitalOffsetCompensation(bool reset);

    ErrorCodes_t sendCommands();
    ErrorCodes_t selectVoltageProtocol(unsigned int idx, bool applyFlag = false);
    ErrorCodes_t applyVoltageProtocol();
    virtual ErrorCodes_t setProtocolVoltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setProtocolTime(unsigned int idx, Measurement_t time, bool applyFlag = false);
    ErrorCodes_t setProtocolSlope(unsigned int idx, Measurement_t slope, bool applyFlag = false);
    ErrorCodes_t setProtocolAdimensional(unsigned int idx, Measurement_t adimensional, bool applyFlag = false);
    ErrorCodes_t checkSelectedProtocol(unsigned int idx, string &message);
    ErrorCodes_t checkProtocolVoltage(unsigned int idx, Measurement_t voltage, string &message);
    ErrorCodes_t checkProtocolTime(unsigned int idx, Measurement_t time, string &message);
    ErrorCodes_t checkProtocolSlope(unsigned int idx, Measurement_t slope, string &message);
    ErrorCodes_t checkProtocolAdimensional(unsigned int idx, Measurement_t adimensional, string &message);
    ErrorCodes_t setVoltageOffset(unsigned int idx, Measurement_t voltage, bool applyFlag = true);
    ErrorCodes_t checkVoltageOffset(unsigned int idx, Measurement_t voltage, string &message);
    ErrorCodes_t applyInsertionPulse(Measurement_t voltage, Measurement_t duration);
    ErrorCodes_t applyReferencePulse(Measurement_t voltage, Measurement_t duration);
    ErrorCodes_t overrideReferencePulse(bool flag, bool applyFlag = true);

    ErrorCodes_t setRawDataFilter(Measurement_t cutoffFrequency, bool lowPassFlag, bool activeFlag);
    ErrorCodes_t applyDacExt(Measurement_t voltage, bool applyFlag = true);
    ErrorCodes_t setFastReferencePulseProtocolWave1Voltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setFastReferencePulseProtocolWave1Time(unsigned int idx, Measurement_t time, bool applyFlag = false);
    ErrorCodes_t setFastReferencePulseProtocolWave2Voltage(unsigned int idx, Measurement_t voltage, bool applyFlag = false);
    ErrorCodes_t setFastReferencePulseProtocolWave2Time(unsigned int idx, Measurement_t time, bool applyFlag = false);
    ErrorCodes_t setFastReferencePulseProtocolWave2Duration(unsigned int idx, Measurement_t time, bool applyFlag = false);

    /*! Device specific controls */

    virtual ErrorCodes_t resetWasherError();
    virtual ErrorCodes_t setWasherPresetSpeeds(vector <int8_t> speedValues);
    virtual ErrorCodes_t startWasher(uint16_t speedIdx);
    virtual ErrorCodes_t updateWasherState();
    virtual ErrorCodes_t updateWasherPresetSpeeds();

    ErrorCodes_t setCompensationsChannel(uint16_t channelIdx);
    ErrorCodes_t turnCFastCompensationOn(bool on);
    ErrorCodes_t setCFastCompensationOptions(uint16_t optionIdx);
    ErrorCodes_t setCFastCapacitance(Measurement_t capacitance);

    ErrorCodes_t setDebugBit(uint16_t byteOffset, uint16_t bitOffset, bool status);
    ErrorCodes_t setDebugByte(uint16_t byteOffset, uint16_t byteValue);

    /****************\
     *  Rx methods  *
    \****************/

    ErrorCodes_t isDeviceUpgradable(string &upgradeNotes, string &notificationTag);
    ErrorCodes_t getDeviceInfo(uint8_t &deviceVersion, uint8_t &deviceSubversion, uint32_t &firmwareVersion);

    ErrorCodes_t getQueueStatus(QueueStatus_t &status);
    ErrorCodes_t getDataPackets(uint16_t * &data, unsigned int packetsNumber, unsigned int * packetsRead);
    ErrorCodes_t purgeData();

    ErrorCodes_t getChannelsNumber(uint32_t &voltageChannelsNumber, uint32_t &currentChannelsNumber);

    ErrorCodes_t getCurrentRanges(vector <RangedMeasurement_t> &currentRanges, vector <uint16_t> &defaultOptions);
    ErrorCodes_t getCurrentRange(RangedMeasurement_t &currentRange, uint16_t channelIdx);
    ErrorCodes_t hasIndependentCurrentRanges();

    ErrorCodes_t getVoltageRanges(vector <RangedMeasurement_t> &voltageRanges, uint16_t &defaultOption);
    ErrorCodes_t getVoltageRange(RangedMeasurement_t &voltageRange);
    ErrorCodes_t getVoltageReferenceRanges(vector <RangedMeasurement_t> &ranges, uint16_t &defaultOption);

    ErrorCodes_t getSamplingRates(vector <Measurement_t> &samplingRates, uint16_t &defaultOption);
    ErrorCodes_t getSamplingRate(Measurement_t &samplingRate);
    ErrorCodes_t getRealSamplingRates(vector <Measurement_t> &samplingRates);
    ErrorCodes_t getOversamplingRatios(vector <uint16_t> &oversamplingRatios);
    ErrorCodes_t getOversamplingRatio(uint16_t &oversamplingRatio);

    ErrorCodes_t getVoltageStimulusLpfs(vector <Measurement_t> &filterOptions, uint16_t &defaultOption);
    ErrorCodes_t getVoltageReferenceLpfs(vector <Measurement_t> &filterOptions, uint16_t &defaultOption);

    ErrorCodes_t hasSelectStimulusChannel(bool &selectStimulusChannelFlag, bool &singleChannelSSCFlag);
    ErrorCodes_t hasDigitalOffsetCompensation(bool &digitalOffsetCompensationFlag, bool &singleChannelDOCFlag, bool &selectableDOCAutostopFlag);
    ErrorCodes_t hasZap(bool &zappableDeviceFlag, bool &singleChannelZapFlag);
    ErrorCodes_t hasChannelOn(bool &channelOnFlag, bool &singleChannelOnFlag);

    ErrorCodes_t hasDigitalOutput();
    ErrorCodes_t hasFrontEndResetDenoiser();

    ErrorCodes_t getLiquidJunctionControl(CompensationControl_t &control);

    ErrorCodes_t getProtocolList(vector <string> &names, vector <string> &images, vector <vector <uint16_t>> &voltages, vector <vector <uint16_t>> &times, vector <vector <uint16_t>> &slopes, vector <vector <uint16_t>> &adimensionals);
    ErrorCodes_t getTriangularProtocolIdx(uint16_t &idx);
    ErrorCodes_t getSealTestProtocolIdx(uint16_t &idx);
    ErrorCodes_t getProtocolVoltage(vector <string> &voltageNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getProtocolTime(vector <string> &timeNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getProtocolSlope(vector <string> &slopeNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getProtocolAdimensional(vector <string> &adimensionalNames, vector <RangedMeasurement_t> &ranges, vector <Measurement_t> &defaultValues);
    ErrorCodes_t getVoltageOffsetControls(RangedMeasurement_t &voltageRange);
    ErrorCodes_t getInsertionPulseControls(RangedMeasurement_t &voltageRange, RangedMeasurement_t &durationRange);
    ErrorCodes_t hasReferencePulseControls(bool &referencePulseImplemented, bool &overrideReferencePulseImplemented);
    ErrorCodes_t getReferencePulseControls(RangedMeasurement_t &voltageRange, RangedMeasurement_t &durationRange);
    ErrorCodes_t getEdhFormat(string &format);
    ErrorCodes_t getRawDataFilterCutoffFrequency(RangedMeasurement_t &range, Measurement_t &defaultValue);
    ErrorCodes_t getLedsNumber(uint16_t &ledsNumber);
    ErrorCodes_t getLedsColors(vector <uint32_t> &ledsColors);
    ErrorCodes_t getFastReferencePulseProtocolWave1Range(RangedMeasurement_t &voltageRange, RangedMeasurement_t &timeRange, uint16_t &nPulse);
    ErrorCodes_t getFastReferencePulseProtocolWave2Range(RangedMeasurement_t &voltageRange, RangedMeasurement_t &timeRange, RangedMeasurement_t &durationRange, uint16_t &nPulse);

    /*! Device specific controls */

    ErrorCodes_t hasNanionTemperatureController();
    virtual ErrorCodes_t getTemperatureControllerRange(int &minTemperature, int &maxTemperature);
    ErrorCodes_t hasWasherControls();
    virtual ErrorCodes_t getWasherSpeedRange(RangedMeasurement_t &range);
    virtual ErrorCodes_t getWasherStatus(WasherStatus_t &status, WasherError_t &error);
    virtual ErrorCodes_t getWasherPresetSpeeds(vector <int8_t> &speedValue);

    ErrorCodes_t hasCFastCompensation();
    ErrorCodes_t getCFastCompensationOptions(vector <string> &options);
    ErrorCodes_t getCFastCapacitanceControl(CompensationControl_t &control);

    virtual ErrorCodes_t updateVoltageOffsetCompensations(vector <Measurement_t> &offsets);

protected:
    typedef enum {
        RxParseLookForHeader,
        RxParseLookForLength,
        RxParseLookForCrc
    } RxParsePhase_t;

    typedef enum {
        ResetIndexChip,
        ResetIndexDigitalOffsetCompensation,
        ResetIndexNum
    } ResetIndex_t;

    typedef enum {
        FtdStatusHeaderFound,
        FtdStatusLookingForHeader,
        FtdStatusBufferFinished
    } FtdBufferAnalaysisStatus_t;

    enum CompensationsTypes {
        CompensationCFast,
        CompensationsNum
    };

    /*************\
     *  Methods  *
    \*************/

    ErrorCodes_t initFtdiChannel(FT_HANDLE * handle, char channel);
    virtual void initializeDevice() = 0;
    virtual bool checkProtocolValidity(string &message) = 0;

    void initializeLsbNoise(bool nullValues = true);
    void initializeCompensations();

    void processCurrentData(uint16_t channelIdx, uint16_t &x);

    void initializeFerdMemory();
    virtual void setFerdParameters();
    double frontEndResetDenoise(uint16_t channelIdx, double x);

    virtual void storeDataFrames(unsigned int framesNum);
    void stackOutgoingMessage(vector <uint8_t> &txDataMessage);

    inline void int322uint16(int32_t from, vector <uint16_t> &to, size_t offset);
    inline void uint322uint16(uint32_t from, vector <uint16_t> &to, size_t offset);

    void computeMinimumPacketNumber();
    void initializeRawDataFilterVariables();
    void computeFilterCoefficients();
    double applyFilter(uint16_t channelIdx, double x);
    void manageVoltageReference();

    /****************\
     *  Parameters  *
    \****************/

    FtdiEepromId_t ftdiEepromId = FtdiEepromId56;

    string upgradeNotes = "NONE";
    string notificationTag = "UNDEFINED";

    char rxChannel;
    char txChannel;

    uint32_t rxSyncWord;
    uint8_t txSyncWord;

    int packetsPerFrame = 16;

    uint16_t voltageChannelsNum = 1;
    uint16_t currentChannelsNum = 1;
    uint16_t totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    uint32_t currentRangesNum;
    uint32_t selectedCurrentRangeIdx = 0;
    vector <RangedMeasurement_t> currentRangesArray;
    vector <uint16_t> defaultCurrentRangesIdx;
    vector <BoolRandomArrayCoder *> currentRangeCoders;

    uint32_t voltageRangesNum;
    vector <RangedMeasurement_t> voltageRangesArray;
    uint16_t defaultVoltageRangeIdx = 0;
    BoolRandomArrayCoder * voltageRangeCoder;

    uint32_t voltageReferenceRangesNum = 0;
    vector <RangedMeasurement_t> voltageReferenceRangesArray;
    uint16_t defaultVoltageReferenceRangeIdx = 0;
    BoolRandomArrayCoder * voltageReferenceRangeCoder;

    int16_t voltageRangeDivider = 1; /*! Divides the voltage data received from the amplifier */

    uint32_t samplingRatesNum;
    vector <Measurement_t> samplingRatesArray;
    uint16_t defaultSamplingRateIdx = 0;
    vector <Measurement_t> realSamplingRatesArray;
    vector <Measurement_t> integrationStepArray;
    BoolRandomArrayCoder * samplingRateCoder;

    bool oversamplingImplemented = false;
    uint32_t oversamplingRatiosNum = 1;
    vector <uint16_t> oversamplingRatiosArray;
    BoolRandomArrayCoder * oversamplingRatioCoder;

    bool selectStimulusChannelFlag = false;
    bool singleChannelSSCFlag = false;
    bool digitalOffsetCompensationFlag = false;
    bool singleChannelDOCFlag = false;
    bool selectableDOCAutostopFlag = false;
    bool zappableDeviceFlag = false;
    bool singleChannelZapFlag = false;
    bool channelOnFlag = false;
    bool singleChannelOnFlag = false;

    bool resetCalibrationFlag = false;

    BoolArrayCoder * selectStimulusChannelCoder;
    vector <bool> selectStimulusChannelStates;

    BoolArrayCoder * deviceResetCoder;
    BoolArrayCoder * calibResetCoder;
    BoolArrayCoder * digitalOffsetCompensationCoder;
    BoolArrayCoder * digitalOffsetCompensationAutostopCoder;
    vector <bool> digitalOffsetCompensationStates;
    BoolArrayCoder * digitalOffsetCompensationResetCoder;

    BoolArrayCoder * zapCoder;
    vector <bool> zapStates;

    BoolArrayCoder * channelOnCoder;
    vector <bool> channelOnStates;

    BoolArrayCoder * VcSel0Coder;
    BoolArrayCoder * VcSel1Coder;
    BoolArrayCoder * digOutCoder;

    vector <RangedMeasurement_t> protocolVoltageRangesArray;
    vector <RangedMeasurement_t> protocolTimeRangesArray;
    vector <RangedMeasurement_t> protocolSlopeRangesArray;

    vector <string> protocolsNames;
    vector <string> protocolsImages;
    vector <vector <uint16_t>> protocolsAvailableVoltages;
    vector <vector <uint16_t>> protocolsAvailableTimes;
    vector <vector <uint16_t>> protocolsAvailableSlopes;
    vector <vector <uint16_t>> protocolsAvailableAdimensionals;
    BoolArrayCoder * protocolsSelectCoder;
    BoolArrayCoder * protocolStartCoder;
    uint16_t defaultProtocol;
    uint16_t selectedProtocol;
    uint16_t triangularProtocolIdx = 0;
    uint16_t sealTestProtocolIdx = 0;

    unsigned int protocolVoltagesNum;
    vector <std::string> protocolVoltageNames;
    vector <RangedMeasurement_t> protocolVoltageRanges;
    vector <DoubleCoder *> protocolVoltageCoders;
    vector <Measurement_t> protocolVoltageDefault;
    vector <Measurement_t> selectedProtocolVoltage;

    unsigned int protocolTimesNum;
    vector <std::string> protocolTimeNames;
    vector <RangedMeasurement_t> protocolTimeRanges;
    vector <DoubleCoder *> protocolTimeCoders;
    vector <Measurement_t> protocolTimeDefault;
    vector <Measurement_t> selectedProtocolTime;

    unsigned int protocolSlopesNum;
    vector <std::string> protocolSlopeNames;
    vector <RangedMeasurement_t> protocolSlopeRanges;
    vector <DoubleCoder *> protocolSlopeCoders;
    vector <Measurement_t> protocolSlopeDefault;
    vector <Measurement_t> selectedProtocolSlope;

    unsigned int protocolAdimensionalsNum;
    vector <std::string> protocolAdimensionalNames;
    vector <RangedMeasurement_t> protocolAdimensionalRanges;
    vector <DoubleCoder *> protocolAdimensionalCoders;
    vector <Measurement_t> protocolAdimensionalDefault;
    vector <Measurement_t> selectedProtocolAdimensional;

    vector <Measurement_t> selectedVoltageOffset;
    Measurement_t minSelectedVoltageOffset = {0, UnitPfxMilli};
    Measurement_t maxSelectedVoltageOffset = {0, UnitPfxMilli};

    bool voltageOffsetControlImplemented = false;
    RangedMeasurement_t voltageOffsetRange;
    vector <DoubleCoder *> voltageOffsetCoders;

    bool insertionPulseImplemented = false;
    RangedMeasurement_t insertionPulseVoltageRange;
    RangedMeasurement_t insertionPulseDurationRange;
    DoubleCoder * insertionPulseVoltageCoder;
    DoubleCoder * insertionPulseDurationCoder;
    BoolCoder * insertionPulseApplyCoder;

    bool referencePulseImplemented = false;
    RangedMeasurement_t referencePulseVoltageRange;
    RangedMeasurement_t referencePulseDurationRange;
    DoubleCoder * referencePulseVoltageCoder;
    DoubleCoder * referencePulseDurationCoder;
    BoolCoder * referencePulseApplyCoder;

    bool overrideReferencePulseImplemented = false;
    BoolCoder * overrideReferencePulseApplyCoder;

    bool fastPulseProtocolImplementation = false;
    uint16_t fastPulseW1num = 0;
    vector <DoubleCoder *> fastPulseW1VoltageCoder;
    RangedMeasurement_t fastPulseW1VoltageRange;
    vector <DoubleCoder *> fastPulseW1TimeCoder;
    RangedMeasurement_t fastPulseW1TimeRange;

    uint16_t fastPulseW2num = 0;
    vector <DoubleCoder *> fastPulseW2VoltageCoder;
    RangedMeasurement_t fastPulseW2VoltageRange;
    vector <DoubleCoder *> fastPulseW2TimeCoder;
    RangedMeasurement_t fastPulseW2TimeRange;
    vector <DoubleCoder *> fastPulseW2DurationCoder;
    RangedMeasurement_t fastPulseW2DurationRange;

    string edhFormat;

    /*! Filter */
    RangedMeasurement_t rawDataFilterCutoffFrequencyRange;
    Measurement_t rawDataFilterCutoffFrequencyDefault;

    /*! Digital output */
    bool digOutImplementedFlag = false;

    /*! Front end denoiser */
    bool ferdImplementedFlag = false;
    unsigned int maxFerdSize = 1;

    bool dacIntFilterAvailable = false;
    vector <Measurement_t> voltageStimulusLpfOptions;
    uint16_t voltageStimulusLpfOptionsNum = 0;
    uint16_t voltageStimulusLpfDefaultOption = 0;
    BoolRandomArrayCoder * dacIntFilterCoder;
    bool dacExtFilterAvailable = false;
    vector <Measurement_t> voltageReferenceLpfOptions;
    uint16_t voltageReferenceLpfOptionsNum = 0;
    uint16_t voltageReferenceLpfDefaultOption = 0;
    BoolRandomArrayCoder * dacExtFilterCoder;

    uint16_t ledsNum = 0;
    vector <BoolArrayCoder *> ledsCoders;
    vector <uint32_t> ledsColorsArray;

    bool dacExtControllableFlag = false; /*! This is true if the voltage applied on the external DAC is directly controllable by the user, not through protocols */
    bool invertedDacExtFlag = false; /*! Negate the DAC value before applying it */
    vector <DoubleOffsetBinaryCoder *> dacExtCoders;
    Measurement_t dacExtDefault;
    Measurement_t voltageReference;
    int16_t voltageReferenceOffset = 0; /*! Value added to returned voltage data to accoutn for the voltage applied on the reference */

    /*! Device specific parameters */

    bool nanionTemperatureControllerFlag = false;
    bool washerControlFlag = false;

    vector <bool> compensationsEnabledArray[CompensationsNum]; /*! Compensations actually enabled on device */

    uint16_t compensationsSettingChannel = 0;

    vector <double> cFastCapacitance;
    vector <bool> cFastCompensationFlag;
    CompensationControl_t cFastCompensationControl;
    vector <string> cFastCompensationOptions;
    vector <DoubleCoder *> cFastControlCoders;
    vector <BoolArrayCoder *> cFastOnCoders;

    BoolArrayCoder * bitDebugCoder = nullptr;
    BoolArrayCoder * byteDebugCoder = nullptr;

    /***************\
     *  Variables  *
    \***************/

    string deviceId;

    FtdiEeprom * ftdiEeprom = nullptr;
    FT_HANDLE * ftdiRxHandle = nullptr;
    FT_HANDLE * ftdiTxHandle = nullptr;

    bool connected = false;
    bool connectionPaused = false;
    bool threadsStarted = false;
    bool stopConnectionFlag = false;

    /*! Read data buffer management */
    mutable mutex readDataMtx;
    int readFrameLength;
    uint8_t nullInfoStruct;
    uint8_t * infoStructPtr = &nullInfoStruct;
    unsigned int infoStructSize = 1;
    unsigned long minReadFrameNumber = FTD_DEFAULT_MIN_READ_FRAME_NUMBER; /*!< Minimum number of frames before they are read from the FTDI driver buffer */
    unsigned long minStoreFrameNumber = FTD_DEFAULT_MIN_STORE_FRAME_NUMBER; /*!< Minimum number of frames before they are stored in the commlib output buffer */
    unsigned int fewFramesSleep = FTD_DEFAULT_FEW_FRAME_SLEEP;
    unsigned char * readDataBuffer = nullptr; /*!< Buffer used in the read to store the data received from the device */
    unsigned int bufferReadOffset = 0; /*!< Device Rx buffer offset position in which data are collected by the outputDataBuffer */
    unsigned int bufferWriteOffset = 0; /*!< Device Rx buffer offset position in which data are written by FTDI device */
    unsigned int bytesReadFromDriver = 0; /*!< Accounts for bytes written from device Rx buffer but not yet written to the output buffer */
    bool bufferDataLossFlag = false; /*!< Set to true by a loss of data, reset by a status read by the user */
    bool exitOnSyncWord = false; /*!< Tells if the last buffer analysis returned with a syncword found */

    /*! Output data buffer management */
    uint16_t ** outputDataBuffer = nullptr; /*!< Buffer used to share received and converted data with the user */
    unsigned int outputBufferReadOffset = 0; /*!< outputDataBuffer offset position in which data are collected by the user */
    unsigned int outputBufferWriteOffset = 0; /*!< outputDataBuffer offset position in which data are converted from readDataBuffer */
    unsigned int outputBufferAvailablePackets = 0; /*!< Number of packets available for the user to read */
    bool outputBufferOverflowFlag = false; /*!< Set to true by an overflow of outputDataBuffer, reset by a status read by the user */
    bool bufferSaturationFlag = false; /*!< Set to true by data saturating the front end range */
    bool bufferIncreaseCurrentRangeFlag = false; /*!< Set to true by data indicating that the front end range might be increased */
    bool bufferDecreaseCurrentRangeFlag = false; /*!< Set to true by data indicating that the front end range might be decreased */
    bool deviceCommunicationErrorFlag = false; /*!< Set to true by failures in communication with the device */

    /*! Write data buffer management */
    uint8_t * txRawBuffer; /*!< Raw outgoing data to the device */
    vector <uint8_t> * txMsgBuffer; /*!< Buffer of arrays of bytes to communicate to the device */
    uint32_t txMsgBufferWriteOffset = 0; /*!< Offset of the part of buffer to be written */
    uint32_t txMsgBufferReadLength = 0; /*!< Length of the part of the buffer to be processed */
    uint16_t txDataBytes;
    unsigned int maxOutputPacketsNum;
    vector <uint8_t> txStatus; /*!< Status of the bytes written */

    double * lsbNoiseArray;
    uint32_t lsbNoiseIdx = 0;

    /*! Front end configuration */
    vector <double> currentResolutions;
    double voltageResolution = 1.0;

    double voltageOffsetCorrected = 0.0; /*!< Value currently corrected in applied voltages by the device (expressed in the unit of the liquid junction control) */
    double voltageOffsetCorrection = 0.0; /*!< Value to be used to correct the measured votlage values (expressed in the unit of current voltage range) */

    Measurement_t voltageOffsetCompensationGain = {1.0, UnitPfxNone};

    uint16_t selectedVoltageRangeIdx = 0;
    uint16_t selectedVoltageReferenceRangeIdx = 0;
    vector <uint16_t> selectedCurrentRangesIdx;
    bool independentCurrentRangesFlag = false;
    RangedMeasurement_t voltageRange;
    RangedMeasurement_t voltageReferenceRange;
    vector <RangedMeasurement_t> currentRanges;

    uint16_t selectedSamplingRateIdx = 0;
    Measurement_t baseSamplingRate = {1.0, UnitPfxKilo};
    Measurement_t samplingRate = {1.0, UnitPfxKilo}; /*!< baseSamplingRate*oversamplingRatio */
    Measurement_t integrationStep = {1.0, UnitPfxMilli};

    uint16_t selectedOversamplingRatioIdx = 0;
    uint16_t oversamplingRatio = 1;

    /*! Filter */
    bool rawDataFilterLowPassFlag = true;
    bool rawDataFilterActiveFlag = false;
    Measurement_t rawDataFilterCutoffFrequency = {30.0, UnitPfxKilo};
    double iirNum[IIR_ORD+1];
    double iirDen[IIR_ORD+1];

    double ** iirX = nullptr;
    double ** iirY = nullptr;

    uint16_t iirOff = 0;

    /*! Front end denoiser */
    bool ferdFlag = false;
    bool ferdInhibition = true;

    unsigned int ferdL = 1; /*!< buffer length */
    double ferdD = 1.0; /*!< inverse of buffer length */
    unsigned int ferdIdx = 0;
    double ferdK; /*!< first order filter coefficient */

    vector <vector <double>> ferdY;
    vector <double> ferdY0;
    vector <double> ferdM;

    CompensationControl_t liquidJunctionControl;
    double liquidJunctionResolution = 1.0;
    double liquidJunctionOffsetBinary = 0.0;

    /********************************************\
     *  Multi-thread synchronization variables  *
    \********************************************/

    thread rxThread;
    thread txThread;

    condition_variable rxMsgBufferNotEmpty;
    condition_variable rxMsgBufferNotFull;

    mutable mutex txMutex;
    condition_variable txMsgBufferNotEmpty;
    condition_variable txMsgBufferNotFull;

    mutable mutex txAckMutex;

    condition_variable txAckCv;

#ifdef DEBUG_PRINT
    FILE * fid;
#endif
};

class MessageDispatcherLegacyEdr3 : public MessageDispatcher {
public:
    MessageDispatcherLegacyEdr3(string deviceId);
    virtual ~MessageDispatcherLegacyEdr3();

protected:
    virtual void storeDataFrames(unsigned int framesNum) override;

    uint16_t rawVoltageZero = 0;
};

#endif // MESSAGEDISPATCHER_H
