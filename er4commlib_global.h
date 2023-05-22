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

/*! \file er4commlib_global.h
 * \brief Defines global macros and typedefs.
 * \note \a ER4COMMLIB_LIBRARY should be defined only during the library compilation.
 * Defining \a ER4COMMLIB_LIBRARY when the library is included in a project will prevent project building.
 */
#ifndef ER4COMMLIB_GLOBAL_H
#define ER4COMMLIB_GLOBAL_H


#include <stdint.h>
//#include <string.h>
//#include <cmath>
//#include <limits>

/****************************\
 *  Shared library defines  *
\****************************/

#if defined(_WIN32)

#if defined(ER4COMMLIB_STATIC)
#  define ER4COMMLIBSHARED_EXPORT
#else
#if defined(ER4COMMLIB_LIBRARY)
#  define ER4COMMLIBSHARED_EXPORT __declspec(dllexport)
#else
#  define ER4COMMLIBSHARED_EXPORT __declspec(dllimport)
#endif
#endif

#elif defined(__APPLE__)

#if defined(ER4COMMLIB_STATIC)
#  define ER4COMMLIBSHARED_EXPORT
#else
#if defined(ER4COMMLIB_LIBRARY)
#  define ER4COMMLIBSHARED_EXPORT __attribute__((visibility("default")))
#else
#  define ER4COMMLIBSHARED_EXPORT __attribute__((visibility("default")))
#endif
#endif

#endif

/**************************\
 *  Arguments qualifiers  *
\**************************/

#define ER4COMMLIB_NAME_MANGLING extern "C"

/*! \def ER4CL_ARGIN
 * \brief Dummy define to identify input arguments.
 */
#define ER4CL_ARGIN

/*! \def ER4CL_ARGOUT
 * \brief Dummy define to identify output arguments.
 */
#define ER4CL_ARGOUT

/*! \def ER4CL_ARGVOID
 * \brief Dummy define to identify void arguments.
 */
#define ER4CL_ARGVOID void

/*! \def ER4CL_DATA_ARRAY_SIZE
 * \brief Size of data array.
 * When calling method readData data corresponding to at most #ER4CL_DATA_ARRAY_SIZE samples is returned.
 * When calling method readData provide an array of float with at least #ER4CL_DATA_ARRAY_SIZE items.
 */
#define ER4CL_DATA_ARRAY_SIZE (0x40000)


/********************\
 *  Other typedefs  *
\********************/

/*! \enum DeviceTypes_t
 * \brief Enumerates the device types that can be handled by er4CommLib.
 */
typedef enum {
    DeviceE1bEL03cEDR3,         /*!< e1b ELo3c chip (Legacy version for EDR3). */
    DeviceE1PlusEL03cEDR3,      /*!< e1+ EL03f chip (Legacy version for EDR3). */
    DeviceE1LightEL03cEDR3,     /*!< e1Light EL03f chip (Legacy version for EDR3). */
    DeviceE1HcEL03cEDR3,        /*!< e1HC EL03f chip (Legacy version for EDR3) */
    DeviceE1PlusEL03fEDR3,      /*!< e1+ EL03f chip (Legacy version for EDR3). */
    DeviceE1LightEL03fEDR3,     /*!< e1Light EL03f chip (Legacy version for EDR3). */
    DeviceE1HcEL03fEDR3,        /*!< e1HC EL03f chip (Legacy version for EDR3) */
    DeviceE16eEDR3,             /*!< e16e (Legacy version for EDR3). */
    DeviceE16ETHEDR3,           /*!< e16ETH (LegacyVersion for EDR3). */
    DeviceE16HC_V01,            /*!< e16HC (no voltage amplifier). */
    DeviceE16HC_V02,            /*!< e16HC. */
    DeviceENPREDR3_V03,         /*!< eNPR (Legacy version for EDR3). */
    DeviceENPREDR3_V04,         /*!< eNPR (Legacy version for EDR3). */
    DeviceENPR,                 /*!< eNPR. */
    DeviceENPRHC,               /*!< eNPR-HC. */
    DeviceE4nEDR3_V04,          /*!< e4 Orbit mini with old ramp protocols (Legacy version for EDR3). */
    DeviceE4eEDR3,              /*!< e4 Elements (Legacy version for EDR3). */
    DeviceE4n_V01,              /*!< e4 Orbit mini. */
    DeviceE4e_V01,              /*!< e4 Elements version. */
    DeviceE16FastPulses_V01,    /*!< e16 Orbit customized for fast pulses. */
    DeviceE16FastPulses_V02,    /*!< e16 Orbit customized for fast pulse trains. */
    DeviceE16FastPulsesEDR3,    /*!< e16 Orbit customized for fast pulses (Legacy version for EDR3). */
    DeviceE16n,                 /*!< e16 2020 release. */
    DeviceE2HC_V01,             /*!< e2HC. */
    DeviceDlp,                  /*!< debug dlp. */
    TestboardEL06b,             /*!< testboard chip EL06b */
    TestboardEL06c,             /*!< testboard chip EL06c */
    TestboardEL06dEL06e,        /*!< testboard chip EL06d and EL06e */
    DeviceE2HCExtAdc,           /*!< e2HC prototype (external ADC). */
    DeviceE2HCIntAdc,           /*!< e2HC prototype (internal ADC). */
    DeviceENPRFairyLight_V01,   /*!< eNPR prototype for Fairy Light project with DAC ext control and only ULN mode. */
    DeviceENPRFairyLight_V02,   /*!< eNPR prototype for Fairy Light project without DAC ext control and both ULN and LN modes. */
    DeviceENPR2Channels_V01,    /*!< eNPR prototype with 2 channels and sinusoidal waveforms. */
    DeviceOrbitMiniSine_V01,    /*!< Orbit mini prototype with additional sinusoidal waveforms. */
    DeviceFakeE16n,             /*!< Fake e16 2020 release. */
    DeviceFakeE16FastPulses,    /*!< Fake e16 Orbit customized for fast pulses. */
    DeviceUnknown,              /*!< Invalid item used only for initiliazation purposes. */
    DevicesNum
} DeviceTypes_t;

/*! \enum UnitPfx_t
 * \brief Enumerates the unit prefixes used.
 */
typedef enum {
    UnitPfxFemto    = 0,    /*!< 10^-15 */
    UnitPfxPico     = 1,    /*!< 10^-12 */
    UnitPfxNano     = 2,    /*!< 10^-9 */
    UnitPfxMicro    = 3,    /*!< 10^-6 */
    UnitPfxMilli    = 4,    /*!< 10^-3 */
    UnitPfxNone     = 5,    /*!< 10^0 = 1 */
    UnitPfxKilo     = 6,    /*!< 10^3 */
    UnitPfxMega     = 7,    /*!< 10^6 */
    UnitPfxGiga     = 8,    /*!< 10^9 */
    UnitPfxTera     = 9,    /*!< 10^12 */
    UnitPfxPeta     = 10,   /*!< 10^15 */
    UnitPfxNum              /*!< Invalid item used only for loop purposes. */
} UnitPfx_t;


/*! \brief Increments a prefix by n.
 *
 * \param value [out] incremented prefix.
 * \param n [in] increment.
 * \return true if the prefix was successfully incremented; false if the prefix surpasses the maximum possible value.
 */
static inline bool incrementUnit(ER4CL_ARGOUT UnitPfx_t &value, ER4CL_ARGIN int n);

/*! \brief Decrements a prefix by n.
 *
 * \param value [out] decremented prefix.
 * \param n [in] decrement.
 * \return true if the prefix was successfully decremented; false if the prefix surpasses the minimum possible value.
 */
static inline bool decrementUnit(ER4CL_ARGOUT UnitPfx_t &value, ER4CL_ARGIN int n);

static inline bool incrementUnit(ER4CL_ARGOUT UnitPfx_t &value, ER4CL_ARGIN int n) {
    if (value < UnitPfxNum-n) {
        int intValue = static_cast <int> (value) +n;
        value = static_cast <UnitPfx_t> (intValue);
        return true;

    } else {
        value = UnitPfxNum;
        decrementUnit(value, 1);
        return false;
    }
}

static inline bool decrementUnit(ER4CL_ARGOUT UnitPfx_t &value, ER4CL_ARGIN int n) {
    if (value > n-1) {
        int intValue = static_cast <int> (value) -n;
        value = static_cast <UnitPfx_t> (intValue);
        return true;

    } else {
        value = static_cast <UnitPfx_t> (0);
        return false;
    }
}

/*! \brief Array with the strings corresponding to the unit prefixes of type UnitPfx_t. */
static const char unitPrefixes[UnitPfxNum] = {
    'f',
    'p',
    'n',
    'u',
    'm',
    ' ',
    'k',
    'M',
    'G',
    'T',
    'P'
};

/*! \brief Convenience array with the precomputed powers of 1000 used to make unit conversions. */
static const double powersOf1000[UnitPfxNum] = {
    1.0,
    1.0e3,
    1.0e6,
    1.0e9,
    1.0e12,
    1.0e15,
    1.0e18,
    1.0e21,
    1.0e24,
    1.0e27,
    1.0e30
};

/*! \struct MeasurementReduced_t
 * \brief Structure used to manage physical quantities that define a value with its unit prefix.
 */
typedef struct MeasurementReduced {
    double value; /*!< Numerical value. */
    UnitPfx_t prefix; /*!< Unit prefix in the range [femto, Peta]. */
} MeasurementReduced_t;

/*! \struct RangedMeasurementReduced_t
 * \brief Structure used to manage physical ranges that define a range with its unit prefix.
 */
typedef struct RangedMeasurementReduced {
    double min; /*!< Minimum value.*/
    double max; /*!< Maximun value.*/
    double step; /*!< Resolution. */
    UnitPfx_t prefix = UnitPfxNone;
} RangedMeasurementReduced_t;

/*! \struct CompensationControl_t
 * \brief Structure used to return detailed information on a specific compensation implemented by the HW.
 */
typedef struct CompensationControl {
    bool implemented = false; /*!< True if the corresponding compensation is implemented by the device. */
    double min = 0.0; /*!< Minimum compensable value. */
    double max = 1.0; /*!< Maximum compensable value globally. */
    double compensable = 1.0; /*!< Maximum compensable value given also the value of the other compensations. */
    double steps = 2; /*!< Number of steps between #min and #max. */
    double step = 1.0; /*!< Resolution. */
    int decimals = 0; /*!< Decimals to represent the compensated value. */
    double value = 0.0; /*!< Compensated value. */
    UnitPfx_t prefix = UnitPfxNone; /*!< Unit prefix in the range [femto, Peta]. */
//    char * unit = ""; /*!< Unit. \note Can be any string, the library is not aware of real units meaning. */
//    char * name = ""; /*!< Name of the compensation. */

    /*! \brief Returns the string corresponding to the prefix.
     *
     * \return String corresponding to the prefix.
     */
//    char getPrefix(ER4CL_ARGVOID) {
//        return unitPrefixes[prefix];
//    }

} CompensationControl_t;

/******************************\
 *  Device specific typedefs  *
\******************************/

/*! \enum WasherStatus_t
 * \brief Enumerates the e16n washers statuses.
 */
typedef enum {
    WasherIdle,
    WasherUnsaved,
    WasherSaving
} WasherStatus_t;

/*! \enum WasherError_t
 * \brief Enumerates the e16n washers errors.
 */
typedef enum {
    WasherOk,
    WasherTimeout,
    WasherPower,
    WasherCommunication,
    WasherExecution,
    WasherOverload,
    WasherChecksumError,
    WasherIllFormedMessage
} WasherError_t;


#endif // ER4COMMLIB_GLOBAL_H
