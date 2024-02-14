//  Copyright (C) 2021-2024 Filippo Cona
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

#include <vector>
#include <string>
#include <math.h>
#include <limits>

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

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
#define ER4COMMLIB_NAME_MANGLING
#else
#define ER4COMMLIB_NAME_MANGLING extern "C"
#endif

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
#define ER4CL_DATA_ARRAY_SIZE (0x400000)

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
namespace er4CommLib {
#endif

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
    DeviceENPRHC_V01,           /*!< eNPR-HC. */
    DeviceENPRHC_V02,           /*!< eNPR-HC with 200ksps option. */
    DeviceE4nEDR3_V04,          /*!< e4 Orbit mini with old ramp protocols (Legacy version for EDR3). */
    DeviceE4nEDR3_V05,          /*!< e4 Orbit mini (Legacy version for EDR3). */
    DeviceE4eEDR3_V05,          /*!< e4 Elements (Legacy version for EDR3). */
    DeviceE4n_V01,              /*!< e4 Orbit mini. */
    DeviceE4e_V01,              /*!< e4 Elements version. */
    DeviceE16FastPulses_V01,    /*!< e16 Orbit customized for fast pulses. */
    DeviceE16FastPulses_V02,    /*!< e16 Orbit customized for fast pulse trains. */
    DeviceE16FastPulsesEDR3,    /*!< e16 Orbit customized for fast pulses (Legacy version for EDR3). */
    DeviceE16n,                 /*!< e16 Orbit TC. */
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
    DeviceENPR2Channels_V02,    /*!< eNPR prototype with 2 channels with independent current ranges and sinusoidal waveforms. */
    DeviceOrbitMiniSine_V01,    /*!< Orbit mini prototype with additional sinusoidal waveforms. */
    DeviceE16nSine_V01,         /*!< e16 Orbit TC with additional sinusoidal waveforms. */
    DeviceENPRNanopipette_V01,  /*!< eNPR prototype with 2 channels with independent current ranges and PWM control. */
    DeviceE1ULN_V01,            /*!< e1ULN prototype with eNPR PCB. */
    DeviceFakeENPR,             /*!< Fake eNPR. */
    DeviceFakeENPRHC,           /*!< Fake eNPR-HC. */
    DeviceFakeE16n,             /*!< Fake e16 Orbit TC. */
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

/*! \struct QueueStatus_t
 * \brief Struct that contains information on the device status.
 * Returned by getQueueStatus.
 */
typedef struct {
    unsigned int availableDataPackets; /*!< Number of data packets available for read.
                                        *   Each data packets consists of 1 sample per channel.
                                        *   Successful calls to readData reduce this number. */
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

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
/*! Momentarily close namespace because it gets open in er4commlib_global_addendum.h */
}  // namespace er4CommLib

#include "er4commlib_global_addendum.h"

namespace er4CommLib {
#endif

#ifdef ER4COMMLIB_LABVIEW_WRAPPER
#define _NI_int8_DEFINED_
#include "extcode.h"
/*! \typedef LVMeasurement_t
 */

/*! \struct LVMeasurement_t
 * \brief Structure used manage physical quantities that define a value with its unit and unit prefix.
 */
typedef struct LVMeasurement {
    double value; /*!< Numerical value. */
    UnitPfx_t prefix; /*!< Unit prefix in the range [femto, Peta]. */
} LVMeasurement_t;

/*! \typedef LVRangedMeasurement_t
 */

/*! \struct LVRangedMeasurement_t
 * \brief Structure used manage physical ranges that define a range with its unit and unit prefix.
 */
typedef struct LVRangedMeasurement {
    double min; /*!< Minimum value. */
    double max; /*!< Maximum value. */
    double step; /*!< Resolution. */
    UnitPfx_t prefix = UnitPfxNone; /*!< Unit prefix in the range [femto, Peta]. */
} LVRangedMeasurement_t;

/*! \struct LVCompensationControl_t
 * \brief Structure used to return detailed information on a specific compensation implemented by the HW.
 */
typedef struct LVCompensationControl {
    bool implemented = false; /*!< True if the corresponding compensation is implemented by the device. */
    double min = 0.0; /*!< Minimum compensable value. */
    double max = 1.0; /*!< Maximum compensable value globally. */
    double compensable = 1.0; /*!< Maximum compensable value given also the value of the other compensations. */
    double steps = 2; /*!< Number of steps between #min and #max. */
    double step = 1.0; /*!< Resolution. */
    int decimals = 0; /*!< Decimals to represent the compensated value. */
    double value = 0.0; /*!< Compensated value. */
    UnitPfx_t prefix = UnitPfxNone; /*!< Unit prefix in the range [femto, Peta]. */
} LVCompensationControl_t;

//#include "lv_prolog.h"
//typedef struct {
//    int32 cnt;                              /* number of measurements that follow */
//    CharMeasurement_t item[1];              /* cnt measurements */
//} LMeas, *LMeasPtr, **LMeasHandle;

//typedef struct {
//    int32 cnt[2];                           /* size of matrix of measurements that follow */
//    CharMeasurement_t item[1];              /* cnt vector of measurements */
//} LVecMeas, *LVecMeasPtr, **LVecMeasHandle;

//typedef struct {
//    int32 cnt;                              /* number of ranged measurements that follow */
//    CharRangedMeasurement_t	item[1];        /* cnt ranged measurements */
//} LRange, *LRangePtr, **LRangeHandle;

//typedef struct {
//    int32 cnt;                              /* number of compensation controls that follow */
//    CharCompensationControl_t item[1];      /* cnt compensation control */
//} LComp, *LCompPtr, **LCompHandle;

//#define LVecBuf(sp)	(&((sp))->item[0])                          /* pointer to first item of vector */
//#define LVecItem(sp, n)	((&((sp))->item[n]))                    /* pointer to n-th item of vector */
//#define LVecLen(sp)	(((sp))->cnt)                               /* # of items in vector */
//#define LMatS1(sp) (((sp))->cnt[0])                             /* # of rows in matrix */
//#define LMatS2(sp) (((sp))->cnt[1])                             /* # of cols in matrix */
//#define LMatLen(sp)	(LMatS1(sp)*LMatS2(sp))                     /* # of items in matrix */
//#define LMatItem(sp, m, n) ((&((sp))->item[m+n*LMatS1(sp)]))    /* pointer to n-th item of vector */
//#include "lv_epilog.h"
#endif

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

#ifndef ER4COMMLIB_LABVIEW_WRAPPER
} // namespace er4CommLib
#endif

#endif // ER4COMMLIB_GLOBAL_H
