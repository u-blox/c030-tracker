/* Copyright (c) 2017 ublox Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This is an Mbed port of the code from the https://github.com/u-blox/tracker
 * which ran on a Particle Electron platform.  This code is targetted
 * at the u-blox C030 board, which uses an STM32F4 processor, a member 
 * of the same processor family as that of the Particle board (so it
 * supports the same power saving behaviours and 4 kBytes of back-up SRAM).
 * Since this is a port from Particle it still adopts the setup()/loop()
 * pattern (see towards the end of this file).
 *
 * In order to use power most efficiently, sleep is used in
 * combination with timed operation in the following way:
 *
 * - If, after establishing network time, the time is found to be
 *   less than START_TIME_UNIX_UTC then, the device returns to
 *   deep sleep (modem off, processor clocks and RAM off,
 *   ~0.1 mA consumed) until START_TIME_UNIX_UTC is reached.
 * - If the time is greater than or equal to START_TIME_UNIX_UTC then
 *   the device checks if the working day has begun, i.e. is the time
 *   greater than START_OF_WORKING_DAY_SECONDS and less than
 *   START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS.
 * - If the time is within the work day then one of two things happens:
 *   - If the time is less than START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC
 *     then the device wakes up only for long enough to get a GNSS reading, 
 *     "slow operation" (timing out at SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS)
 *     and repeats this SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY times,
 *     evenly spread throughout the working day.  Between wake-ups the
 *     device returns to deep sleep with the modem off.
 *   - If the time is greater than or equal to
 *     START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC then the device remains
 *     awake for all of the working day, putting the processor to sleep and
 *     keeping the modem registered with the network if wake-ups are frequent
 *     enough, so that we can collect and report position for the full working day.
 * - If the time is not within the working day then the device returns to
 *   deep sleep, with modem off, until the start of the next working day.
 *
 * The default timings are set so that the device wakes-up in slow operation and
 * then goes into full working day operation a few days later. This was for a show
 * where the trackers were installed a few days before the start of the show and
 * we wanted to monitor that they were OK but not consume a lot of battery power.
 * By judicious choice of:
 *
 *  - START_TIME_UNIX_UTC
 *  - START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC
 *  - START_OF_WORKING_DAY_SECONDS
 *  - LENGTH_OF_WORKING_DAY_SECONDS
 *  - SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY
 *
 * ... you should be able to achieve the tracker behaviour you want.  To always
 * get slow operation, set START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC to a
 * time far in the future.  To always get full monitoring throughout the working
 * day, set START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC to START_TIME_UNIX_UTC.
 * To set the working day to be 24 hours, set START_OF_WORKING_DAY_SECONDS to 0 and
 * LENGTH_OF_WORKING_DAY_SECONDS to 3600.
 *
 * MESSAGES
 *
 * The chosen message formats are highly compressed to
 * save data and are of the following form:
 *
 * "gnss:353816058851462;51.283645;-0.776569;1465283731;1;5.15"
 *
 * ...where the first item is the 15-digit IMEI of the device, the
 * second one the latitude in degrees as a float, the third one the
 * longitude in degrees as a float, then there's the timestamp
 * in Unix time (UTC) and finally the last two, inserted at the end
 * to preserve backwards-compatibility with previous implementations of this
 * messaging system; these are a 1 if the reading was triggered due to the
 * accelerometer indicating motion (0 otherwise) and then the horizontal
 * dilution of position (i.e. accuracy) as a float.  All fields up to and
 * including the timestamp must be present; if either of the fields after the
 * timestamp need to be included then both must be included. This message is
 * queued at every wakeup, if a fix is achieved.
 * 
 * "telemetry:353816058851462;80.65;-70;1465283731;1"
 *
 * ...where the first item is the 15-digit IMEI, the second one
 * the battery left as a percentage, the third the signal strength
 * in dBm and then the timestamp in Unix time (UTC).  After the timestamp,
 * for backwards compatibility with earlier versions of this protocol,
 * is the SW version. All fields except the SW version must be present.
 * This is sent periodically when the device wakes up from deep sleep and
 * every TELEMETRY_PERIOD_SECONDS seconds thereafter.
 *
 * NOTE: in addition to the above there is a "stats" message with
 * a similar format. See queueStatsReport() for more details.
 *
 * NOTE: if you want to use a different message format, maybe
 * using a more standard (though less efficient) JSON approach,
 * simply modify queueTelemeteryReport(), queueGnssReport()
 * (and, if required, queueStatsReport()) to match.
 */

#include "mbed.h"
#include "stm32f4xx_hal_pwr.h" // To enable the WKUP pin
#include "accelerometer_adxl345.h"
#include "gnss.h"
#include "low_power.h"
#include "battery_gauge_bq27441.h"
#include "UbloxAtCellularInterface.h"

/****************************************************************
 * CONDITIONAL COMPILATION OPTIONS
 ***************************************************************/

/// Define this to force a development build, which sends
// stats and initiates "working day" operation straight away.
//#define DEV_BUILD

/// Define this to do GNSS readings irrespective of the state of the
// accelerometer.
//#define DISABLE_ACCELEROMETER

/// Define this if using the USB port for debugging.
#define USB_DEBUG

/// Define this if a 2D GNSS fix is sufficient.
#define GNSS_FIX_2D

/// Define this to skip doing anything with the cellular connection.
//#define DISABLE_CELLULAR_CONNECTION

/// Define this to send stats at each GNSS attempt
#define ENABLE_SEND_STATS_AT_GNSS_ATTEMPT

// The credentials of the SIM in the board.  If PIN checking is enabled
// for your SIM card you must set this to the required PIN.
#define PIN "0000"

// Network credentials.  You should set this according to your
// network/SIM card.  For C030 non-N2xx boards, leave the parameters as NULL
// otherwise, if you do not know the APN for your network, you may
// either try the fairly common "internet" for the APN (and leave the
// username and password NULL), or you may leave all three as NULL and then
// a lookup will be attempted for a small number of known networks
// (see APN_db.h in mbed-os/features/netsocket/cellular/utils).
#define APN         NULL
#define USERNAME    NULL
#define PASSWORD    NULL

// The address of the server we are sending to.
#define SERVER_ADDRESS "ciot.it-sgn.u-blox.com"

// The port of the server we are sending to.
#define SERVER_PORT 5065

/****************************************************************
 * CONFIGURATION MACROS
 ***************************************************************/

/// The start time for the device (in Unix, UTC).
// If the device wakes up before this time it will return
// to deep sleep.  After this time it will work in slow operation,
// waking up at SLOW_MODE_INTERVAL_SECONDS after the start of the
// working day for up to SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS
// each time.
// Use http://www.onlineconversion.com/unix_time.htm to work this out.
#ifdef DEV_BUILD
# define START_TIME_UNIX_UTC 1469340000 // 24th July 2016 @ 06:00 UTC
#else
# define START_TIME_UNIX_UTC 1473933600 // 15th September 2016 @ 12:00 UTC
#endif

/// The start time for full working day operation (in Unix, UTC).
// After this time the device will be awake for the whole working day and send reports
// as necessary.
// This time must be later than or equal to START_TIME_UNIX_UTC.
// Use http://www.onlineconversion.com/unix_time.htm to work this out.
#ifdef DEV_BUILD
# define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1469340000 // 24th July 2016 @ 06:00 UTC
#else
# define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1473933600 // 15th September 2016 @ 12:00 UTC
#endif

/// The version string of this software (an incrementing integer).
#define SW_VERSION 0

/// The minimum time for which we go to deep sleep.  If the sleep time
// is less than this we will use clock-stop sleep instead
#define MINIMUM_DEEP_SLEEP_PERIOD_SECONDS 600

/// If the system cannot establish time by talking to the 
// Particle server, wait this long and try again.
#define TIME_SYNC_RETRY_PERIOD_SECONDS 30

/// The number of times to retry before giving up if an attempt
// to connect fails.
#define CONNECT_RETRIES 10

/// The time between wake-ups as a result of motion being detected.
#define MIN_MOTION_PERIOD_SECONDS 30
#define MAX_MOTION_PERIOD_SECONDS (60 * 5)

/// The amount of time to wait for a publish to complete.
#define WAIT_FOR_PUBLISH_SECONDS 10

/// The offset at the start of a UBX protocol message.
#define GNSS_UBX_PROTOCOL_HEADER_SIZE 6

/// The time we stay awake waiting for GNSS to get a fix every
// MOTION_PERIOD_SECONDS.
// NOTE: the GNSS device may stay awake for longer, this is only
// the period for which the processing system running this
// code stays awake waiting.
#define GNSS_FIX_TIME_SECONDS 20

/// The interval at which we check for a fix, within GNSS_FIX_TIME_SECONDS
#define GNSS_CHECK_INTERVAL_SECONDS 5

/// The largest wake-up period once we're past START_TIME_UNIX_UTC.
#define MAX_WAKEUP_PERIOD_SECONDS (3600 * 2)

/// The periodicity of telemetry reports.
#define TELEMETRY_PERIOD_SECONDS MAX_WAKEUP_PERIOD_SECONDS

/// The periodicity of stats reports.
#define STATS_PERIOD_SECONDS TELEMETRY_PERIOD_SECONDS

/// The report period in seconds.  At this interval the
// queued-up records are sent (though they may be sent
// earlier if QUEUE_SEND_LEN is reached).
#define REPORT_PERIOD_SECONDS (60 * 10)

/// The queue length at which to begin sending records.
#ifdef ENABLE_SEND_STATS_AT_GNSS_ATTEMPT
# define QUEUE_SEND_LEN 8
#else 
# define QUEUE_SEND_LEN 4
#endif

/// The size of a record's contents.
#define LEN_RECORD_CONTENTS 120

/// The size of a record type descriptor when it is a string.
#define LEN_RECORD_TYPE 10

/// The minimum possible value of Time (in Unix, UTC), used
// to check the sanity of our RTC time.
#define MIN_TIME_UNIX_UTC 1451606400 // 1 Jan 2016 @ midnight

/// The maximum number of consecutive connection failures
// before we take further action.
#define MAX_NUM_CONSECUTIVE_CONNECT_FAILURES 5

/// The number of times to wake-up during the working day when in slow operation.
#define SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY 1

/// The maximum amount of time to wait for a GNSS fix to be established while we are in
// slow operation.  After this time, or as soon as a GNSS fix has been established
// and transmitted, we can go to deep sleep.
#define SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS (60 * 10)

/// Start of day in seconds after midnight UTC.
#define START_OF_WORKING_DAY_SECONDS ((3600 * 0) + 1) // 00:01 UTC, the +1
                                                      // to avoid a compiler warning
                                                      // about comparing with 0

/// Duration of a working day in seconds.
#define LENGTH_OF_WORKING_DAY_SECONDS (3600 * 24) // 24 hours

/// Modem wake-up delay.
#define MODEM_POWER_ON_DELAY_MILLISECONDS 1000

/// The accelerometer activity threshold (in units of 62.5 mg).
#define ACCELEROMETER_ACTIVITY_THRESHOLD 3

/// The maximum time to wait for a GNSS fix
#define GNSS_MAX_ON_TIME_SECONDS (60 * 10)

/// How long to wait for an Ack message back from the GNSS module.
#define GNSS_WAIT_FOR_ACK_MILLISECONDS 3000

/// How long to wait for responses from the GNSS module.
#define GNSS_WAIT_FOR_RESPONSE_MILLISECONDS 5000

/// The minimum number of satellites for which we want to have ephemeris data.
#define GNSS_MIN_NUM_EPHEMERIS_DATA 5

/// Something to use as an invalid angle.
#define GNSS_INVALID_ANGLE 999999999

/// Something to use as an invalid HDOP.
#define GNSS_INVALID_HDOP 999999999

/// Amount of time considered as being in motion after an initial accelerometer motion
#define IN_MOTION_TIME_SECONDS (60 * 10)

/***************************************************************
 * OTHER MACROS
 ***************************************************************/

/// A magic string to indicate that retained RAM has been initialised.
#define RETAINED_INITIALISED "RetInit"

/// The length of the IMEI, used as the device ID.
#define IMEI_LENGTH 15

/// Work out the number of seconds between each wake-up in slow mode.
#define SLOW_MODE_INTERVAL_SECONDS (LENGTH_OF_WORKING_DAY_SECONDS / (SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY + 1))

/****************************************************************
 * TYPES
 ***************************************************************/

/// The types of fatal error that can occur.
typedef enum {
    FATAL_TYPE_NULL,
    FATAL_RECORDS_OVERRUN_1,
    FATAL_RECORDS_OVERRUN_2,
    FATAL_RECORDS_OVERRUN_3,
    MAX_NUM_FATAL_TYPES
} FatalType_t;

/// Struct to hold an accelerometer reading.
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelerometerReading_t;

/// The possible record types.
typedef enum {
    RECORD_TYPE_NULL,
    RECORD_TYPE_TELEMETRY,
    RECORD_TYPE_GNSS,
    RECORD_TYPE_STATS,
    MAX_NUM_RECORD_TYPES
} RecordType_t;

/// A single record.
typedef struct {
    bool isUsed;
    RecordType_t type;
    char contents[LEN_RECORD_CONTENTS];
} Record_t;

/// For debugging: LED flash types.
typedef enum {
    DEBUG_IND_OFF,
    DEBUG_IND_TOGGLE,
    DEBUG_IND_GNSS_FIX,
    DEBUG_IND_ACTIVITY,
    DEBUG_IND_RETAINED_RESET,
    DEBUG_IND_REALLY_RETAINED_RESET,
    DEBUG_IND_BOOT_COMPLETE,
    MAX_NUM_DEBUG_INDS
} DebugInd_t;

/// Enum used to do some minimal logging over
// time that can be stored in our tiny retained RAM.
// This is used for bit-masking into a 32-bit wide
// value.
// NOTE: if you add an item to here, don't forget to
// add it to logFlagsAsString() also.
typedef enum {
    LOG_FLAG_TIME_ESTABLISHED_VIA_NETWORK                 = 0x00000001,
    LOG_FLAG_TIME_ESTABLISHED_VIA_GNSS                    = 0x00000002,
    LOG_FLAG_TIME_ESTABLISHED_VIA_RTC                     = 0x00000004,
    LOG_FLAG_TIME_NOT_ESTABLISHED                         = 0x00000008,
    LOG_FLAG_MOTION_DETECTED                              = 0x00000010,
    LOG_FLAG_START_TIME_PASSED                            = 0x00000020,
    LOG_FLAG_START_TIME_FULL_WORKING_DAY_OPERATION_PASSED = 0x00000040,
    LOG_FLAG_IN_WORKING_DAY                               = 0x00000080,
    LOG_FLAG_TELEMETRY_RECORD_QUEUED                      = 0x00000100,
    LOG_FLAG_STATS_RECORD_QUEUED                          = 0x00000200,
    LOG_FLAG_GNSS_RECORD_QUEUED                           = 0x00004000,
    LOG_FLAG_SEND_ATTEMPTED                               = 0x00008000,
    LOG_FLAG_PUBLISH_SUCCEEDED                            = 0x00010000,
    LOG_FLAG_CONNECT_FAILED                               = 0x00020000,
    LOG_FLAG_PUBLISH_FAILED                               = 0x00040000,
    LOG_FLAG_MODEM_CONNECTED                              = 0x00080000,
    LOG_FLAG_GNSS_ON_NOT_OFF                              = 0x00100000,
    LOG_FLAG_GNSS_FIX_ACHIEVED                            = 0x00200000,
    LOG_FLAG_GNSS_FIX_FAILED                              = 0x00400000,
    LOG_FLAG_WAKE_ON_INTERRUPT                            = 0x00800000,
    LOG_FLAG_DEEP_SLEEP_NOT_CLOCK_STOP                    = 0x01000000,
    LOG_FLAG_WOKEN_UP_EARLY                               = 0x02000000,
    LOG_FLAG_STARTUP_WARM_NOT_COLD                        = 0x04000000,
    LOG_FLAG_FREE_1                                       = 0x08000000,
    LOG_FLAG_FREE_2                                       = 0x10000000,
    LOG_FLAG_FREE_3                                       = 0x20000000,
    LOG_FLAG_FREE_4                                       = 0x40000000,
    LOG_FLAG_STARTUP_NOT_LOOP_ENTRY                       = 0x80000000
} LogFlag_t;

/// The stuff that is stored in retained RAM.
// NOTE: the variables that follow are retained so that
// we can go to deep sleep and still keep a record of
// what happened from initial power-on.  They are kept
// in one structure so that they can all be reset with
// a single memset().
// NOTE: if you add anything to here then you should modify
// the debug function debugPrintRetained() also.
// NOTE: if you add or remove anything in this structure
// you should increment SW_VERSION to be sure that retained
// memory is reset when a device that has just loaded with
// the new software boots, otherwise you risk reading garbage.
typedef struct {
    // Something to use as a key so that we know whether 
    // retained memory has been initialised or not
    char key[sizeof(RETAINED_INITIALISED)];
    // The SW version that this retained memory was written
    // from
    uint32_t swVersion;
    // Set this to true to indicate that we don't need to
    // re-do the initialisation
    bool warmStart;
    // Shadow the state of GNSS here so that we can replicate
    // it on return from deep sleep
    bool gnssOn;
    // Parameters that allow management of sleep
    time_t sleepStartSeconds;
    time_t minSleepPeriodSeconds;
    time_t sleepForSeconds;
    time_t modemStaysAwake;
    // The IMEI of the module
    char imei[IMEI_LENGTH];
    // When we last tried to get a fix
    time_t lastGnssSeconds;
    // Time Of the last telemetry message
    time_t lastTelemetrySeconds;
    // Time of the last stats report
    time_t lastStatsSeconds;
    // Time when last sent queued reports
    time_t lastReportSeconds;
    // The time that motion was last detected
    time_t lastMotionSeconds;
    // The time of the last cold start
    time_t lastColdStartSeconds;
    // The last time the accelerometer got a nudge
    time_t lastAccelerometerNudge;
    // Whether we've asked for a GNSS fix or not
    bool gnssFixRequested;
    // The records accumulated
    Record_t records[22];
    // The next free record
    uint32_t nextFreeRecord;
    // The next record to send
    uint32_t nextPubRecord;
    // The number of records queued
    uint32_t numRecordsQueued;
    // The time we went down to low power state
    time_t powerSaveTime;
    // Count the number of times around the loop, for info
    uint32_t numLoops;
    // Count the number of loops on which motion was detected, for info
    uint32_t numLoopsMotionDetected;
    // Count the number of loops where position  was needed, for info
    uint32_t numLoopsLocationNeeded;
    // Count the number of loops for which a GNSS fix was attempted, for info
    uint32_t numLoopsGnssOn;
    // Count the number of loops for which a GNSS fix was achieved, for info
    uint32_t numLoopsGnssFix;
    // Count the number of loops for which we reported a valid location
    // (which is different to numLoopsGnssFix as we may not need to get a fix
    // if we haven't moved)
    uint32_t numLoopsLocationValid;
    // Count the number of seconds we've been in power saving state for
    uint32_t totalPowerSaveSeconds;
    // Count the number of seconds GNSS has been on for
    time_t gnssPowerOnTime;
    uint32_t gnssSeconds;
    uint32_t totalGnssSeconds;
    // Count the number of publish attempts
    uint32_t numPublishAttempts;
    // Count the number of publishes that failed
    uint32_t numPublishFailed;
    // Count the number of connect attempts
    uint32_t numConnectAttempts;
    // Count the number of connect failures
    uint32_t numConnectFailed;
    // Count the number of entries into Setup()
    uint32_t numStarts;
    // Hold the last accelerometer reading
    AccelerometerReading_t accelerometerReading;
} Retained_t;

/// A log flag entry
typedef struct {
    time_t timestamp;
    uint32_t flags;
} LogFlagsEntry_t;

/// This stuff is also stored in retained RAM, but
// needs to be reset separately from the above
typedef struct {
    // Something to use as a key so that we know whether 
    // retained memory has been initialised or not
    char key[sizeof(RETAINED_INITIALISED)];
    // The SW version that this retained memory was written
    // from
    uint32_t swVersion;
    // Storage for fatals
    uint32_t numFatals;
    FatalType_t fatalList[20];
    // Storage for "log flag" bytes
    uint8_t numLogFlagsEntries;
    LogFlagsEntry_t logFlagsEntries[20];
} ReallyRetained_t;

/****************************************************************
 * GLOBAL VARIABLES
 ***************************************************************/

/// Record type strings.
// NOTE: must match the RecordType_t enum above and must be no less
// than LEN_RECORD_TYPE characters long.
static const char *recordTypeString[] = {"null:", "telemetry:", "gnss:", "stats:"};

/// Whether we have an accelerometer or not
static bool accelerometerConnected = false;

/// The interrupt input pin from the accelerometer, has to be
// this pin as this is the only pin that can wake the processor
// from Standby mode.
static InterruptIn accelerometerInterrupt(PA_0);

/// The stats period, stored in RAM so that it can
// be overridden.
static time_t statsPeriodSeconds = STATS_PERIOD_SECONDS;

/// Track, for info, the number of satellites that
// could be used for navigation, if there were any.
static uint32_t gnssNumSatellitesUsable = 0;

/// Track, for info, the peak C/N of the satellites
// used for navigation the last time there were any.
static uint32_t gnssPeakCNUsed = 0;

/// Track, for info, the average C/N of the satellites
// used for navigation the last time there were any.
static uint32_t gnssAverageCNUsed = 0;

/// All the normal retained variables.
BACKUP_SRAM
static Retained_t r;

/// A set of retained variables which are reset separately
// from the normal set.
BACKUP_SRAM
static ReallyRetained_t rr;

/// A buffer used when printing out the time.
static char timeBuf[32];

/// A general purpose buffer, used for sending
// and receiving UBX commands to/from the GNSS module.
static char msgBuffer[1024];

/// A buffer used when sending messages to the server.
static char sendBuffer[LEN_RECORD_TYPE + LEN_RECORD_CONTENTS];

/// A UDP socket.
static UDPSocket udpSock;

/// The address of the destination UDP server.
static SocketAddress udpServer;

/// For hex printing.
static const char hexTable[] =
{ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

/// For date conversion.
static const uint8_t daysInMonth[] = 
{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const uint8_t daysInMonthLeapYear[] = 
{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/// Instantiate lower power helper
static LowPower lowPower;

/// Instantiate the accelerometer.
static AccelerometerAdxl345 accelerometer;

/// Instantiate GNSS.
static GnssSerial gnss;

/// Instantiate the I2C used for devices on board C030.
static I2C i2COnBoard(I2C_SDA_B, I2C_SCL_B);

/// Instantiate the I2C used for devices on the Arduino connector.
static I2C i2C(I2C_SDA, I2C_SCL);

/// Instantiate a battery gauge.
static BatteryGaugeBq27441 batteryGauge;

/// Instantiate cellular.
static UbloxATCellularInterface cellular;

/// Debug LED (blue).
static DigitalOut debugLed(LED3, 1);

/// To pass the time.
static Timer upTimer;

/****************************************************************
 * DEBUG
 ***************************************************************/

/// Macros to invoke asserts/fatals.
// NOTE: the FATAL macro also resets retained RAM (but not "really retained"
// RAM), as it could be something in there that has caused the assert.
#define FATAL(fatalType) {if (rr.numFatals < (sizeof (rr.fatalList) / sizeof (rr.fatalList[0]))) {rr.fatalList[rr.numFatals] = fatalType;} rr.numFatals++; resetRetained(); NVIC_SystemReset();}
#define ASSERT(condition, fatalType) {if (!(condition)) {FATAL(fatalType)}}

#ifdef USB_DEBUG
# define LOG_MSG(...) printf(__VA_ARGS__)
#else
# define LOG_MSG(...)
#endif

/****************************************************************
 * FUNCTION PROTOTYPES
 ***************************************************************/

static uint32_t littleEndianUint32(char *pByte);
static bool handleInterrupt();
static void debugInd(DebugInd_t debugInd);
static void printHex(const char *pBytes, uint32_t lenBytes);
static void debugPrintRetained();
static void addLogFlagsEntry();
static void setLogFlag(uint32_t flag);
static void clearLogFlag(uint32_t flag);
static void logFlagsAsString(uint32_t flags);
static uint32_t readGnssMsg (char *pBuffer, uint32_t bufferLen, int32_t waitMilliseconds);
static bool waitAckGnssMsg(uint8_t msgClass, uint8_t msgId);
static bool configureGnss();
static bool gotGnssFix(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop);
static bool gnssGetTime(time_t *pUnixTimeUtc);
static bool gnssCanPowerSave();
static void resetRetained();
static void resetReallyRetained();
static time_t gnssOn();
static bool gnssOff();
static bool gnssIsOn();
static bool gnssUpdate(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop);
static bool cellularGetTime(time_t *pTime);
static bool connect();
static bool send(RecordType_t type, const char *pContents);
static char *timeString(time_t time);
static bool isLeapYear(uint32_t year);
static bool establishTime();
static time_t getSleepTime(time_t lastTime, time_t period);
static time_t setTimings(uint32_t secondsSinceMidnight, bool atLeastOneGnssReportSent, bool fixAchieved, time_t *pMinSleepPeriodSeconds);
static uint32_t secondsInDayToWorkingDayStart(uint32_t secondsToday);
static time_t truncateToDay(time_t unixTime);
static time_t sleepLimitsCheck(time_t sleepForSeconds);
static char *getRecord(RecordType_t type);
static void freeRecord(Record_t *pRecord);
static uint32_t incModRecords (uint32_t x);
static void queueTelemetryReport();
static void queueGnssReport(float latitude, float longitude, bool motion, float hdop);
static void queueStatsReport();
static bool sendQueuedReports(bool *pAtLeastOneGnssReportSent);

/****************************************************************
 * STATIC FUNCTIONS: MISC
 ***************************************************************/

/// Return a uint32_t from a pointer to a little-endian uint32_t
// in memory.
static uint32_t littleEndianUint32(char *pByte)
{
    uint32_t retValue;
    
    retValue = *pByte;
    retValue += ((uint32_t) *(pByte + 1)) << 8;
    retValue += ((uint32_t) *(pByte + 2)) << 16;
    retValue += ((uint32_t) *(pByte + 3)) << 24;
    
    return  retValue;
}

/// Reset the normally retained variables.
static void resetRetained()
{
    LOG_MSG("Resetting retained memory to defaults.\n");
    memset (&r, 0, sizeof (r));
    strcpy (r.key, RETAINED_INITIALISED);
    r.swVersion = SW_VERSION;
    debugInd(DEBUG_IND_RETAINED_RESET);
}

/// Reset the separate set of "really retained" variables.
static void resetReallyRetained()
{
    LOG_MSG("Resetting really retained memory to defaults.\n");
    memset (&rr, 0, sizeof (rr));
    strcpy (rr.key, RETAINED_INITIALISED);
    rr.swVersion = SW_VERSION;
    debugInd(DEBUG_IND_REALLY_RETAINED_RESET);
}

/// Handle the accelerometer interrupt, returning true if activity
// was detected, otherwise false.
static bool handleInterrupt()
{
    bool activityDetected = false;
    
    if (accelerometerConnected) {
        AccelerometerAdxl345::EventsBitmap_t eventsBitmap;
        // Disable interrupts
        CriticalSectionLock lock;
        accelerometer.read(&r.accelerometerReading.x, &r.accelerometerReading.y, &r.accelerometerReading.z);
        eventsBitmap = accelerometer.handleInterrupt();
        // Re-enable interrupts
        CriticalSectionLock unlock;
        
        // Flag if we're in motion
        if (eventsBitmap & AccelerometerAdxl345::EVENT_ACTIVITY) {
            activityDetected = true;
            debugInd(DEBUG_IND_ACTIVITY);
        }
    }
    
    return activityDetected;
}

/****************************************************************
 * STATIC FUNCTIONS: DEBUG
 ***************************************************************/

/// Use the LED for debug indications.
static void debugInd(DebugInd_t debugInd)
{
    
    switch (debugInd) {
        case DEBUG_IND_OFF:
            debugLed = 1;
        break;
        case DEBUG_IND_RETAINED_RESET:
            // One long flash
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(25);
        break;
        case DEBUG_IND_REALLY_RETAINED_RESET:
            // Two long flashes
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(25);
        break;
        case DEBUG_IND_TOGGLE:
            // This one is the lowest-cost debug option
            debugLed = !debugLed;
        break;
        case DEBUG_IND_GNSS_FIX:
            // Two flashes
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(25);
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(25);
            debugLed = 1;
            wait_ms(25);
        break;
        case DEBUG_IND_ACTIVITY:
            // One flash
            debugLed = 1;
            wait_ms(25);
            debugLed = 0;
            wait_ms(25);
            debugLed = 1;
            wait_ms(25);
        break;
        case DEBUG_IND_BOOT_COMPLETE:
            // Three long flashes
            debugLed = 1;
            wait_ms(250);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(250);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(250);
            debugLed = 0;
            wait_ms(500);
            debugLed = 1;
            wait_ms(250);
        break;
        default:
        break;
    }
}

/// Print an array of bytes as hex.
static void printHex(const char *pBytes, uint32_t lenBytes)
{
    char hexChar[2];

    for (uint32_t x = 0; x < lenBytes; x++)
    {
        hexChar[0] = hexTable[(*pBytes >> 4) & 0x0f]; // upper nibble
        hexChar[1] = hexTable[*pBytes & 0x0f]; // lower nibble
        pBytes++;
        LOG_MSG("%.2s-", hexChar);
    }
}

/// Print out retained RAM
static void debugPrintRetained()
{
    uint32_t x;
    
    LOG_MSG("Printing out retained RAM:\n");
    LOG_MSG("  key: \"%s\".\n", r.key); 
    LOG_MSG("  swVersion: %d.\n", (int) r.swVersion);
    LOG_MSG("  warmStart: %d.\n", r.warmStart);
    LOG_MSG("  gnssOn: %d.\n", r.gnssOn);
    LOG_MSG("  sleepStartSeconds: %d (UTC %s).\n", (int) r.sleepStartSeconds, timeString(r.sleepStartSeconds));
    LOG_MSG("  minSleepPeriodSeconds: %d.\n", (int) r.minSleepPeriodSeconds);
    LOG_MSG("  sleepForSeconds: %d.\n", (int) r.sleepForSeconds);
    LOG_MSG("  modemStaysAwake: %d.\n", (int) r.modemStaysAwake);
    LOG_MSG("  imei: \"%s\".\n", r.imei);
    LOG_MSG("  lastGnssSeconds: %d (UTC %s).\n", (int) r.lastGnssSeconds, timeString(r.lastGnssSeconds));
    LOG_MSG("  lastTelemetrySeconds: %d (UTC %s).\n", (int) r.lastTelemetrySeconds, timeString(r.lastTelemetrySeconds));
    LOG_MSG("  lastStatsSeconds: %d (UTC %s).\n", (int) r.lastStatsSeconds, timeString(r.lastStatsSeconds));
    LOG_MSG("  lastReportSeconds: %d (UTC %s).\n", (int) r.lastReportSeconds, timeString(r.lastReportSeconds));
    LOG_MSG("  lastMotionSeconds: %d (UTC %s).\n", (int) r.lastMotionSeconds, timeString(r.lastMotionSeconds));
    LOG_MSG("  lastColdStartSeconds: %d (UTC %s).\n", (int) r.lastColdStartSeconds, timeString(r.lastColdStartSeconds));
    LOG_MSG("  gnssFixRequested: %d.\n", r.gnssFixRequested);
    LOG_MSG("  nextFreeRecord: %u.\n", (unsigned int) r.nextFreeRecord);
    LOG_MSG("  nextPubRecord: %u.\n", (unsigned int) r.nextPubRecord);
    LOG_MSG("  numRecordsQueued: %u.\n", (unsigned int) r.numRecordsQueued);

    for (x = 0; x < sizeof (r.records) / sizeof (r.records[0]); x++) {
        if (r.records[x].isUsed) {
            LOG_MSG("    %02d: type %d, contents \"%s\".\n", (int) (x + 1), r.records[x].type, r.records[x].contents);
        } else {
            LOG_MSG("    %02d: empty.\n", (int) (x + 1));
        }
    }

    LOG_MSG("  powerSaveTime: %d (UTC %s).\n", (int) r.powerSaveTime, timeString(r.powerSaveTime));
    LOG_MSG("  numLoops: %u.\n", (unsigned int) r.numLoops);
    LOG_MSG("  numLoopsMotionDetected: %u.\n", (unsigned int) r.numLoopsMotionDetected);
    LOG_MSG("  numLoopsLocationNeeded: %u.\n", (unsigned int) r.numLoopsLocationNeeded);
    LOG_MSG("  numLoopsGnssOn: %u.\n", (unsigned int) r.numLoopsGnssOn);
    LOG_MSG("  numLoopsGnssFix: %u.\n", (unsigned int) r.numLoopsGnssFix);
    LOG_MSG("  numLoopsLocationValid: %u.\n", (unsigned int) r.numLoopsLocationValid);
    LOG_MSG("  totalPowerSaveSeconds: %u.\n", (unsigned int) r.totalPowerSaveSeconds);
    LOG_MSG("  gnssPowerOnTime: %d (UTC %s).\n", (int) r.gnssPowerOnTime, timeString(r.gnssPowerOnTime));
    LOG_MSG("  gnssSeconds: %u.\n", (unsigned int) r.gnssSeconds);
    LOG_MSG("  totalGnssSeconds: %u.\n", (unsigned int) r.totalGnssSeconds);
    LOG_MSG("  numPublishAttempts: %u.\n", (unsigned int) r.numPublishAttempts);
    LOG_MSG("  numPublishFailed: %u.\n", (unsigned int) r.numPublishFailed);
    LOG_MSG("  numConnectAttempts: %u.\n", (unsigned int) r.numConnectAttempts);
    LOG_MSG("  numConnectFailed: %u.\n", (unsigned int) r.numConnectFailed);
    LOG_MSG("  numStarts: %u.\n", (unsigned int) r.numStarts);
    LOG_MSG("  accelerometerReading: x %d, y %d, z %d.\n", r.accelerometerReading.x, r.accelerometerReading.y, r.accelerometerReading.z);
    LOG_MSG("  numFatals: %d (", (int) rr.numFatals);
    for (x = 0; x < sizeof (rr.fatalList) / sizeof (rr.fatalList[0]); x++) {
        LOG_MSG("%02d-", rr.fatalList[x]);
    }
    LOG_MSG(").\n");
    LOG_MSG("  numLogFlagsEntries: %d.\n", rr.numLogFlagsEntries);
    for (x = 0; x < rr.numLogFlagsEntries; x++) {
        LOG_MSG("%02d:  %s - ", (int) (x + 1), timeString(rr.logFlagsEntries[x].timestamp));
        logFlagsAsString(rr.logFlagsEntries[x].flags);
    }
}

/// Start a new entry in the log flags.
static void addLogFlagsEntry()
{
    int32_t x;
    
    // Increment the count
    rr.numLogFlagsEntries++;
    if (rr.numLogFlagsEntries > sizeof (rr.logFlagsEntries) / sizeof (rr.logFlagsEntries[0])) {
        rr.numLogFlagsEntries = sizeof (rr.logFlagsEntries) / sizeof (rr.logFlagsEntries[0]);
        // Shuffle the entries down
        for (x = 1; x < rr.numLogFlagsEntries; x++) {
            rr.logFlagsEntries[x - 1].flags = rr.logFlagsEntries[x].flags;
            rr.logFlagsEntries[x - 1].timestamp = rr.logFlagsEntries[x].timestamp;
        }
    }
    
    // Set up the new one
    rr.logFlagsEntries[rr.numLogFlagsEntries - 1].flags = 0;
    rr.logFlagsEntries[rr.numLogFlagsEntries - 1].timestamp = time(NULL);
    
}

// Set a log flag in the current log flags entry
static void setLogFlag(uint32_t flag)
{
    if (rr.numLogFlagsEntries > 0) {
        rr.logFlagsEntries[rr.numLogFlagsEntries - 1].flags |= flag;
    }
}

// Clear a log flag in the current log flags entry
static void clearLogFlag(uint32_t flag)
{
    if (rr.numLogFlagsEntries > 0) {
        rr.logFlagsEntries[rr.numLogFlagsEntries - 1].flags &= ~flag;
    }
}

// Print a string describing a set of log flags
static void logFlagsAsString(uint32_t flags)
{
    if (flags & LOG_FLAG_STARTUP_NOT_LOOP_ENTRY)     {
        LOG_MSG("Startup: ");
        if (flags & LOG_FLAG_STARTUP_WARM_NOT_COLD) {
            LOG_MSG("warm start, ");
        } else {
            LOG_MSG("cold start, ");
        }
    } else {
        LOG_MSG("Loop: ");
    }
    if (flags & LOG_FLAG_TIME_ESTABLISHED_VIA_RTC)     {
        LOG_MSG("time from RTC, ");
    }
    if (flags & LOG_FLAG_TIME_ESTABLISHED_VIA_NETWORK)     {
        LOG_MSG("time from network, ");
    }
    if (flags & LOG_FLAG_TIME_ESTABLISHED_VIA_GNSS)     {
        LOG_MSG("time from GNSS, ");
    }
    if (flags & LOG_FLAG_TIME_NOT_ESTABLISHED)     {
        LOG_MSG("time not established, ");
    }
    if (flags & LOG_FLAG_START_TIME_PASSED)     {
        LOG_MSG("start time passed, ");
    }
    if (flags & LOG_FLAG_START_TIME_FULL_WORKING_DAY_OPERATION_PASSED)     {
        LOG_MSG("in full-working day operation, ");
    }
    if (flags & LOG_FLAG_IN_WORKING_DAY)     {
        LOG_MSG("during the working day, ");
    }
    if (flags & LOG_FLAG_MOTION_DETECTED)     {
        LOG_MSG("motion detected, ");
    }
    if (flags & LOG_FLAG_WOKEN_UP_EARLY)     {
        LOG_MSG("awoke early, ");
    }
    if (flags & LOG_FLAG_TELEMETRY_RECORD_QUEUED)     {
        LOG_MSG("telemetry queued, ");
    }
    if (flags & LOG_FLAG_STATS_RECORD_QUEUED)     {
        LOG_MSG("stats queued, ");
    }
    if (flags & LOG_FLAG_GNSS_FIX_ACHIEVED)     {
        LOG_MSG("GNSS fix, ");
    }
    if (flags & LOG_FLAG_GNSS_FIX_FAILED)     {
        LOG_MSG("no GNSS fix, ");
    }
    if (flags & LOG_FLAG_GNSS_RECORD_QUEUED)     {
        LOG_MSG("GNSS queued, ");
    }
    if (flags & LOG_FLAG_SEND_ATTEMPTED)     {
        LOG_MSG("send attempted, ");
    }
    if (flags & LOG_FLAG_MODEM_CONNECTED)     {
        LOG_MSG("modem connected, ");
    }
    if (flags & LOG_FLAG_CONNECT_FAILED)     {
        LOG_MSG("connect failed, ");
    }
    if (flags & LOG_FLAG_PUBLISH_SUCCEEDED)     {
        LOG_MSG("publish succeeded, ");
    }
    if (flags & LOG_FLAG_PUBLISH_FAILED)     {
        LOG_MSG("publish failed, ");
    }
    if (flags & LOG_FLAG_GNSS_ON_NOT_OFF)     {
        LOG_MSG("GNSS on at the end, ");
    } else {
        LOG_MSG("GNSS off at the end, ");
    }
    if (flags & LOG_FLAG_WAKE_ON_INTERRUPT)     {
        LOG_MSG("will wake on interrupt, ");
    }
    // Don't print this for a startup entry as it is irrelevant
    if (!(flags & LOG_FLAG_STARTUP_NOT_LOOP_ENTRY)) {
        if (flags & LOG_FLAG_DEEP_SLEEP_NOT_CLOCK_STOP)     {
            LOG_MSG("using deep sleep, ");
        } else {
            LOG_MSG("using clock-stop sleep, ");
        }
    }
    if (flags & LOG_FLAG_FREE_1)     {
        LOG_MSG("FREE_1, ");
    }
    if (flags & LOG_FLAG_FREE_2)     {
        LOG_MSG("FREE_2, ");
    }
    if (flags & LOG_FLAG_FREE_3)     {
        LOG_MSG("FREE_3, ");
    }
    if (flags & LOG_FLAG_FREE_4)     {
        LOG_MSG("FREE_4, ");
    }
    LOG_MSG("END.\n");
}

/****************************************************************
 * STATIC FUNCTIONS: GNSS
 ***************************************************************/

/// Read a UBX format GNSS message into a buffer.  If waitMilliseconds
// is zero then don't hang around except for the intercharacter delay,
// otherwise wait up to waitMilliseconds for the message.
static uint32_t readGnssMsg(char *pBuffer, uint32_t bufferLen, int32_t waitMilliseconds)
{
    uint32_t returnCode;
    uint32_t length = 0;
    Timer timer;

    timer.start();
    while (((returnCode = gnss.getMessage(pBuffer, bufferLen)) > 0) &&
           (timer.read_ms() < waitMilliseconds) &&
           (length == 0)) {
        if (PROTOCOL(returnCode) == GnssParser::UBX) {
            length = LENGTH(returnCode);
        }
    }
    timer.stop();

    
    if (length > 0) {
        LOG_MSG("Read %d byte(s): ", (int) length);
        printHex(pBuffer, length);
        LOG_MSG("\n");
    }
    
    if (length >= bufferLen) {
        LOG_MSG("WARNING: hit end of buffer (%d bytes).\n", (int) length);
    }
    
    return length;
}

/// Check for an ack for a GNSS message.
// See ublox7-V14_ReceiverDescrProtSpec section 33 (ACK).
// NOTE: this re-uses msgBuffer.
static bool waitAckGnssMsg(uint8_t msgClass, uint8_t msgId)
{
    bool ack = false;
    bool gotAckOrNack = false;
    Timer timer;

    LOG_MSG("  > ");
    // Wait around for a message that is an ack
    timer.start();
    while (!gotAckOrNack && (timer.read_ms() < GNSS_WAIT_FOR_ACK_MILLISECONDS)) {
        if (readGnssMsg(msgBuffer, sizeof (msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) == 10) { // 10 is the Ack message size
            // Ack is  0xb5-62-05-00-02-00-msgclass-msgid-crcA-crcB
            // Nack is 0xb5-62-05-01-02-00-msgclass-msgid-crcA-crcB
            if ((msgBuffer[4] == 2) && (msgBuffer[5] == 0) && (msgBuffer[6] == msgClass) && (msgBuffer[7] == msgId) && (msgBuffer[2] == 0x05)) {
                gotAckOrNack = true;
                if (msgBuffer[3] == 0x01) {
                    ack = true;
                    LOG_MSG("  > [Ack]\n");
                } else {
                    LOG_MSG("!!> [Nack]\n");
                }
            }
        }
    }
    timer.stop();

    return ack;
}

/// Configure the GNSS module.
// NOTE: it is up to the caller to make sure that the module is powered up.
static bool configureGnss()
{
    bool success1 = false;
    bool success2 = false;

    LOG_MSG("Configuring GNSS...\n");
    
    // See ublox7-V14_ReceiverDescrProtSpec section 35.9 (CFG-NAV5)
    LOG_MSG("Setting automotive mode (CFG-NAV5)...\n");
    memset (msgBuffer, 0, sizeof (msgBuffer));
    msgBuffer[0] = 0x00; // Set dynamic config only
    msgBuffer[1] = 0x01;
    msgBuffer[2] = 0x04; // Automotive
    success1 = (gnss.sendUbx(0x06, 0x24, msgBuffer, 36) >= 0) && waitAckGnssMsg(0x06, 0x24);

    // See ublox7-V14_ReceiverDescrProtSpec section 35.2 (CFG-CFG)
    LOG_MSG("Storing settings in battery-backed RAM (CFG-CFG)...\n");
    memset (msgBuffer, 0, sizeof (msgBuffer));
     // Set all items in all bitmaps so that we clear, save and re-load
    msgBuffer[0] = 0x00;
    msgBuffer[1] = 0x00;
    msgBuffer[2] = 0x06;
    msgBuffer[3] = 0x1F;
    msgBuffer[4] = 0x00;
    msgBuffer[5] = 0x00;
    msgBuffer[6] = 0x06;
    msgBuffer[7] = 0x1F;
    msgBuffer[8] = 0x00;
    msgBuffer[9] = 0x00;
    msgBuffer[10] = 0x06;
    msgBuffer[11] = 0x1F;
    msgBuffer[12] = 0x01;  // Save in BBR
    success2 = (gnss.sendUbx(0x06, 0x09, msgBuffer, 13) >= 0) && waitAckGnssMsg(0x06, 0x09);

    return success1 && success2;
}

/// Return true if we have a GNSS fix and fill in any given parameters (any of which may be NULL).
// NOTE: it is up to the caller to make sure that the module is powered up.
static bool gotGnssFix(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop)
{
    bool gotFix = false;
    float longitude;
    float latitude;
    float elevation = 0;
    float hdop = GNSS_INVALID_HDOP;
    
    // See ublox7-V14_ReceiverDescrProtSpec section 39.7 (NAV-PVT)
    LOG_MSG("Checking fix (NAV-PVT)...\n");
    if (gnss.sendUbx(0x01, 0x07, NULL, 0) >= 0) {
        if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
#ifdef GNSS_FIX_2D
            // Have we got at least a 2D fix?
            if ((msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) || (msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE] == 0x02)) {
#else
            // Have we got at least a 3D fix?
            if (msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) {
#endif
                LOG_MSG("%dD fix achieved.\n", msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE]);
                if ((msgBuffer[21 + GNSS_UBX_PROTOCOL_HEADER_SIZE] & 0x01) == 0x01) {
                    LOG_MSG("gnssFixOK flag is set.\n");
                    gotFix = true;
                    longitude = ((float) (int32_t) littleEndianUint32(&(msgBuffer[24 + GNSS_UBX_PROTOCOL_HEADER_SIZE]))) / 10000000;
                    latitude = ((float) (int32_t) littleEndianUint32(&(msgBuffer[28 + GNSS_UBX_PROTOCOL_HEADER_SIZE]))) / 10000000;
                    elevation = ((float) (int32_t) littleEndianUint32(&(msgBuffer[36 + GNSS_UBX_PROTOCOL_HEADER_SIZE]))) / 1000;
                    LOG_MSG("  > %d satellites used.\n", msgBuffer[23 + GNSS_UBX_PROTOCOL_HEADER_SIZE]);
                    LOG_MSG("  > Latitude %.6f.\n", latitude);
                    LOG_MSG("  > Longitude %.6f.\n", longitude);
                    if (msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) {
                        LOG_MSG("  > Elevation %.2f.\n", elevation);
                    } else {
                        LOG_MSG("  > Elevation ---.\n");
                    }
                    
                    // Now get HDOP
                    // See ublox7-V14_ReceiverDescrProtSpec section 39.4 (NAV-DOP)
                    LOG_MSG("Getting HDOP (NAV-DOP)...\n");
                    if (gnss.sendUbx(0x01, 0x04, NULL, 0) >= 0) {
                        if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                            hdop = ((float) ((uint32_t) (msgBuffer[12 + GNSS_UBX_PROTOCOL_HEADER_SIZE]) + ((uint32_t) (msgBuffer[13 + GNSS_UBX_PROTOCOL_HEADER_SIZE]) << 8))) / 100;
                            LOG_MSG("  > HDOP %.2f.\n", hdop);
                        } else {
                            LOG_MSG("No response.\n");
                        }
                    }

                    if (pLatitude != NULL) {
                        *pLatitude = latitude;
                    }
                    if (pLongitude != NULL) {
                        *pLongitude = longitude;
                    }
                    if (pElevation != NULL) {
                        *pElevation = elevation;
                    }
                    if (pHdop != NULL) {
                        *pHdop = hdop;
                    }
                } else {
                    LOG_MSG("gnssFixOK flag is NOT set (flags are 0x%02x).\n", msgBuffer[21 + GNSS_UBX_PROTOCOL_HEADER_SIZE]);
                }
            } else {
                LOG_MSG("No fix (fix is %d).\n", msgBuffer[20 + GNSS_UBX_PROTOCOL_HEADER_SIZE]);
            }
        } else {
            LOG_MSG("No response.\n");
        }
    }
    
    return gotFix;
}

/// Read the time from the GNSS module
// NOTE: it is up to the caller to make sure that the module is powered up.
static bool gnssGetTime(time_t *pUnixTimeUtc)
{
    time_t gnssTime = 0;
    bool success = false;
    uint32_t months;
    uint32_t year;
    uint32_t x;
    bool switchGnssOffAgain = !gnssIsOn();

    // See ublox7-V14_ReceiverDescrProtSpec section 39.13 (NAV-TIMEUTC)
    LOG_MSG("Reading time from GNSS (NAV-TIMEUTC)...\n");
    if (gnss.sendUbx(0x01, 0x21, NULL, 0) >= 0) {
        if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
            // Check the validity flag
            if (msgBuffer[19 + GNSS_UBX_PROTOCOL_HEADER_SIZE] & 0x04) {
                // Year 1999-2099, so need to adjust to get year since 1970
                year = ((uint32_t) (msgBuffer[12 + GNSS_UBX_PROTOCOL_HEADER_SIZE])) + ((uint32_t) (msgBuffer[13 + GNSS_UBX_PROTOCOL_HEADER_SIZE]) << 8) - 1999 + 29;
                // Month (1 to 12), so take away 1 to make it zero-based
                months = msgBuffer[14 + GNSS_UBX_PROTOCOL_HEADER_SIZE] - 1;
                months += year * 12;
                // Work out the number of seconds due to the year/month count
                for (x = 0; x < months; x++) {
                    if (isLeapYear ((x / 12) + 1970)) {
                        gnssTime += daysInMonthLeapYear[x % 12] * 3600 * 24;
                    } else {
                        gnssTime += daysInMonth[x % 12] * 3600 * 24;
                    }
                }
                // Day (1 to 31)
                gnssTime += ((uint32_t) msgBuffer[15 + GNSS_UBX_PROTOCOL_HEADER_SIZE] - 1) * 3600 * 24;
                // Hour (0 to 23)
                gnssTime += ((uint32_t) msgBuffer[16 + GNSS_UBX_PROTOCOL_HEADER_SIZE]) * 3600;
                // Minute (0 to 59)
                gnssTime += ((uint32_t) msgBuffer[17 + GNSS_UBX_PROTOCOL_HEADER_SIZE]) * 60;
                // Second (0 to 60)
                gnssTime += msgBuffer[18 + GNSS_UBX_PROTOCOL_HEADER_SIZE];
                
                LOG_MSG("GNSS time is %s.\n", timeString(gnssTime));
    
                success = true;
                if (pUnixTimeUtc != NULL) {
                    *pUnixTimeUtc = gnssTime;
                }
            } else {
                LOG_MSG("GNSS time not valid.\n");
            }
        } else {
            LOG_MSG("No response.\n");
        }
    } else {
        LOG_MSG("Read request failed.\n");
    }

    // Switch GNSS off again if it was off to start with
    if (switchGnssOffAgain) {
        gnssOff();
    }
    
    return success;
}

/// Determine whether GNSS has the necessary data to be
// put into power save state.  This is to have downloaded
// ephemeris data from sufficient satellites and to have
// calibrated its RTC.  There is also a timeout check.
// NOTE: it is up to the caller to check separately if we
// have the required accuracy of fix.
static bool gnssCanPowerSave()
{
    bool canPowerSave = false;

    LOG_MSG("Checking if GNSS can power save...\n");
    
    if (gnssIsOn()) {
        LOG_MSG("GNSS has been on for %d second(s).\n", (int) (time(NULL) - r.gnssPowerOnTime));
        // Time GNSS out if we're outside the maximum time and are in full working-day
        // operation.  NOTE: if we're in "slow mode" operation, rather than full
        // working-day operation, we want to try our damndest and will power GNSS
        // off anyway at the end of the short "slow mode" wakeup.
        if ((time(NULL) - r.gnssPowerOnTime < GNSS_MAX_ON_TIME_SECONDS) || (time(NULL) < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC)) {
            uint32_t powerSaveState = 0;
            uint32_t numEntries = 0;
            uint32_t numEphemerisData = 0;
            uint32_t totalCN = 0;
            uint32_t peakCN = 0;
            
            // See ublox7-V14_ReceiverDescrProtSpec section 38.2 (MON-HW)
            LOG_MSG("Checking if RTC is calibrated (MON-HW)...\n");
            if (gnss.sendUbx(0x0A, 0x09, NULL, 0) >= 0) {
                if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                    if ((msgBuffer[22 + GNSS_UBX_PROTOCOL_HEADER_SIZE] & 0x01) == 0x01) {
                        // If the rtcCalib bit is set, we're doing good...
                        powerSaveState++;
                        LOG_MSG("RTC is calibrated.\n");
                    } else {
                        LOG_MSG("RTC is NOT calibrated.\n");
                    }
                } else {
                    LOG_MSG("No response.\n");
                }
            }
            
            // See ublox7-V14_ReceiverDescrProtSpec section 39.11 (NAV-SVINFO)
            LOG_MSG("Checking ephemeris data (NAV-SVINFO)...\n");
            if (gnss.sendUbx(0x01, 0x30, NULL, 0) >= 0) {
                if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                    // Check how many satellites we have ephemeris data for
                    numEntries = msgBuffer[4 + GNSS_UBX_PROTOCOL_HEADER_SIZE];
                    LOG_MSG("  > %d entry/entries in the list", (int) numEntries);
                    
                    if (numEntries > 0) {
                        LOG_MSG(": \n");
                        uint32_t i = GNSS_UBX_PROTOCOL_HEADER_SIZE  + 8; // 8 is offset to start of satellites array
                        // Now need to check that enough are used for navigation
                        for (uint32_t x = 0; x < numEntries; x++) {
                            // Print out the info
                            LOG_MSG("  > chn %3d", msgBuffer[i]);
                            LOG_MSG(", svid %3d", msgBuffer[i + 1]);
                            LOG_MSG(", flags 0x%02x", msgBuffer[i + 2]);
                            LOG_MSG(", quality 0x%02x", msgBuffer[i + 3]);
                            LOG_MSG(", C/N (dBHz) %3d", msgBuffer[i + 4]);
                            LOG_MSG(", elev %3d", msgBuffer[i + 5]);
                            LOG_MSG(", azim %5d", (int) (((uint16_t) msgBuffer[i + 6]) + ((uint16_t) (msgBuffer[i + 7] << 8))));
                            LOG_MSG(", prRes %10d", (int) littleEndianUint32(&(msgBuffer[i + 8])));
                            if ((msgBuffer[i + 2] & 0x01) == 0x01) { // 2 is offset to flags field in a satellite block
                                numEphemerisData++;
                                totalCN += msgBuffer[i + 4];
                                if (msgBuffer[i + 4] > peakCN) {
                                    peakCN = msgBuffer[i + 4];
                                }
                                LOG_MSG(", used for navigation.\n");
                            } else {
                                LOG_MSG(", NOT usable.\n");
                            }
                            i += 12; // 12 is the size of a satellite block
                        }
                        LOG_MSG("  > %d satellite(s) used for navigation with %d required.\n", (int) numEphemerisData, GNSS_MIN_NUM_EPHEMERIS_DATA);
                        if (numEphemerisData >= GNSS_MIN_NUM_EPHEMERIS_DATA) {
                            // Doing even better
                            powerSaveState++;
                        }
                    } else {
                        LOG_MSG(".\n");
                    }
                } else {
                    LOG_MSG("No response.\n");
                }
                
                // Calculate the informational variables
                if (numEphemerisData > 0) {
                    gnssNumSatellitesUsable = numEphemerisData;
                    gnssPeakCNUsed = peakCN;
                    gnssAverageCNUsed = totalCN / numEphemerisData;
                }
        
                // Determine the outcome
                if (powerSaveState == 2) {
                    canPowerSave = true;
                    LOG_MSG("GNSS can now power save.\n");
                } else {
                    LOG_MSG("GNSS NOT yet ready to power save.\n");
                }
            }
        } else {
            canPowerSave = true;
            LOG_MSG("GNSS isn't ready but we're out of time so put GNSS to sleep anyway.\n");
        }
    } else {
        canPowerSave = true;
        LOG_MSG("GNSS is already off.\n");
    }
    
    return canPowerSave;
}

/// Switch GNSS on, returning the time that GNSS was switched on.
static time_t gnssOn()
{
    if (!r.gnssOn) {        
        r.gnssOn = gnss.init();
        if (r.gnssOn) {
            // Log the time that GNSS was switched on
            r.gnssPowerOnTime = time(NULL);
            LOG_MSG("GNSS initialised.\n");
            setLogFlag(LOG_FLAG_GNSS_ON_NOT_OFF);
        }
    }

    return r.gnssPowerOnTime;
}

/// Switch GNSS off.
static bool gnssOff()
{
    // Record the duration that GNSS was on for
    uint32_t x = time(NULL) - r.gnssPowerOnTime;
    
    // Ignore silly values which could occur if
    // the timebase underneath us is updated between
    // gnssPowerOnTime and now
    if (x < 31536000) { // 1 year
        r.totalGnssSeconds += x;
    }

    gnss.powerOff();
    r.gnssOn = false;
    clearLogFlag(LOG_FLAG_GNSS_ON_NOT_OFF);
    
    return true;
}

/// Return true if GNSS is on, otherwise false.
static bool gnssIsOn()
{
    return r.gnssOn;
}

/// Update the GNSS, making sure it's been on for
// enough time to give us a fix. pLatitude, pLongitude, pElevation,
// pHdop and are filled in with fix values and their accuracy
// if a fix is achieved (and true is returned), otherwise
// they are left alone.
static bool gnssUpdate(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop)
{
    bool fixAchieved = false;
    uint32_t startTimeSeconds = time(NULL);

    LOG_MSG("Checking for GNSS fix for up to %d second(s):\n", GNSS_FIX_TIME_SECONDS);

    // Make sure GNSS is switched on
    gnssOn();

    while (!fixAchieved && (time(NULL) - startTimeSeconds < GNSS_FIX_TIME_SECONDS)) {
        // Check for a fix
        fixAchieved = gotGnssFix(pLatitude, pLongitude, pElevation, pHdop);
        // Sleep while we're waiting
        if (!fixAchieved) {
            wait_ms(GNSS_CHECK_INTERVAL_SECONDS * 1000);
        }
    }

    if (fixAchieved){
        LOG_MSG("Fix achieved in %d second(s): latitude: %.6f, longitude: %.6f, elevation: %.3f m",
                (int) (time(NULL) - startTimeSeconds), *pLatitude, *pLongitude, *pElevation);
        setLogFlag(LOG_FLAG_GNSS_FIX_ACHIEVED);
        if (*pHdop != 0) {
            LOG_MSG(", HDOP: %.2f.\n", *pHdop);
        } else {
            LOG_MSG(", no HDOP.\n");
        }
        // See ublox7-V14_ReceiverDescrProtSpec section 41.3 (TIM-VRFY)
        LOG_MSG("\nChecking RTC innaccuracy, for info, (TIM-VRFY)...");
        if (gnss.sendUbx(0x0d, 0x06, NULL, 0) >= 0) {
            if (readGnssMsg(msgBuffer, sizeof(msgBuffer), GNSS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                LOG_MSG("  > tow (ms): %d.%0d\n", (int) littleEndianUint32(&(msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE])), \
                                                  (int) littleEndianUint32(&(msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 4])));
                LOG_MSG("  > delta (ms): ");
                bool negative = false;
                int32_t iDelta = (int32_t) littleEndianUint32(&(msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 8]));
                int32_t fDelta = (int32_t) littleEndianUint32(&(msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 12]));
                if (iDelta < 0) {
                    negative = true;
                    iDelta = -iDelta;
                }
                if (fDelta < 0) {
                    negative = true;
                    fDelta = -fDelta;
                }
                if (negative) {
                    LOG_MSG("-");
                }
                LOG_MSG("%d.%0d\n", (int) iDelta, (int) fDelta);
                LOG_MSG("  > week %d\n", (int) ((uint32_t) (msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 16]) + (((uint32_t) msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 17]) << 8)));
                LOG_MSG("  > flags 0x%02x", msgBuffer[GNSS_UBX_PROTOCOL_HEADER_SIZE + 18]);
            } else {
                LOG_MSG("No response.");
            }
        }
    } else {
        setLogFlag(LOG_FLAG_GNSS_FIX_FAILED);
    }
    
    return fixAchieved;
}

/****************************************************************
 * STATIC FUNCTIONS: MODEM
 ***************************************************************/

/// Set the time using a NTP server
static bool cellularGetTime(time_t *pTime)
{
    bool success = false;
    UDPSocket sockTime;
    SocketAddress serverTime;
    SocketAddress senderAddressTime;
    char buf[48];
    int x;
    time_t timestamp = 0;
    
    LOG_MSG("Powering up cellular to get time...\n");
    cellular.init();
    cellular.set_credentials(APN, USERNAME, PASSWORD);
    printf("Connecting to the packet network ...\n");
    for (x = 0; (cellular.connect() != 0) && (x < CONNECT_RETRIES); x++) {
        printf("Retrying (have you checked that an antenna is plugged in and your APN is correct?)...\n");
        wait_ms(1000);
    }
    
    if (cellular.is_connected()) {
        if (cellular.gethostbyname("2.pool.ntp.org", &serverTime) == 0) {
            serverTime.set_port(123);
            printf("\"2.pool.ntp.org\" address: %s on port %d.\n", serverTime.get_ip_address(), serverTime.get_port());
            if (sockTime.open(&cellular) == 0) {
                sockTime.set_timeout(10000);
                printf("Sending time request to \"2.pool.ntp.org\" over UDP socket...\n");
                memset (buf, 0, sizeof(buf));
                *buf = '\x1b';
                if (sockTime.sendto(serverTime, (void *) buf, sizeof(buf)) == sizeof(buf)) {
                    x = sockTime.recvfrom(&senderAddressTime, buf, sizeof (buf));
                    if (x >= 43) {
                        success = true;
                        timestamp |= ((int) *(buf + 40)) << 24;
                        timestamp |= ((int) *(buf + 41)) << 16;
                        timestamp |= ((int) *(buf + 42)) << 8;
                        timestamp |= ((int) *(buf + 43));
                        timestamp -= 2208988800U; // to get the timestamp offset from 1970;
                        LOG_MSG("Network time is %s.\n", timeString(timestamp));
                        if (pTime != NULL) {
                            *pTime = timestamp;
                        }
                    }
                }
                sockTime.close();
            }
        }    
        cellular.disconnect();
    }
    
    return success;
}
 
/// Connect to the network, returning true if successful.
static bool connect()
{
    bool success = false;
    
#ifdef DISABLE_CELLULAR_CONNECTION
    success = true;
#else
    if (cellular.is_connected()) {
        success = true;
        setLogFlag(LOG_FLAG_MODEM_CONNECTED);
    } else {
        r.numConnectAttempts++;
        LOG_MSG("Making sure modem is powered up...\n");
        cellular.init(PIN);
        cellular.set_credentials(APN, USERNAME, PASSWORD);
        LOG_MSG("Connecting to the packet network...\n");
        for (int x = 0; (cellular.connect() != 0) && (x < CONNECT_RETRIES); x++) {
            LOG_MSG("Retrying (have you checked that an antenna is plugged in and your APN is correct?)...\n");
            wait_ms(1000);
        }
        if (cellular.is_connected()) {
            printf("Getting the IP address of \"%s\"...\n", SERVER_ADDRESS);
            if (cellular.gethostbyname(SERVER_ADDRESS, &udpServer) == 0) {
                udpServer.set_port(SERVER_PORT);
                LOG_MSG("\%s\" address: %s on port %d.\n", SERVER_ADDRESS, udpServer.get_ip_address(), udpServer.get_port());
                if (udpSock.open(&cellular) == 0) {
                    success = true;
                    udpSock.set_timeout(WAIT_FOR_PUBLISH_SECONDS * 1000);
                    setLogFlag(LOG_FLAG_MODEM_CONNECTED);
                } else {
                    setLogFlag(LOG_FLAG_CONNECT_FAILED);
                    r.numConnectFailed++;
                    LOG_MSG("Couldn't open UDP socket.\n");
                }
            } else {
                setLogFlag(LOG_FLAG_CONNECT_FAILED);
                r.numConnectFailed++;
                LOG_MSG("Couldn't find host \"%s\".\n", SERVER_ADDRESS);
            }
        } else {
            setLogFlag(LOG_FLAG_CONNECT_FAILED);
            r.numConnectFailed++;
            LOG_MSG("WARNING: connection failed.\n");
        }
    }
#endif
    
    return success;
}

/// Send stuff
static bool send(RecordType_t type, const char *pContents)
{
    bool success = false;
    int typeLen;
    int contentsLen;
    
#ifdef DISABLE_CELLULAR_CONNECTION
    success = true;
#else
    typeLen = strlen(recordTypeString[type]);
    if (typeLen > LEN_RECORD_TYPE) {
        typeLen = LEN_RECORD_TYPE;
    }
    memcpy (sendBuffer, recordTypeString[type], typeLen);

    contentsLen = strlen(pContents);
    if (contentsLen > LEN_RECORD_CONTENTS) {
        contentsLen = LEN_RECORD_CONTENTS;
    }
    memcpy (sendBuffer + typeLen, pContents, contentsLen);
    
    if (udpSock.sendto(udpServer, (void *) sendBuffer, contentsLen + typeLen) == contentsLen + typeLen) {
        success = true;
    }
#endif
    
    return success;
}

/****************************************************************
 * STATIC FUNCTIONS: TIME
 ***************************************************************/

/// Return a string expressing time.
// Note: sometimes gmtime returns the value of zero time,
// even through the time value passed in clearly is not
// zero; not sure why.
static char *timeString(time_t time)
{
    strftime(timeBuf, sizeof (timeBuf), "%F %X", gmtime(&time));
    return timeBuf;
}

/// Check if a year is a leap year
static bool isLeapYear(uint32_t year)
{
    bool leapYear = false;

    if (year % 400 == 0) {
        leapYear = true;
    } else if (year % 4 == 0) {
        leapYear = true;
    }

    return leapYear;
}

/// Make sure we have time sync.
static bool establishTime()
{
    bool success = false;
    time_t newTime = 0;
    time_t tNow = time(NULL);

    if (tNow <= MIN_TIME_UNIX_UTC) {
        LOG_MSG("time(NULL) reported as UTC %s.\n", timeString(tNow));
        if (gnssGetTime(&newTime)) {
            set_time(newTime);
            tNow = time(NULL);
            LOG_MSG("Using GNSS time instead, UTC %s", timeString(tNow));
            setLogFlag(LOG_FLAG_TIME_ESTABLISHED_VIA_GNSS);
        } else {
            // Set time using cellular
            if (cellularGetTime(&newTime)) {
                set_time(newTime);
                tNow = time(NULL);
                LOG_MSG("Using cellular time instead UTC %s.\n", timeString(tNow));
                setLogFlag(LOG_FLAG_TIME_ESTABLISHED_VIA_NETWORK);
            }
        }
    } else {
        setLogFlag(LOG_FLAG_TIME_ESTABLISHED_VIA_RTC);
    }
    
    tNow = time(NULL);
    if (tNow > MIN_TIME_UNIX_UTC) {
        success = true;
        if (r.lastColdStartSeconds <= MIN_TIME_UNIX_UTC) {
            // If we didn't already have time established when we left setup(),
            // reset it to a more correct time here.
            r.lastColdStartSeconds = tNow;
        }
    } else {
        LOG_MSG("WARNING: unable to establish time (time now is %d).\n", (int) tNow);
        setLogFlag(LOG_FLAG_TIME_NOT_ESTABLISHED);
    }

    return success;
}

/// Get the sleep time to the next event given the last time the event occurred and
// it's periodicity
static time_t getSleepTime(time_t lastTime, time_t period)
{
    time_t sleepDuration = period;
    
    if ((lastTime > 0) && (lastTime < time(NULL))) {
        sleepDuration = period - (time(NULL) - lastTime);
        if (sleepDuration < 0) {
            sleepDuration = 0;
        }
    }
    
    return sleepDuration;
}

/// Sort out our timings after waking up to do something
static time_t setTimings(uint32_t secondsSinceMidnight, bool atLeastOneGnssReportSent, bool fixAchieved, time_t *pMinSleepPeriodSeconds)
{
    time_t x;
    uint32_t numIntervalsPassed;
    time_t minSleepPeriodSeconds = MIN_MOTION_PERIOD_SECONDS;
    time_t sleepForSeconds = MIN_MOTION_PERIOD_SECONDS;

    if (time(NULL) >= START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
        // Begin by setting the sleep time to the longest possible time
        sleepForSeconds = TELEMETRY_PERIOD_SECONDS;
        LOG_MSG("In full working day operation, setting next wake-up in %d second(s).\n", (int) sleepForSeconds);
        
        // Check if we're doing a GNSS fix (after motion was triggered)
        if (gnssIsOn()) {
            // Sleep for the minimum period
            sleepForSeconds = MIN_MOTION_PERIOD_SECONDS;
            LOG_MSG("  Still looking for a GNSS fix so wake-up again in %d second(s).\n", (int) sleepForSeconds);
        } else {
            // If we didn't achieve a fix then set the minimum sleep period
            // to a larger value to avoid wasting power in places where there
            // is motion but no GNSS coverage
            if (r.gnssFixRequested && !fixAchieved) {
                LOG_MSG("  GNSS fix failure, setting min sleep period to %d second(s).\n", MAX_MOTION_PERIOD_SECONDS);
                minSleepPeriodSeconds = MAX_MOTION_PERIOD_SECONDS;
                // Queue a GNSS report with dummy values to indicate that we tried and failed
                queueGnssReport(GNSS_INVALID_ANGLE, GNSS_INVALID_ANGLE, true, GNSS_INVALID_HDOP);
            }
            // Reset the fix request flag
            r.gnssFixRequested = false;
        }

        // The things that can wake us up are queueing telemetry reports, queuing stats reports
        // and actually sending off the accumulated reports
        x = getSleepTime(r.lastTelemetrySeconds, TELEMETRY_PERIOD_SECONDS);
        LOG_MSG("  Next wake-up to record telemetry is in %d second(s) (last was UTC %s).\n",
                 (int) x, timeString(r.lastTelemetrySeconds));
        if (x < sleepForSeconds) {
            sleepForSeconds = x;
            LOG_MSG("    Next wake-up set to %d second(s).\n", (int) x);
        }
        x = getSleepTime (r.lastStatsSeconds, statsPeriodSeconds);
        LOG_MSG("  Next wake-up to record stats is in %d second(s) (last was UTC %s).\n",
                (int) x, timeString(r.lastStatsSeconds));
        if (x < sleepForSeconds) {
            sleepForSeconds = x;
            LOG_MSG("    Next wake-up set to %d second(s).\n", (int) x);
        }
        if (r.numRecordsQueued > 0) {
            x = getSleepTime (r.lastReportSeconds, REPORT_PERIOD_SECONDS);
            LOG_MSG("  Next wake-up to send the %d queued report(s) is in %d second(s) (last was UTC %s).\n",
                    (int) r.numRecordsQueued, (int) x, timeString(r.lastReportSeconds));
            if (x < sleepForSeconds) {
                sleepForSeconds = x;
                LOG_MSG("    Next wake-up set to %d second(s).\n", (int) x);
            }
        } else {
            LOG_MSG("  No records queued so not waking-up to send them.\n");
        }
        
        // Make sure that the minSleepPeriodSeconds doesn't prevent us waking up for one of the above
        if (sleepForSeconds < minSleepPeriodSeconds) {
            LOG_MSG("  Min sleep time, %d, is greater than the sleep time we want, setting min sleep time to %d second(s).\n", (int) minSleepPeriodSeconds, (int) sleepForSeconds);
            minSleepPeriodSeconds = sleepForSeconds;
        }
    } else {
        LOG_MSG("In \"slow mode\" operation, next wake-up set to %d second(s).\n", (int) sleepForSeconds);
        // But, if at least one GNSS report has been sent, or we've ended this slow
        // operation wake-up, then we can go to deep sleep until the interval expires
        if ((atLeastOneGnssReportSent && fixAchieved) || (time(NULL) - r.lastColdStartSeconds > SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS)) {
            x = secondsSinceMidnight - START_OF_WORKING_DAY_SECONDS;
            if (x < 0) {
                x = 0;
            }
            numIntervalsPassed = x / SLOW_MODE_INTERVAL_SECONDS;
            if (numIntervalsPassed == 0) {
                LOG_MSG("  Initialisation wake-up in \"slow mode\" is complete.\n");
            } else {
                LOG_MSG("  This \"slow mode\" wake-up (%d of %d each working day) is complete.\n", (int) numIntervalsPassed, SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY);
            }
            sleepForSeconds = START_OF_WORKING_DAY_SECONDS + (numIntervalsPassed + 1) * SLOW_MODE_INTERVAL_SECONDS -
                              secondsSinceMidnight;
            if (numIntervalsPassed >= SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY) {
                sleepForSeconds = (3600 * 24) - secondsSinceMidnight + START_OF_WORKING_DAY_SECONDS + SLOW_MODE_INTERVAL_SECONDS;
                LOG_MSG("  Next \"slow mode\" wake-up is tomorrow.\n");
            }
            LOG_MSG("  Next \"slow mode\" wake-up set to %d second(s).\n", (int) sleepForSeconds);
            if (time(NULL) + sleepForSeconds >= START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
                sleepForSeconds = START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC - time(NULL);
                LOG_MSG("  But by that time we would be in full working day operation, so sleeping for %d second(s) instead.\n", (int) sleepForSeconds);
            }
        }
    }

    LOG_MSG("Final sleep time setting is %d second(s).\n", (int) sleepForSeconds);

    // Set the variable passed in
    if (pMinSleepPeriodSeconds != NULL) {
        *pMinSleepPeriodSeconds = minSleepPeriodSeconds;
    }
    
    return sleepForSeconds;
}

/// Calculate the number of seconds in the day to the start of the working day.
static uint32_t secondsInDayToWorkingDayStart(uint32_t secondsToday)
{
    uint32_t seconds = 0;
    struct tm *pT;
    struct tm tNow;
    struct tm tStart;
    struct tm tEnd;
    time_t t;
   
    // We're awake outside the working day, calculate the new wake-up time
    if (secondsToday < START_OF_WORKING_DAY_SECONDS) {
        seconds = START_OF_WORKING_DAY_SECONDS - secondsToday;
    } else {
        if (secondsToday > START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS) {
            // After the end of the day, so wake up next tomorrow morning
            seconds = START_OF_WORKING_DAY_SECONDS + (3600 * 24) - secondsToday;
        }
    }
    
    if (seconds > 0) {
        t = time(NULL);
        pT = gmtime(&t);
        memcpy(&tNow, pT, sizeof (tNow));
        t = START_OF_WORKING_DAY_SECONDS;
        pT = gmtime(&t);
        memcpy(&tStart, pT, sizeof (tStart));
        t = START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS;
        pT = gmtime(&t);
        memcpy(&tEnd, pT, sizeof (tEnd));
        t = time(NULL) + seconds;
        LOG_MSG("Time now %02d:%02d:%02d UTC, working day is %02d:%02d:%02d to %02d:%02d:%02d, going back to sleep for %d second(s) in order to wake up at UTC %s.\n",
                tNow.tm_hour, tNow.tm_min, tNow.tm_sec,
                tStart.tm_hour, tStart.tm_min, tStart.tm_sec,
                tEnd.tm_hour, tEnd.tm_min, tEnd.tm_sec,
                (int) seconds, timeString(t));
    }
    // If we will still be in slow mode when we awake, no need to wake up until the first slow-operation
    // wake-up time
    if (time(NULL) + seconds < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
        seconds += SLOW_MODE_INTERVAL_SECONDS;
        LOG_MSG("Adding %d second(s) to wake-up time as we're in slow operation mode, so will actually wake up at UTC %s.\n",
                SLOW_MODE_INTERVAL_SECONDS, timeString(t));
    }
    
    return seconds;
}

/// Work out the unix time for midnight on a given day.  In other
// words, truncate a unixTime to a day (so 08:00 on 24th June becomes
// 00:00 on 24th June).
static time_t truncateToDay(time_t unixTime)
{
    return (unixTime / (3600 * 24)) * 3600 * 24;
}

/// Do a limits check of the sleep time.
static time_t sleepLimitsCheck(time_t sleepForSeconds)
{
    
    if (sleepForSeconds < 0) {
        LOG_MSG("WARNING: sleep period went negative (%d second(s)), setting it to zero.\n", (int) sleepForSeconds);
        sleepForSeconds = 0;
    } else if (sleepForSeconds > MAX_WAKEUP_PERIOD_SECONDS) {
        LOG_MSG("WARNING: sleep period (%d second(s)) greater than the max (%d second(s)), setting it to the max.\n", (int) sleepForSeconds, MAX_WAKEUP_PERIOD_SECONDS);
        sleepForSeconds = MAX_WAKEUP_PERIOD_SECONDS;
    }
    
    return sleepForSeconds;
}

/// check if we are still in motion by checking how long it has been since the last accelerometer nudge
static bool stillWithinMotionDelaySinceLastAcceleration(time_t lastAccelerometerNudge, time_t timeNow)
{
    return (timeNow <= (lastAccelerometerNudge + IN_MOTION_TIME_SECONDS));
}

/****************************************************************
 * STATIC FUNCTIONS: REPORTS
 ***************************************************************/

/// Get a new record from the list.
static char *getRecord(RecordType_t type)
{
    char *pContents;

    LOG_MSG("Using record %d.\n", (int) r.nextFreeRecord);
    
    ASSERT (r.nextFreeRecord < (sizeof (r.records) / sizeof (r.records[0])), FATAL_RECORDS_OVERRUN_1);
    
    if (r.records[r.nextFreeRecord].isUsed) {
        LOG_MSG("WARNING: records queue has wrapped, over-writing old data.\n");
    } else {
        r.numRecordsQueued++;
    }
    
    ASSERT (r.numRecordsQueued < (sizeof (r.records) / sizeof (r.records[0])), FATAL_RECORDS_OVERRUN_2);
    
    r.records[r.nextFreeRecord].isUsed = true;
    r.records[r.nextFreeRecord].type = type;
    pContents = r.records[r.nextFreeRecord].contents;
    
    // Move the index on to the next record
    r.nextFreeRecord = incModRecords(r.nextFreeRecord);
    LOG_MSG("Incremented nextFreeRecord to %d.\n", (int) r.nextFreeRecord);
    
    return pContents;
}

/// Free a record.
static void freeRecord(Record_t *pRecord)
{
    if (pRecord != NULL) {
        pRecord->isUsed = false;
        if (r.numRecordsQueued > 0) {
            r.numRecordsQueued--;
        }
    }
}

/// Increment a number and return the incremented
// number modulo the number of records available.
static uint32_t incModRecords (uint32_t x)
{
    x++;
    if (x >= (sizeof (r.records) / sizeof (r.records[0]))) {
        x = 0;
    }
    
    return x;
}

/// Queue a telemetry report.
static void queueTelemetryReport()
{
    char *pRecord;
    uint32_t contentsIndex = 0;
    int32_t batteryPercent = 0;
    time_t t = time(NULL);
    
   LOG_MSG("Queuing a telemetry report.\n");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_TELEMETRY);

    // Add the device ID
    contentsIndex += snprintf(pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, "%.*s", IMEI_LENGTH, r.imei);  // -1 for terminator

    // Add battery status
    batteryGauge.getRemainingPercentage(&batteryPercent);
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf(pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%d", (int) batteryPercent);  // -1 for terminator
        LOG_MSG("Battery level is %d.\n", (int) batteryPercent);
    } else {
        LOG_MSG("WARNING: couldn't fit battery level into report.\n");
    }
    
    // Add signal strength
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        int32_t signal = cellular.rssi();
        contentsIndex += snprintf(pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%d", (int) signal); // -1 for terminator
        LOG_MSG("RSSI is %d.\n", (int) signal);
    } else {
        LOG_MSG("WARNING: couldn't fit Signal Strength reading into report.\n");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf(pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%u", (unsigned int) time(NULL));  // -1 for terminator
        LOG_MSG("Time now is UTC %s.\n", timeString(t));
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    // Add software version
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%u", SW_VERSION);  // -1 for terminator
        LOG_MSG("SW version is %u.\n", SW_VERSION);
    } else {
        LOG_MSG("WARNING: couldn't fit SW version into report.\n");
    }
    
    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", (int) (contentsIndex + 1), (int) (LEN_RECORD_CONTENTS - (contentsIndex + 1))); // +1 to account for terminator
    setLogFlag(LOG_FLAG_TELEMETRY_RECORD_QUEUED);
}

/// Queue a GNSS report.
static void queueGnssReport(float latitude, float longitude, bool motion, float hdop)
{
    char *pRecord;
    uint32_t contentsIndex = 0;
    time_t t = time(NULL);

    LOG_MSG("Queuing a GNSS report.\n");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_GNSS);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, "%.*s", IMEI_LENGTH, r.imei);  // -1 for terminator

    // Add GNSS
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%.6f;%.6f", latitude, longitude);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit GNSS reading into report.\n");
    }
        
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%u", (unsigned int) time(NULL));  // -1 for terminator
        LOG_MSG("Time now is UTC %s.\n", timeString(t));
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    // Add motion to the end, to preserve backwards-compatibility
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%d", motion);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit motion indication into report.\n");
    }
        
    // Add HDOP to the end, to preserve backwards-compatibility
    if (hdop != GNSS_INVALID_HDOP) {
        if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
            contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%.2f", hdop);  // -1 for terminator
        } else {
            LOG_MSG("WARNING: couldn't fit PDOP into report.\n");
        }
    }

    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", (int) (contentsIndex + 1), (int) (LEN_RECORD_CONTENTS - (contentsIndex + 1))); // +1 to account for terminator
    setLogFlag(LOG_FLAG_GNSS_RECORD_QUEUED);
}

/// Queue a stats report.
static void queueStatsReport()
{
    char *pRecord;
    uint32_t contentsIndex = 0;
    uint32_t upTimeSeconds = upTimer.read_ms() / 1000 + r.totalPowerSaveSeconds;
    time_t t = time(NULL);
    
    if (upTimeSeconds == 0) {
        upTimeSeconds = 1; // avoid div by 0
    }

    LOG_MSG("Queuing a stats report.\n");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_STATS);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, "%.*s", IMEI_LENGTH, r.imei);  // -1 for terminator

    // Add fatal count and types
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";F%ld", rr.numFatals); // -1 for terminator
        for (uint8_t x = 0; (x < rr.numFatals) && (x < (sizeof (rr.fatalList) / sizeof (rr.fatalList[0]))); x++) {
            contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ".%02d", rr.fatalList[x]); // -1 for terminator
        }
    }
    if (contentsIndex >= LEN_RECORD_CONTENTS) {
        LOG_MSG("WARNING: couldn't fit fatal count and types into report.\n");
    }
    
    // Add up-time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%ld.%ld:%02ld:%02ld",
                                   upTimeSeconds / 86400, (upTimeSeconds / 3600) % 24, (upTimeSeconds / 60) % 60,  upTimeSeconds % 60);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit up-time into report.\n");
    }
    
    // Add %age power save time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%ld%%", r.totalPowerSaveSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit percentage power saving time into report.\n");
    }
    
    // Add %age GNSS on time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";~%ld%%", r.totalGnssSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit percentage GNSS on time into report.\n");
    }
    
    // Add loop counts and position percentage
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";L%ldM%ldG%ldP%ld%%",
                                   r.numLoops, r.numLoopsMotionDetected, r.numLoopsGnssOn, (r.numLoopsLocationNeeded != 0) ? (r.numLoopsLocationValid * 100 / r.numLoopsLocationNeeded) : 0); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit loop counts into report.\n");
    }
    
    // Add GNSS background data
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";N%ldCP%ldCA%ld",
                                   gnssNumSatellitesUsable, gnssPeakCNUsed, gnssAverageCNUsed); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit GNSS background data into report.\n");
    }

    // Add connect counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";C%ld-%ld", r.numConnectAttempts, r.numConnectFailed); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit connect counts into report.\n");
    }
    
    // Add publish (== send) counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";S%ld-%ld", r.numPublishAttempts, r.numPublishFailed); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit publish counts into report.\n");
    }

    // Add last accelerometer reading
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";X%dY%dZ%d",
                                   r.accelerometerReading.x, r.accelerometerReading.y, r.accelerometerReading.z); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit last accelerometer reading into report.\n");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD_CONTENTS)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD_CONTENTS - contentsIndex - 1, ";%u", (unsigned int) time(NULL));  // -1 for terminator
        LOG_MSG("Time now is UTC %s.\n", timeString(t));
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", (int) (contentsIndex + 1), (int) (LEN_RECORD_CONTENTS - (contentsIndex + 1))); // +1 to account for terminator
    setLogFlag(LOG_FLAG_STATS_RECORD_QUEUED);
}

/// Send the queued reports, returning true if all have been sent.
static bool sendQueuedReports(bool *pAtLeastOneGnssReportSent)
{
    bool atLeastOneGnssReportSent = false;
    uint32_t sentCount = 0;
    uint32_t failedCount = 0;
    uint32_t x;

    // Publish any records that are marked as used
    x = r.nextPubRecord;
    LOG_MSG("Sending report(s) (numRecordsQueued %u, nextPubRecord %u, nextFreeRecord %u).\n", (unsigned int) r.numRecordsQueued, (unsigned int) x, (unsigned int) r.nextFreeRecord);

    while (x != r.nextFreeRecord) {
        ASSERT (x < (sizeof (r.records) / sizeof (r.records[0])), FATAL_RECORDS_OVERRUN_3);
        LOG_MSG("Report %d: ", (unsigned int) x);
        if (r.records[x].isUsed) {
            // This is something we want to publish, so first connect
            if (connect()) {
                r.numPublishAttempts++;
#ifdef DISABLE_CELLULAR_CONNECTION
                if (true) {
#else
                if (send(r.records[x].type, r.records[x].contents)) {
#endif
                    LOG_MSG("sent %s.\n", r.records[x].contents);
                    // Keep track if we've sent a GNSS report as, if we're in pre-operation mode,
                    // we can then go to sleep for longer.
                    if (r.records[x].type == RECORD_TYPE_GNSS) {
                        atLeastOneGnssReportSent = true;
                    }
                    freeRecord(&r.records[x]);
                    sentCount++;
                    setLogFlag(LOG_FLAG_PUBLISH_SUCCEEDED);
                } else {
                    setLogFlag(LOG_FLAG_PUBLISH_FAILED);
                    r.numPublishFailed++;
                    failedCount++;
                    LOG_MSG("WARNING: send failed.\n");
                }
            } else {
                failedCount++;
                LOG_MSG("WARNING: send failed due to not being connected.\n");
            }

            // If there has not yet been a publish failure, increment the place to start next time
            if (failedCount == 0) {
                r.nextPubRecord = incModRecords(r.nextPubRecord);
                LOG_MSG("Incremented nextPubRecord to %d.\n", (unsigned int) r.nextPubRecord);
            }
        } else {
            LOG_MSG("unused.\n");
            if (failedCount == 0) {
                // This must be a record which we attemped to send previously but
                // we did not increment the publish record index as there had been
                // a failure; if we've had no transmit failures we can increment
                // the publish record index now so as not to fall behind.
                r.nextPubRecord = incModRecords(r.nextPubRecord);
                LOG_MSG("Incremented nextPubRecord to %d.\n", (unsigned int) r.nextPubRecord);
            }
        }

        // Increment x
        x = incModRecords(x);
        
        setLogFlag(LOG_FLAG_SEND_ATTEMPTED);
    }

    LOG_MSG("%u report(s) sent, %u failed to send.\n", (unsigned int) sentCount, (unsigned int) failedCount);
    // If there's been a publish failure and nextPubRecord is
    // equal to nextFreeRecord then we must have wrapped.  In this
    // case, increment nextPubRecord to keep things in time order.
    if ((failedCount > 0) && (r.nextPubRecord == r.nextFreeRecord)) {
        r.nextPubRecord = incModRecords(r.nextPubRecord);
    }
    
    // Set the in/out parameter
    if (pAtLeastOneGnssReportSent != NULL) {
        *pAtLeastOneGnssReportSent = atLeastOneGnssReportSent;
    }

    setLogFlag(LOG_FLAG_TELEMETRY_RECORD_QUEUED);
    return failedCount == 0;
}

/****************************************************************
 * STATIC FUNCTIONS: SETUP() AND LOOP() FROM THE PARTICLE CODE
 ***************************************************************/

static void setup()
{
    bool gnssConfigured = false;
    
    LOG_MSG("\n-> Starting up.\n");
    debugPrintRetained();
    
    // Set up retained and "really retained" memory, if required
    if ((strcmp (r.key, RETAINED_INITIALISED) != 0) || (r.swVersion != SW_VERSION)) {
        resetRetained();
    }
    if ((strcmp (rr.key, RETAINED_INITIALISED) != 0) || (rr.swVersion != SW_VERSION)) {
        resetReallyRetained();
    }
    
    // Add a log flag for starting up
    addLogFlagsEntry();
    setLogFlag(LOG_FLAG_STARTUP_NOT_LOOP_ENTRY);
    
    if (r.warmStart) {
        LOG_MSG("\n-> Start-up after deep sleep.\n");
        setLogFlag(LOG_FLAG_STARTUP_WARM_NOT_COLD);
    } else {
        LOG_MSG("\n-> Start-up from power-off.\n");
        clearLogFlag(LOG_FLAG_STARTUP_WARM_NOT_COLD);
    }
    
    // Starting again
    r.numStarts++;
    
    // Set the debug LED off
    debugInd(DEBUG_IND_OFF);

    // Get the IMEI of the modem module and, while
    // we're doing that, set the right time
    if (r.imei[0] < '0') {
        LOG_MSG("Powering up cellular to get IMEI...\n");
        cellular.init();
        wait_ms(MODEM_POWER_ON_DELAY_MILLISECONDS);
        LOG_MSG("Getting IMEI...\n");
        strcpy(r.imei, cellular.imei());
        LOG_MSG("IMEI is: %.*s.\n", IMEI_LENGTH, r.imei);
    }

    // Start GNSS
    // NOTE: need to do this before establishing time as we access the
    // GNSS module during that process
    if (!gnss.init()) {
        LOG_MSG("WARNING: unable to power up GNSS but continuing anyway.\n");
    }

    // Establish time
    establishTime();
    
#ifdef DISABLE_ACCELEROMETER
    accelerometerConnected = false;
#else
    // Setup the accelerometer interface
    accelerometerConnected = accelerometer.init(&i2C);
    if (!r.warmStart) {
        // Configure the accelerometer as we've not done it before
        LOG_MSG("Configuring accelerometer...\n");
        if (!accelerometer.setActivityThreshold(ACCELEROMETER_ACTIVITY_THRESHOLD)) {
            LOG_MSG("WARNING: couldn't configure accelerometer but continuing anyway.\n");
        }
    } else {
        LOG_MSG("Skipping accelerometer configuration as this is a warm start.\n");
    }
#endif

    if (!r.warmStart) {
        // Do the very initial configuration of GNSS and battery
        // gauge and then switch them off again.  This puts settings
        // into battery backed RAM that can be restored later.
        gnssConfigured = configureGnss();
        if (!gnssConfigured) {
            LOG_MSG("WARNING: couldn't configure GNSS but continuing anyway.\n");
        }
    } else {
        LOG_MSG("Skipping GNSS configuration as this is a warm start.\n");
    }
    
    // Initialise the battery gauge (don't care if this fails, carry on anyway)
    if (batteryGauge.init(&i2COnBoard)) {
        if (!r.warmStart) {
            if (!batteryGauge.enableGauge()) {
                LOG_MSG("WARNING: couldn't enable battery gauging but continuing anyway.\n");
            }
        }
    } else {
        LOG_MSG("WARNING: couldn't initialise battery gauge but continuing anyway.\n");
    }
    
    // If GNSS was off make sure it stays off
    if (!r.gnssOn) {
        gnssOff();
    } else {
        LOG_MSG("Leaving GNSS on as it was on before we started.\n");
    }

    // If this is a cold start then flash the LED to say that all is good
    if (!r.warmStart) {
#ifdef DISABLE_ACCELEROMETER
        if (gnssConfigured) {
#else
        if (accelerometerConnected && gnssConfigured) {
#endif
            debugInd(DEBUG_IND_BOOT_COMPLETE);
        }
    }
    
    // Setup is now complete
    if (!r.warmStart) {
        r.lastColdStartSeconds = time(NULL);
        // All future starts are warm starts
        r.warmStart = true;
    }
    LOG_MSG("Start-up completed.\n");
}

static void loop()
{
    bool forceSend = false;
    bool wakeOnAccelerometer = false;
    bool atLeastOneGnssReportSent = false;
    bool inMotion = false;
    bool accelerationDetected = false;
    bool stillInMotion = false;
    bool fixAchieved = false;
    float latitude = GNSS_INVALID_ANGLE;
    float longitude = GNSS_INVALID_ANGLE;
    float elevation = 0;
    float hdop = GNSS_INVALID_HDOP;
    time_t t;
    time_t tNow;

    // Add a new log flag entry for this loop
    addLogFlagsEntry();
    clearLogFlag(LOG_FLAG_STARTUP_NOT_LOOP_ENTRY);
    
    // Set the GNSS log flag (as otherwise it won't be
    // set unless GNSS is toggled on this loop)
    if (gnssIsOn()) {
        setLogFlag(LOG_FLAG_GNSS_ON_NOT_OFF);
    }
    
    // This only for info
    r.numLoops++;
    tNow = time(NULL);
    LOG_MSG("\n-> Starting loop %d at %d (UTC %s) having power saved since %d second(s) ago (UTC %s).\n", (unsigned int) r.numLoops,
            (int) tNow, timeString(tNow), (int) (tNow - r.powerSaveTime), timeString(r.powerSaveTime));

    // If we are in slow operation override the stats timer period
    // so that it is more frequent; it is useful to see these stats in
    // this "slow operation" case to check that the device is doing well.
    // Also make sure that we request a GNSS fix, irrespective of whether
    // we're moving or not, as that is also useful data.
    if (time(NULL) < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
        statsPeriodSeconds = MIN_MOTION_PERIOD_SECONDS;
        r.gnssFixRequested = true;
    } else {
        setLogFlag(LOG_FLAG_START_TIME_FULL_WORKING_DAY_OPERATION_PASSED);
    }

    // Having valid time is fundamental to this program and so, if time
    // has not been established try to establish it again
    if (establishTime()) {

        // Add up the time we were in power save mode (for info)
        if (r.powerSaveTime != 0) {
            r.totalPowerSaveSeconds += time(NULL) - r.powerSaveTime;
            r.powerSaveTime = 0;
        }

        // If sleepStartSeconds was recorded, for some reason, before we
        // established time then it will be out of wack.  In this case,
        // give it a sensible time
        if (r.sleepStartSeconds < MIN_TIME_UNIX_UTC) {
            r.sleepStartSeconds = time(NULL);
        }

        // Check if we should be awake at all
        tNow = time(NULL);
        if (time(NULL) >= START_TIME_UNIX_UTC) {
            struct tm *pT = gmtime(&tNow);
            uint32_t secondsSinceMidnight = pT->tm_hour * 3600 + pT->tm_min * 60 + pT->tm_sec;

            setLogFlag(LOG_FLAG_START_TIME_PASSED);
            
            // Check if we are inside the working day
            if ((secondsSinceMidnight >= START_OF_WORKING_DAY_SECONDS) &&
                (secondsSinceMidnight <= START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS)) {
                    
                LOG_MSG("It is during the working day.\n");
                setLogFlag(LOG_FLAG_IN_WORKING_DAY);

                // Respond to interrupts
                wakeOnAccelerometer = true;

                // See if we've moved
                accelerationDetected = handleInterrupt();
                stillInMotion = stillWithinMotionDelaySinceLastAcceleration(r.lastAccelerometerNudge, time(NULL));
                inMotion = (accelerationDetected || stillInMotion);
                if (inMotion) {
                  r.gnssFixRequested = true;
                  r.lastMotionSeconds = time(NULL);
                  if (accelerationDetected) {
                    r.lastAccelerometerNudge = r.lastMotionSeconds;
                    LOG_MSG("*** Acceleration was detected.\n");
                  } else {
                    LOG_MSG("*** Motion was detected based on time.\n");
                  }
                  r.numLoopsMotionDetected++;
                  setLogFlag(LOG_FLAG_MOTION_DETECTED);
                }

                // First of all, check if this is a wake-up from a previous sleep
                // where we set a minimum sleep period
                if (time(NULL) >= r.sleepStartSeconds + r.minSleepPeriodSeconds) {
                    // If we're outside the minimum sleep period, reset all these
                    // variables to defaults for this wake-up
                    r.sleepForSeconds = MIN_MOTION_PERIOD_SECONDS;
                    r.minSleepPeriodSeconds = MIN_MOTION_PERIOD_SECONDS;
                    r.modemStaysAwake = false;

                    // Queue a telemetry report, if required
                    if (time(NULL) - r.lastTelemetrySeconds >= TELEMETRY_PERIOD_SECONDS) {
                        r.lastTelemetrySeconds = time(NULL);
                        // Switch the modem on for this so that we can get the RSSI and,
                        // in any case, we will be forcing a send
                        cellular.init(PIN);
                        cellular.set_credentials(APN, USERNAME, PASSWORD);
                        wait_ms(MODEM_POWER_ON_DELAY_MILLISECONDS);
                        queueTelemetryReport();
                        LOG_MSG("Forcing a send.\n");
                        forceSend = true;
                    }
    
                    // Queue a stats report, if required
                    if (time(NULL) - r.lastStatsSeconds >= statsPeriodSeconds) {
                        r.lastStatsSeconds = time(NULL);
                        queueStatsReport();
                    }
    
                    // Queue a GNSS report if we're in motion or can't tell if we're in motion
                    if (r.gnssFixRequested || !accelerometerConnected) {
                        r.numLoopsLocationNeeded++;
                        
                        if (!accelerometerConnected) {
                            LOG_MSG("No accelerometer, getting GNSS reading every time.\n");
                        }
                        if (gnssIsOn()) {
                            LOG_MSG("Still trying to get a GNSS fix from last time.\n");
                        }
    
                        r.numLoopsGnssOn++;
                        // Get the latest output from GNSS
                        fixAchieved = gnssUpdate(&latitude, &longitude, &elevation, &hdop);
                        if (fixAchieved) {
                            r.numLoopsGnssFix++;
                            r.numLoopsLocationValid++;
                            r.lastGnssSeconds = time(NULL);
                            queueGnssReport(latitude, longitude, inMotion, hdop);
                        }
                    }
    
                    // If GNSS meets the power-save criteria, switch it off
                    if (!inMotion && gnssIsOn() && gnssCanPowerSave()) {
                        gnssOff();
                    }
    
#ifdef ENABLE_SEND_STATS_AT_GNSS_ATTEMPT
                    // If a fix was requested, and now that we have the GNSS stats (which were
                    // populated in the gnssCanPowerSave() call), queue a stats report with
                    // the data in.
                    if (r.gnssFixRequested) {
                        queueStatsReport();
                    }
#endif
                    // Check if it's time to publish the queued reports
                    if (forceSend || ((time(NULL) - r.lastReportSeconds >= REPORT_PERIOD_SECONDS) || (r.numRecordsQueued >= QUEUE_SEND_LEN))) {
                        if (forceSend) {
                            LOG_MSG("\"Force Send\" was set.\n");
                        }
                        
                        if (r.numRecordsQueued >= QUEUE_SEND_LEN) {
                            r.modemStaysAwake = true;
                            LOG_MSG("Keeping modem awake while sleeping as we had a lot of records queued this time.\n");
                        }
                        
                        sendQueuedReports(&atLeastOneGnssReportSent);
                        r.lastReportSeconds = time(NULL); // Do this at the end as transmission could, potentially, take some time
                    }
    
                    // If we're in slow-mode, haven't sent a GNSS report yet and are still within
                    // the time-window then stay awake, otherwise shut the modem and GNSS down
                    if (time(NULL) < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
                       if (!(atLeastOneGnssReportSent && fixAchieved) && (time(NULL) - r.lastColdStartSeconds <= SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS)) {
                            r.modemStaysAwake = true;
                            LOG_MSG("Keeping modem awake while sleeping as we're in the short \"slow mode\" wake-up.\n");
                        } else {
                            // Clear the retained variables as we'd like a fresh start when we awake
                            resetRetained();
                            LOG_MSG("Switching modem and GNSS off at the end of this \"slow mode\" wake-up.\n");
                            gnssOff();
                            cellular.deinit();
                            r.modemStaysAwake = false;
                            wakeOnAccelerometer = false;
                        }
                    }

                    // Sort out how long we can sleep for
                    r.sleepForSeconds = setTimings(secondsSinceMidnight, atLeastOneGnssReportSent, fixAchieved, &r.minSleepPeriodSeconds);
                    
                } else { // END if() we've woken up early
                    // If we have awoken early, put us back to sleep again for the remaining sleep time
                    r.sleepForSeconds = r.minSleepPeriodSeconds - (time(NULL) - r.sleepStartSeconds);
                    LOG_MSG("Interrupt woke us up early, going back to bed again.\n");
                    setLogFlag(LOG_FLAG_WOKEN_UP_EARLY);
                } // END else condition of if() we've woken up early

                // Make sure we don't set a wacky sleep period during the working day
                r.sleepForSeconds = sleepLimitsCheck(r.sleepForSeconds);
                // Make sure we don't set a wacky minimum sleep period
                r.minSleepPeriodSeconds = r.sleepForSeconds;
                if (r.minSleepPeriodSeconds > MAX_MOTION_PERIOD_SECONDS) {
                    r.minSleepPeriodSeconds = r.minSleepPeriodSeconds;
                }
                
            } else { // END if() we're inside the working day
                LOG_MSG("Awake outside the working day.\n");
                // Clear the retained variables as we'd like a fresh start when we awake
                resetRetained();
                // We're awake outside of the working day, calculate the time to the start of the working day
                r.sleepForSeconds = secondsInDayToWorkingDayStart(secondsSinceMidnight);
               // Make sure GNSS and the modem are off
                gnssOff();
                cellular.deinit();
                r.modemStaysAwake = false;
            } // END else condition of if() we're inside the working day
        } else { // END if() time >= START_TIME_UNIX_UTC
            t = START_TIME_UNIX_UTC;
            tNow = time(NULL);
            LOG_MSG("Awake before start time:\n time now UTC %s, start time UTC %s.\n", 
                    timeString(tNow), timeString(t));
            // Clear the retained and "really retained" variables as we'd like a fresh start when we awake
            resetRetained();
            resetReallyRetained();

            // We're awake when we shouldn't have started operation at all yet, calculate new wake-up time
            r.sleepForSeconds = START_TIME_UNIX_UTC - time(NULL);
            // Make sure GNSS and the modem are off
            gnssOff();
            cellular.deinit();
            r.modemStaysAwake = false;

            // If we will still be in slow mode when we awake, no need to wake up until the first interval of the working day expires
            if (time(NULL) + r.sleepForSeconds < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
                r.sleepForSeconds = truncateToDay(START_TIME_UNIX_UTC) - time(NULL) + START_OF_WORKING_DAY_SECONDS + SLOW_MODE_INTERVAL_SECONDS;
            }
        } // END else condition of if() time >= START_TIME_UNIX_UTC

        // Record the time we went into power saving state (only if we have established time though)
        r.powerSaveTime = time(NULL);
    } else { // END if() time has been established
        r.sleepForSeconds = TIME_SYNC_RETRY_PERIOD_SECONDS;
        // Make sure GNSS is off
        gnssOff();
        // Keep the modem awake in this case as we will want to establish network time
        r.modemStaysAwake = true;
        setLogFlag(LOG_FLAG_TIME_NOT_ESTABLISHED);
    } // END else condition of if() time has been established

    // Catch odd sleep times
    if (r.sleepForSeconds < 0) {
        LOG_MSG("### WARNING: sleep period went negative (%d second(s)), setting it to zero.\n", (int) r.sleepForSeconds);
        r.sleepForSeconds = 0;
    } else if (r.sleepForSeconds > MAX_WAKEUP_PERIOD_SECONDS) {
        // This will legitimately happen if we're not yet in full working day mode or we are outside the working day,
        // hence it is for information only
        LOG_MSG("### INFO: sleep period is greater than the maximum during the working day (%d second(s)).\n", (int) r.sleepForSeconds);
    }
    if (r.minSleepPeriodSeconds < 0) {
        LOG_MSG("### WARNING: minimum sleep period went negative (%d second(s)), setting it to zero.\n", (int) r.minSleepPeriodSeconds);
        r.minSleepPeriodSeconds = 0;
    } else if (r.minSleepPeriodSeconds > MAX_MOTION_PERIOD_SECONDS) {
        LOG_MSG("### WARNING: minimum sleep period (%d second(s)) greater than expected, setting it to the maximum (%d).\n", (int) r.minSleepPeriodSeconds, MAX_MOTION_PERIOD_SECONDS);
        r.minSleepPeriodSeconds = MAX_MOTION_PERIOD_SECONDS;
    }

    // Print a load of informational stuff
    t = time(NULL) + r.sleepForSeconds;
    LOG_MSG("-> Ending loop %u: now sleeping for up to %d second(s) with a minimum of %d second(s), will awake at UTC %s.\n",
             (unsigned int) r.numLoops, (int) r.sleepForSeconds, (int) r.minSleepPeriodSeconds, timeString(t));
    LOG_MSG("-> The modem will ");
    setLogFlag(LOG_FLAG_FREE_1);
    if (r.modemStaysAwake) {
        LOG_MSG("be unaffected by sleep");
    } else {
        LOG_MSG("be OFF");
    }
    if (cellular.is_connected()) {
        setLogFlag(LOG_FLAG_FREE_2);
        LOG_MSG(" (it is currently CONNECTED)");
    }
    setLogFlag(LOG_FLAG_FREE_3);
    LOG_MSG(", GNSS will be ");
    if (gnssIsOn()) {
        LOG_MSG("ON");
    } else {
        LOG_MSG("OFF");
    }
    LOG_MSG(", we ");
    if (wakeOnAccelerometer) {
        LOG_MSG("WILL");
    } else {
        LOG_MSG("will NOT");
    }
    LOG_MSG(" wake-up on movement.\n");

    // Make sure the debug LED is off to save power
    debugInd(DEBUG_IND_OFF);
    
#ifdef USB_DEBUG
    // Leave a little time for serial prints to leave the building before sleepy-byes
    wait_ms(1000);
#endif
    
    // Now go to sleep for the allotted time.
    r.sleepStartSeconds = time(NULL);
    if (r.sleepForSeconds > 0) {
        // If the accelerometer interrupt goes off it will wake us up and then be
        // reset when we eventually awake.
        if (accelerometerConnected) {
            if (wakeOnAccelerometer) {
                accelerometer.enableInterrupts();
                setLogFlag(LOG_FLAG_WAKE_ON_INTERRUPT);
           } else {
                accelerometer.disableInterrupts();
                clearLogFlag(LOG_FLAG_WAKE_ON_INTERRUPT);
            }
        }
        
        // The sleep calls here will wake up when the accelerometer raises it's interrupt line.
        if (r.modemStaysAwake || (r.sleepForSeconds < MINIMUM_DEEP_SLEEP_PERIOD_SECONDS)) {
            // Sleep with the network connection up so that we can send reports
            // without having to re-register
            clearLogFlag(LOG_FLAG_DEEP_SLEEP_NOT_CLOCK_STOP);
            upTimer.stop();
            lowPower.enterStandby(r.sleepForSeconds * 1000);
        } else {
            // Otherwise we can go to deep sleep and will re-register when we awake
            // NOTE: we will come back from reset after this, only the
            // retained variables will be kept
            cellular.disconnect();
            cellular.deinit();
            if (r.sleepForSeconds > MINIMUM_DEEP_SLEEP_PERIOD_SECONDS) {
                setLogFlag(LOG_FLAG_DEEP_SLEEP_NOT_CLOCK_STOP);
                upTimer.stop();
                lowPower.enterStop(r.sleepForSeconds * 1000);
            } else {
                clearLogFlag(LOG_FLAG_DEEP_SLEEP_NOT_CLOCK_STOP);
                upTimer.stop();
                lowPower.enterStandby(r.sleepForSeconds * 1000);
            }
        }
    }
}


/****************************************************************
 * PUBLIC FUNCTIONS: MAIN
 ***************************************************************/

int main()
{
    // First exit Debug mode on the chip, otherwise it will not be
    // able to enter Standby mode
    lowPower.exitDebugMode();
    
    // Make sure the external wakeup pin is enabled
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    
    // Do all the setup stuff
    setup();
    
    // Run the loop forever
    upTimer.start();
    while (1) {
        loop();
    }
}

// End of file