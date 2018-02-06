Introduction
============

This is an Mbed port of the code from the https://github.com/u-blox/tracker which ran on a Particle Electron platform.  This code is targeted at the u-blox C030 board, which uses an STM32F4 processor, a member of the same processor family as that of the Particle board (so it supports all of the power saving capabilities of the STM32F4 and 4 kbytes of back-up SRAM).  Since this is a port from Particle it still adopts the `setup()`/`loop()` pattern.

Hardware Configuration
======================
For this code to work correctly the `INT1` output from the ADXL345 accelerometer must be connected to the `WKUP` pin of the STM32F4  CPU on the C030 board.  The `WKUP` pin is not brought out on the current C030 boards, instead being attached to a pull-up resistor, so some HW surgery is required.

Configuration
=============
In order to use power most efficiently, sleep is used in combination with timed operation in the following way:

* If, after establishing network time, the time is found to be less than `START_TIME_UNIX_UTC` then, the device returns to deep sleep (modem off, processor clocks and RAM off, ~0.1 mA consumed) until `START_TIME_UNIX_UTC` is reached.
* If the time is greater than or equal to `START_TIME_UNIX_UTC` then the device checks if the working day has begun, i.e. is the time greater than `START_OF_WORKING_DAY_SECONDS` and less than `START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS`.
* If the time is within the work day then one of two things happens:
  * If the time is less than `START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC` then the device wakes up only for long enough to get a GNSS reading, "slow operation" (timing out at `SLOW_OPERATION_MAX_TIME_TO_GNSS_FIX_SECONDS`) and repeats this `SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY` times, evenly spread throughout the working day.  Between wake-ups the device returns to deep sleep with the modem off.
  * If the time is greater than or equal to `START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC` then the device remains awake for all of the working day, putting the processor to sleep and keeping the modem registered with the network if wake-ups are frequent enough, so that we can collect and report position for the full working day.
* If the time is not within the working day then the device returns to deep sleep, with modem off, until the start of the next working day.

The default timings are set so that the device wakes-up in slow operation and then goes into full working day operation a few days later. This was for a show where the trackers were installed a few days before the start of the show and we wanted to monitor that they were OK but not consume a lot of battery power. By judicious choice of:

* `START_TIME_UNIX_UTC`
* `START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC`
* `START_OF_WORKING_DAY_SECONDS`
* `LENGTH_OF_WORKING_DAY_SECONDS`
* `SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY`

... you should be able to achieve the tracker behaviour you want.  To always get slow operation, set `START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC` to a time far in the future.  To always get full monitoring throughout the working day, set `START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC` to `START_TIME_UNIX_UTC`.  To set the working day to be 24 hours, set `START_OF_WORKING_DAY_SECONDS` to 0 and `LENGTH_OF_WORKING_DAY_SECONDS` to 3600.

Messages
========
The chosen message formats are highly compressed to save data and are of the following form:

`gnss:353816058851462;1465283731;51.283645;-0.776569;1;5.15`

...where the first item is the 15-digit IMEI of the device, the second one the Unix timestamp, the third one the latitude in degrees as a float, the fourth one the longitude in degrees as a float the fifth a 1 if the reading was triggered due to the accelerometer indicating motion (0 otherwise) and finally the horizontal dilution of position (i.e. accuracy) as a float.  This message is queued at every wakeup, if a fix is achieved.

`telemetry:353816058851462;1465283731;80.65%;-70;1`

...where the first item is the 15-digit IMEI, the second one the Unix timestamp, the third the battery left as a percentage, the fourth the signal strength in dBm the fifth the SW version. This is sent periodically when the device wakes up from deep sleep and every `TELEMETRY_PERIOD_SECONDS` seconds thereafter.

NOTE: in addition to the above there is a `stats` message with a similar format. See `queueStatsReport()` for more details.

NOTE: if you want to use a different message format, maybe using a more standard (though less efficient) JSON approach, simply modify `queueTelemeteryReport()`, `queueGnssReport()` (and, if required, `queueStatsReport()`) to match.