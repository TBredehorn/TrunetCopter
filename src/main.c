/*
Copyright (C) 2012 Wagner Sartori Junior <wsartori@gmail.com>
http://www.wsartori.com

This file is part of TrunetCopter project.

TrunetCopter program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* ChibiOS includes */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/* ARM includes */
#include "math.h"

/*
 ******************************************************************************
 * TrunetCopter INCLUDES
 ******************************************************************************
 */
#include "main.h"
#include "i2c_local.h"

#include "eeprom/eeprom.h"

#include "sensors/baro_ms5611.h"
#include "sensors/imu_mpu6050.h"
#include "sensors/magn_hmc5883.h"
#include "sensors/gps_mtk.h"

#include "algebra.h"

uint16_t cnt_debug = 0;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
uint32_t GlobalFlags = 0;

EepromFileStream EepromFile;

baro_data_t baro_data;
imu_data_t imu_data;
gps_data_t gps_data;
float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

EventSource imu_event;

extern float sampleFreq;

/* 
 * Threads
 */
#ifdef DEBUG
static WORKING_AREA(waThreadDebug, 256);
static msg_t ThreadDebug(void *arg) {
	(void)arg;
	chRegSetThreadName("Debug");

	struct EventListener self_el;
	chEvtRegister(&imu_event, &self_el, 5);

	while (TRUE) {
		chEvtWaitOne(EVENT_MASK(4));
#ifndef DEBUG_OUTPUT_QUARTENION_BINARY
		chprintf((BaseChannel *)&SERIAL_DEBUG, "frequency: %f\r\n", sampleFreq);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Temperature: %f\r\n", baro_data.ftempms);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Pressure: %f\r\n", baro_data.fbaroms);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Altitude: %f\r\n", baro_data.faltims);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Accelerometer(x, y, z): %f, ", imu_data.acc_x);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f, ", imu_data.acc_y);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f\r\n", imu_data.acc_z);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Gyroscope(x, y, z): %f, ", imu_data.gyro_x);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f, ", imu_data.gyro_y);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f\r\n", imu_data.gyro_z);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Magnetometer(x, y, z): %f, ", imu_data.mag_x);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f, ", imu_data.mag_y);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f\r\n", imu_data.mag_z);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_DEBUG, "q0, q1, q2, q3: %f, ", q0);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f, ", q1);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f, ", q2);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "%f\r\n", q3);
		if (gps_data.valid == 1) {
			chprintf((BaseChannel *)&SERIAL_DEBUG, "----------------------------------\r\n");
			chprintf((BaseChannel *)&SERIAL_DEBUG, "satelites: %d\r\n", gps_data.satellites);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "latitude: %d\r\n", gps_data.latitude);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "longitude: %d\r\n", gps_data.longitude);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "altitude: %f\r\n", gps_data.altitude);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "speed(m/s): %f\r\n", gps_data.speed);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "heading(degrees): %f\r\n", gps_data.heading);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "UTC date: %d\r\n", gps_data.utc_date);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "UTC time: %d\r\n", gps_data.utc_time);
			chprintf((BaseChannel *)&SERIAL_DEBUG, "milliseconds from epoch: %d\r\n", gps_data.time);
		}
		chprintf((BaseChannel *)&SERIAL_DEBUG, "==================================\r\n");
		chEvtBroadcastFlags(&imu_event, EVENT_MASK(5));
		chThdSleepMilliseconds(500);
#else
		if (cnt_debug == 4) {
			uint8_t i;

			uint8_t * b1 = (uint8_t *) &q0;
			for(i=0; i<4; i++) {
				uint8_t b1q1 = (b1[i] >> 4) & 0x0f;
				uint8_t b2q1 = (b1[i] & 0x0f);

				uint8_t c1q1 = (b1q1 < 10) ? ('0' + b1q1) : 'A' + b1q1 - 10;
				uint8_t c2q1 = (b2q1 < 10) ? ('0' + b2q1) : 'A' + b2q1 - 10;

				sdWrite(&SERIAL_DEBUG, &c1q1, 1);
				sdWrite(&SERIAL_DEBUG, &c2q1, 1);
			}
			chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
			uint8_t * b2 = (uint8_t *) &q1;
			for(i=0; i<4; i++) {
				uint8_t b1q2 = (b2[i] >> 4) & 0x0f;
				uint8_t b2q2 = (b2[i] & 0x0f);

				uint8_t c1q2 = (b1q2 < 10) ? ('0' + b1q2) : 'A' + b1q2 - 10;
				uint8_t c2q2 = (b2q2 < 10) ? ('0' + b2q2) : 'A' + b2q2 - 10;

				sdWrite(&SERIAL_DEBUG, &c1q2, 1);
				sdWrite(&SERIAL_DEBUG, &c2q2, 1);
			}
			chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
			uint8_t * b3 = (uint8_t *) &q2;
			for(i=0; i<4; i++) {
				uint8_t b1q3 = (b3[i] >> 4) & 0x0f;
				uint8_t b2q3 = (b3[i] & 0x0f);

				uint8_t c1q3 = (b1q3 < 10) ? ('0' + b1q3) : 'A' + b1q3 - 10;
				uint8_t c2q3 = (b2q3 < 10) ? ('0' + b2q3) : 'A' + b2q3 - 10;

				sdWrite(&SERIAL_DEBUG, &c1q3, 1);
				sdWrite(&SERIAL_DEBUG, &c2q3, 1);
			}
			chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
			uint8_t * b4 = (uint8_t *) &q3;
			for(i=0; i<4; i++) {
				uint8_t b1q4 = (b4[i] >> 4) & 0x0f;
				uint8_t b2q4 = (b4[i] & 0x0f);

				uint8_t c1q4 = (b1q4 < 10) ? ('0' + b1q4) : 'A' + b1q4 - 10;
				uint8_t c2q4 = (b2q4 < 10) ? ('0' + b2q4) : 'A' + b2q4 - 10;

				sdWrite(&SERIAL_DEBUG, &c1q4, 1);
				sdWrite(&SERIAL_DEBUG, &c2q4, 1);
			}
			chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
			uint8_t * b5 = (uint8_t *) &sampleFreq;
			for(i=0; i<4; i++) {
				uint8_t b1 = (b5[i] >> 4) & 0x0f;
				uint8_t b2 = (b5[i] & 0x0f);

				uint8_t c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
				uint8_t c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

				sdWrite(&SERIAL_DEBUG, &c1, 1);
				sdWrite(&SERIAL_DEBUG, &c2, 1);
			}
			chprintf((BaseChannel *)&SERIAL_DEBUG, ",\r\n");

			cnt_debug = 0;
		}
		chEvtBroadcastFlags(&imu_event, EVENT_MASK(5));
		cnt_debug++;
#endif
	}

	return 0;
}
#endif

/*
 *  _____       _                             _
 * |_   _|     | |                           | |
 *   | |  _ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ ___
 *   | | | '_ \| __/ _ \ '__| '__| | | | '_ \| __/ __|
 *  _| |_| | | | ||  __/ |  | |  | |_| | |_) | |_\__ \
 * |_____|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|___/
 *                                     | |
 *                                     |_|
 */
static void mpu6050_interrupt_handler(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		chEvtBroadcastFlagsI(&imu_event, EVENT_MASK(0));
		chSysUnlockFromIsr();
}
static void hmc5883_interrupt_handler(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		chEvtBroadcastFlagsI(&imu_event, EVENT_MASK(1));
		chSysUnlockFromIsr();
}
static const EXTConfig extcfg = {
	{
	    {EXT_CH_MODE_DISABLED, NULL},
   	 	{EXT_CH_MODE_DISABLED, NULL},
   	 	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, mpu6050_interrupt_handler},
		{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, hmc5883_interrupt_handler}
	},
	EXT_MODE_EXTI(0, /* 0 */
	              0, /* 1 */
	              0, /* 2 */
	              0, /* 3 */
	              0, /* 4 */
	              EXT_MODE_GPIOB, /* 5 */
	              0, /* 6 */
	              0, /* 7 */
	              0, /* 8 */
	              0, /* 9 */
	              0, /* 10 */
	              0, /* 11 */
	              0, /* 12 */
	              0, /* 13 */
	              0, /* 14 */
	              EXT_MODE_GPIOC) /* 15 */
};

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThreadLed, 64);
static msg_t ThreadLed(void *arg) {
	(void)arg;
	chRegSetThreadName("LED");
	
	while (TRUE) {
		palClearPad(GPIOB, 1);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, 1);
		chThdSleepMilliseconds(500);
	}
	
	return 0;
}

/*
 *  __  __       _
 * |  \/  |     (_)
 * | \  / | __ _ _ _ __
 * | |\/| |/ _` | | '_ \
 * | |  | | (_| | | | | |
 * |_|  |_|\__,_|_|_| |_|
 *
 */
int main(void) {
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	/*
	 * Activates the serial driver 1 using the driver default configuration.
	 */
	sdStart(&SERIAL_DEBUG, NULL);
	chprintf((BaseChannel *)&SERIAL_DEBUG, "\r\nTrunetcopter\r\n");

	/*
	 * Set I2C1 ports to opendrain
	 */
	palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

	chEvtInit(&imu_event);
	
	EepromOpen(&EepromFile);
	
	chThdSleepMilliseconds(100);
	I2CInitLocal();
	chThdSleepMilliseconds(100);

	baro_ms5611_start();
	imu_mpu6050_start();
	magn_hmc5883_start();
	gps_mtk_start();
	
	algebra_start();

	extStart(&EXTD1, &extcfg);

	/*
	 * Creates the threads.
	 */
	chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO, ThreadLed, NULL);
#ifdef DEBUG
	chThdCreateStatic(waThreadDebug, sizeof(waThreadDebug), NORMALPRIO, ThreadDebug, NULL);
#endif

	return 0;
}
