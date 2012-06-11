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

#include "algebra.h"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
uint32_t GlobalFlags = 0;

EepromFileStream EepromFile;

baro_data_t baro_data;
imu_data_t imu_data;
float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

Mutex mtx_imu;

/* 
 * Threads
 */
#ifdef DEBUG
static WORKING_AREA(waThreadDebug, 256);
static msg_t ThreadDebug(void *arg) {
	(void)arg;

	chThdSleepMilliseconds(500);

	while (TRUE) {

		chprintf((BaseChannel *)&SERIAL_DEBUG, "Temperature: %f\r\n", baro_data.ftempms);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Pressure: %f\r\n", baro_data.fbaroms);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "Altitude: %f\r\n", baro_data.faltims);
		chprintf((BaseChannel *)&SERIAL_DEBUG, "----------------------------------\r\n");
		chMtxLock(&mtx_imu);
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
		chMtxUnlock();
		chprintf((BaseChannel *)&SERIAL_DEBUG, "==================================\r\n");
		chThdSleepMilliseconds(500);

		/*
		chMtxLock(&mtx_imu);
		uint8_t i;
		uint8_t * b1 = (uint8_t *) &q0;
		uint8_t * b2 = (uint8_t *) &q1;
		uint8_t * b3 = (uint8_t *) &q2;
		uint8_t * b4 = (uint8_t *) &q3;
		for(i=0; i<4; i++) {
			uint8_t b1q1 = (b1[i] >> 4) & 0x0f;
			uint8_t b2q1 = (b1[i] & 0x0f);

			uint8_t c1q1 = (b1q1 < 10) ? ('0' + b1q1) : 'A' + b1q1 - 10;
			uint8_t c2q1 = (b2q1 < 10) ? ('0' + b2q1) : 'A' + b2q1 - 10;

			sdWrite(&SERIAL_DEBUG, &c1q1, 1);
			sdWrite(&SERIAL_DEBUG, &c2q1, 1);
		}
		chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
		for(i=0; i<4; i++) {
			uint8_t b1q2 = (b2[i] >> 4) & 0x0f;
			uint8_t b2q2 = (b2[i] & 0x0f);

			uint8_t c1q2 = (b1q2 < 10) ? ('0' + b1q2) : 'A' + b1q2 - 10;
			uint8_t c2q2 = (b2q2 < 10) ? ('0' + b2q2) : 'A' + b2q2 - 10;

			sdWrite(&SERIAL_DEBUG, &c1q2, 1);
			sdWrite(&SERIAL_DEBUG, &c2q2, 1);
		}
		chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
		for(i=0; i<4; i++) {
			uint8_t b1q3 = (b3[i] >> 4) & 0x0f;
			uint8_t b2q3 = (b3[i] & 0x0f);

			uint8_t c1q3 = (b1q3 < 10) ? ('0' + b1q3) : 'A' + b1q3 - 10;
			uint8_t c2q3 = (b2q3 < 10) ? ('0' + b2q3) : 'A' + b2q3 - 10;

			sdWrite(&SERIAL_DEBUG, &c1q3, 1);
			sdWrite(&SERIAL_DEBUG, &c2q3, 1);
		}
		chprintf((BaseChannel *)&SERIAL_DEBUG, ",");
		for(i=0; i<4; i++) {
			uint8_t b1q4 = (b4[i] >> 4) & 0x0f;
			uint8_t b2q4 = (b4[i] & 0x0f);

			uint8_t c1q4 = (b1q4 < 10) ? ('0' + b1q4) : 'A' + b1q4 - 10;
			uint8_t c2q4 = (b2q4 < 10) ? ('0' + b2q4) : 'A' + b2q4 - 10;

			sdWrite(&SERIAL_DEBUG, &c1q4, 1);
			sdWrite(&SERIAL_DEBUG, &c2q4, 1);
		}
		chprintf((BaseChannel *)&SERIAL_DEBUG, ",\r\n");
		//chprintf((BaseChannel *)&SERIAL_DEBUG, "%f,", q0);
		//chprintf((BaseChannel *)&SERIAL_DEBUG, "%f,", q1);
		//chprintf((BaseChannel *)&SERIAL_DEBUG, "%f,", q2);
		//chprintf((BaseChannel *)&SERIAL_DEBUG, "%f\r\n", q3);
		chMtxUnlock();
		chThdSleepMilliseconds(20);
		*/
	}

	return 0;
}
#endif

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThreadLed, 64);
static msg_t ThreadLed(void *arg) {
	(void)arg;
	
	while (TRUE) {
		palClearPad(GPIOB, 1);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, 1);
		chThdSleepMilliseconds(500);
	}
	
	return 0;
}

/*
 * Application entry point.
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

	chMtxInit(&mtx_imu);
	
	EepromOpen(&EepromFile);
	
	I2CInitLocal();

	baro_ms5611_start();
	imu_mpu6050_start();
	magn_hmc5883_start();
	
	algebra_start();
	/*
	 * Creates the threads.
	 */
	chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO, ThreadLed, NULL);
#ifdef DEBUG
	chThdCreateStatic(waThreadDebug, sizeof(waThreadDebug), NORMALPRIO, ThreadDebug, NULL);
#endif

	return 0;
}
