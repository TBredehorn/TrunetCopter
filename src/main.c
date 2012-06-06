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

/* ARM includes */
#include "math.h"

/* MPU6050 */
#include "MPU60X0.h"

uint8_t txbuf[8];
uint8_t rxbuf[8];

int32_t P;   // compensated pressure value 
int32_t T;   // compensated temperature value
#define SEA_LEVEL_PRESSURE 1020.00
int32_t ALT; // calculated altitude

static
void out(char *msg)
{
    size_t c = 0;
    while (msg[c] != '\0')
        chIOPut(&SD1, msg[c++]);
   chThdSleepMilliseconds(10);
}

static
void outInt(uint16_t n, size_t magnitude)
{
    char msg[64];
    size_t c = 0;
    size_t mag = magnitude;
    while (c < 63) {
        msg[c++] = 48 + (n / mag) % 10;
        if (mag <= 1)
            break;
        mag = mag / 10;
    }
    msg[c] = '\0';
    out(msg);
}

static
void outln(char *msg)
{
    out(msg);
    chIOPut(&SD1, '\n');
    chIOPut(&SD1, '\r');
    chThdSleepMilliseconds(10);
}

/*
 * I2C
 */
static I2CConfig i2ccfg = {
                           OPMODE_I2C,
                           400000,
                           //STD_DUTY_CYCLE
                           FAST_DUTY_CYCLE_16_9
                          };

/* MS561101BA */
#define MS561101BA_ADDR 0x77

#define MS561101BA_RESET    0x1E  // ADC reset command 
#define MS561101BA_ADC_READ 0x00  // ADC read command 
#define MS561101BA_ADC_CONV 0x40  // ADC conversion command 
#define MS561101BA_ADC_D1   0x00  // ADC D1 conversion 
#define MS561101BA_ADC_D2   0x10  // ADC D2 conversion 
#define MS561101BA_ADC_256  0x00  // ADC OSR=256 
#define MS561101BA_ADC_512  0x02  // ADC OSR=512 
#define MS561101BA_ADC_1024 0x04  // ADC OSR=1024 
#define MS561101BA_ADC_2048 0x06  // ADC OSR=2048 
#define MS561101BA_ADC_4096 0x08  // ADC OSR=4096 
#define MS561101BA_PROM_RD  0xA0  // Prom read command

uint16_t C[8];   // calibration coefficients

void ms5611_reset(void);
uint32_t ms5611_adc(uint8_t cmd);
uint16_t ms5611_prom(int8_t coef_num);
uint8_t ms5611_crc4(uint16_t n_prom[]);

void ms5611_reset(void) {
	txbuf[0] = MS561101BA_RESET;
	i2cMasterTransmit(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 0);
	chThdSleepMilliseconds(3);
}

uint32_t ms5611_adc(uint8_t cmd) {
	uint32_t temp=0;
	txbuf[0] = MS561101BA_ADC_CONV + cmd;
	i2cMasterTransmit(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 0); // send conversion command
	switch (cmd & 0x0f) {    // wait necessary conversion time
		case MS561101BA_ADC_256 : chThdSleepMicroseconds(900); break;
		case MS561101BA_ADC_512 : chThdSleepMilliseconds(3);   break;
		case MS561101BA_ADC_1024: chThdSleepMilliseconds(4);   break;
		case MS561101BA_ADC_2048: chThdSleepMilliseconds(6);   break;
		case MS561101BA_ADC_4096: chThdSleepMilliseconds(10);  break;
	}
	
	txbuf[0] = MS561101BA_ADC_READ;
	i2cMasterTransmit(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 3); // read ADC
	
	temp = 65536*rxbuf[0]; // read MSB
	temp = temp+256*rxbuf[1]; // read byte
	temp = temp+rxbuf[2]; // read LSB and not acknowledge
	
	return temp;
}

uint16_t ms5611_prom(int8_t coef_num) {
	uint16_t rC=0;
	
	txbuf[0] = MS561101BA_PROM_RD+coef_num*2;
	i2cMasterTransmit(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 2); // send PROM READ command
	rC=256*rxbuf[0]; // read MSB
	rC=rC+rxbuf[1]; // read LSB
	return rC;
}

uint8_t ms5611_crc4(uint16_t n_prom[]) {
	int cnt;     // simple counter
	unsigned int n_rem;    // crc reminder
	unsigned int crc_read;   // original value of the crc
	unsigned char  n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7];    //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7]));  //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) {     // operation is performed on bytes
     									// choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
  		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12));  // final 4-bit reminder is CRC code
	n_prom[7]=crc_read;   // restore the crc_read to its original place
	return (n_rem ^ 0x0);
}

static WORKING_AREA(waThreadBaro, 512);
static msg_t ThreadBaro(void *arg) {
	(void)arg;
	while (TRUE) {
		uint32_t D1;    // ADC value of the pressure conversion 
		uint32_t D2;    // ADC value of the temperature conversion
		int64_t dT;   // difference between actual and measured temperature 
		int64_t OFF;   // offset at actual temperature 
		int64_t SENS;   // sensitivity at actual temperature
		D1=0;
		D2=0;
		i2cAcquireBus(&I2CD1);
		D2=ms5611_adc(MS561101BA_ADC_D2+MS561101BA_ADC_4096);   // read D2 
		D1=ms5611_adc(MS561101BA_ADC_D1+MS561101BA_ADC_4096);   // read D1
		i2cReleaseBus(&I2CD1);

		//chprintf((BaseChannel *)&SD1, "Uncompensated Temperature: %d\r\n", D2);
		//chprintf((BaseChannel *)&SD1, "Uncompensated Pressure: %d\r\n", D1);

		// calculate 1st order pressure and temperature (MS5607 1st order algorithm)
		dT = D2 - ((uint64_t)C[5] << 8);
		OFF  = ((int64_t)C[2] << 16) + ((dT * C[4]) >> 7);
		SENS = ((int32_t)C[1] << 15) + ((dT * C[3]) >> 8);

		T = 2000 + dT * C[6]/8388608;
		P = (D1 * SENS/2097152 - OFF)/32768;

		if (T < 2000) { // calculate 2nd order pressure and temperature (MS5607 2nd order algorithm)
			int32_t T1    = 0;
			int64_t OFF1  = 0;
			int64_t SENS1 = 0;

			T1    = pow(dT, 2) / 2147483648;
			OFF1  = 5 * pow((T - 2000), 2) / 2;
			SENS1 = 5 * pow((T - 2000), 2) / 4;

			if(T < -1500) {
				OFF1  = OFF1 + 7 * pow((T + 1500), 2); 
				SENS1 = SENS1 + 11 * pow((T + 1500), 2) / 2;
			}

			T -= T1;
			OFF -= OFF1; 
			SENS -= SENS1;
		}

		float pressure, temperature;
		float altitude;
		pressure = P/100;
		temperature = T/100;
		altitude = ((pow((SEA_LEVEL_PRESSURE / pressure), 1/5.257) - 1.0) * (temperature + 273.15)) / 0.0065;
		ALT = altitude * 100;
		
		chprintf((BaseChannel *)&SD1, "Temperature: %d\r\n", T);
		chprintf((BaseChannel *)&SD1, "Pressure: %d\r\n", P);
		chprintf((BaseChannel *)&SD1, "Altitude: %d\r\n", ALT);
		
		chThdSleepMilliseconds(500);
	}
	
	return 0;
}

static WORKING_AREA(waThreadIMU, 256);
static msg_t ThreadIMU(void *arg) {
	(void)arg;
	while (TRUE) {
		i2cAcquireBus(&I2CD1);
		mpu_i2c_read_data(0x3B, 14); /* Read accelerometer, temperature and gyro data */
		i2cReleaseBus(&I2CD1);
		chThdSleepMilliseconds(500);
	}
	
	return 0;
}

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThreadLed, 128);
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
	sdStart(&SD1, NULL);
	sdWrite(&SD1, (uint8_t *)"Trunetcopter\r\n", 14);

	/*
	 * I2C Initiliazation
	 */
	i2cInit();
	chThdSleepMilliseconds(20);
	i2cStart(&I2CD1, &i2ccfg);

	/*
	 * Set I2C1 ports to opendrain
	 */
	palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

	/*
	 * MPU6050 Initialization
	 */
	set_mpu_sample_rate(9);
	set_mpu_config_regsiter(EXT_SYNC_SET0, DLPF_CFG0);
	set_mpu_gyro(XG_ST_DIS, YG_ST_DIS, ZG_ST_DIS, FS_SEL_250);
	set_mpu_accel(XA_ST_DIS, YA_ST_DIS, ZA_ST_DIS, AFS_SEL_2g, ACCEL_HPF0);
	set_mpu_power_mgmt1(DEVICE_RESET_DIS, SLEEP_DIS, CYCLE_DIS, TEMPERATURE_EN, CLKSEL_XG);
	set_mpu_user_control(USER_FIFO_DIS, I2C_MST_DIS, I2C_IF_DIS, FIFO_RESET_DIS, I2C_MST_RESET_DIS, SIG_COND_RESET_DIS);

	write_mpu_power_mgmt1();
	write_mpu_gyro();
	write_mpu_accel();
	write_mpu_sample_rate();

	/*
	 * MS561101BA Initialization
	 */
	int i;
	uint8_t n_crc; // crc value of the prom
	ms5611_reset(); // reset IC
	for (i=0;i<8;i++){
		C[i]=ms5611_prom(i);
		chprintf((BaseChannel *)&SD1, "C%d: %d\r\n", i, C[i]);
	} // read coefficients 
	n_crc=ms5611_crc4(C); // calculate the CRC
	if (n_crc == 0) {
		chprintf((BaseChannel *)&SD1, "MS5611 CRC OK - Calculated: %x\r\n", C[7]);
	} else {
		chprintf((BaseChannel *)&SD1, "MS5611 CRC NOT OK - Calculate: %x\r\n", C[7]);
	}

	/*
	 * Creates the threads.
	 */
	chThdCreateStatic(waThreadBaro, sizeof(waThreadBaro), NORMALPRIO, ThreadBaro, NULL);
	chThdCreateStatic(waThreadIMU, sizeof(waThreadIMU), NORMALPRIO, ThreadIMU, NULL);
	chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO, ThreadLed, NULL);

	return 0;
}
