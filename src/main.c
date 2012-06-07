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
 * Global Variables
 */
float q[4];
float iq0, iq1, iq2, iq3;
float exInt, eyInt, ezInt;  // scaled integral error
volatile float twoKp;      // 2 * proportional gain (Kp)
volatile float twoKi;      // 2 * integral gain (Ki)
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx,  integralFBy, integralFBz;
systime_t lastUpdate, now; // sample period expressed in milliseconds
float sampleFreq; // half the sample period expressed in seconds
int startLoopTime;
float values[9];

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

/*
 * Quartenion
 */
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/* 
 * MPU6050
 */
#include "MPU60X0.h"

uint8_t txbuf[8];
uint8_t rxbuf[8];

int32_t P;   // compensated pressure value 
int32_t T;   // compensated temperature value
#define SEA_LEVEL_PRESSURE 1020.00
int32_t ALT; // calculated altitude

/*
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
*/

/*
 * USART
 */ 
/*
SerialConfig config = {
    115200
  };
*/

/*
 * I2C
 */
static I2CConfig i2ccfg = {
                           OPMODE_I2C,
                           400000,
                           //STD_DUTY_CYCLE
                           FAST_DUTY_CYCLE_16_9
                          };

/**
 * @brief     Calculates requred timeout.
 */
static systime_t calc_timeout(I2CDriver *i2cp, size_t txbytes, size_t rxbytes){
  const uint32_t bitsinbyte = 10;
  uint32_t tmo;
  tmo = ((txbytes + rxbytes + 1) * bitsinbyte * 1000);
  tmo /= i2cp->config->clock_speed;
  tmo += 5; /* some additional time to be safer */
  return MS2ST(tmo);
}

/*
 * HMC5883L
 */
#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (HMC58X3_X_SELF_TEST_GAUSS)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

float x_scale,y_scale,z_scale,x_max,y_max,z_max;
const int16_t counts_per_milligauss[8]={  
  1370,
  1090,
  820,
  660,
  440,
  390,
  330,
  230
};

void hmc5883_init(void) {
	x_scale=1.0F;
	y_scale=1.0F;
	z_scale=1.0F;
	systime_t tmo = calc_timeout(&I2CD1, 2, 0);
	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = 0x70; // 8 samples averaged, 75Hz frequency, no artificial bias.
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
	txbuf[0] = HMC58X3_R_CONFB;
	txbuf[1] = 0xA0;
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
	txbuf[0] = HMC58X3_R_MODE;
	txbuf[1] = 0x00;
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
}

void hmc5883_setGain(unsigned char gain) { 
	systime_t tmo = calc_timeout(&I2CD1, 2, 0);
	// 0-7, 1 default
	if (gain > 7) return;
	txbuf[0] = HMC58X3_R_CONFB;
	txbuf[1] = gain << 5;
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
}

void hmc5883_setMode(unsigned char mode) {
	systime_t tmo = calc_timeout(&I2CD1, 2, 0);
	
	if (mode > 2) {
		return;
	}

	txbuf[0] = HMC58X3_R_MODE;
	txbuf[1] = mode;
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
	chThdSleepMilliseconds(100);
}

void hmc5883_getRaw(int *x,int *y,int *z) {
	systime_t tmo = calc_timeout(&I2CD1, 1, 6);
	txbuf[0] = HMC58X3_R_XM; // will start from DATA X MSB and fetch all the others
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 1, rxbuf, 6, tmo);
	
	// read out the 3 values, 2 bytes each.
	*x = (rxbuf[0] << 8) | rxbuf[1];
	*z = (rxbuf[2] << 8) | rxbuf[3];
	*y = (rxbuf[4] << 8) | rxbuf[5];
}

void hmc5883_getValuesfloat(float *x,float *y,float *z) {
  int xr,yr,zr;
  
  hmc5883_getRaw(&xr, &yr, &zr);
  *x= ((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = ((float) zr) / z_scale;
}

void hmc5883_getValuesint(int *x,int *y,int *z) {
  float fx,fy,fz;
  hmc5883_getValuesfloat(&fx,&fy,&fz);
  *x= (int) (fx + 0.5);
  *y= (int) (fy + 0.5);
  *z= (int) (fz + 0.5);
}

void hmc5883_calibrate(unsigned char gain) {
	x_scale=1; // get actual values
	y_scale=1;
	z_scale=1;
	systime_t tmo = calc_timeout(&I2CD1, 2, 0);
	int i;
	
	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = 0x010 + HMC_POS_BIAS; // Reg A DOR=0x010 + MS1,MS0 set to pos bias
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
	hmc5883_setGain(gain);
	float x, y, z, mx=0, my=0, mz=0, t=10;

	for (i=0; i<(int)t; i++) { 
		hmc5883_setMode(1);
		hmc5883_getValuesfloat(&x,&y,&z);
		if (x>mx) mx=x;
		if (y>my) my=y;
		if (z>mz) mz=z;
	}

	float max=0;
	if (mx>max) max=mx;
	if (my>max) max=my;
	if (mz>max) max=mz;
	x_max=mx;
	y_max=my;
	z_max=mz;
	x_scale=max/mx; // calc scales
	y_scale=max/my;
	z_scale=max/mz;

	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = 0x010; // set RegA/DOR back to default
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
}

void hmc5883_setDOR(unsigned char DOR) {
	systime_t tmo = calc_timeout(&I2CD1, 2, 0);
	
	if (DOR>6) return;
	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = DOR<<2;
	i2cMasterTransmitTimeout(&I2CD1, HMC58X3_ADDR, txbuf, 2, rxbuf, 0, tmo);
}

/* 
 * MS561101BA 
 */
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
	systime_t tmo = calc_timeout(&I2CD1, 1, 0);
	txbuf[0] = MS561101BA_RESET;
	i2cMasterTransmitTimeout(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 0, tmo);
	chThdSleepMilliseconds(3);
}

uint32_t ms5611_adc(uint8_t cmd) {
	uint32_t temp=0;
	systime_t tmo = calc_timeout(&I2CD1, 1, 0);
	txbuf[0] = MS561101BA_ADC_CONV + cmd;
	i2cMasterTransmitTimeout(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 0, tmo); // send conversion command
	switch (cmd & 0x0f) {    // wait necessary conversion time
		case MS561101BA_ADC_256 : chThdSleepMicroseconds(900); break;
		case MS561101BA_ADC_512 : chThdSleepMilliseconds(3);   break;
		case MS561101BA_ADC_1024: chThdSleepMilliseconds(4);   break;
		case MS561101BA_ADC_2048: chThdSleepMilliseconds(6);   break;
		case MS561101BA_ADC_4096: chThdSleepMilliseconds(10);  break;
	}
	
	tmo = calc_timeout(&I2CD1, 1, 3);
	txbuf[0] = MS561101BA_ADC_READ;
	i2cMasterTransmitTimeout(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 3, tmo); // read ADC
	
	temp = 65536*rxbuf[0]; // read MSB
	temp = temp+256*rxbuf[1]; // read byte
	temp = temp+rxbuf[2]; // read LSB and not acknowledge
	
	return temp;
}

uint16_t ms5611_prom(int8_t coef_num) {
	uint16_t rC=0;
	systime_t tmo = calc_timeout(&I2CD1, 1, 0);
	
	txbuf[0] = MS561101BA_PROM_RD+coef_num*2;
	i2cMasterTransmitTimeout(&I2CD1, MS561101BA_ADDR, txbuf, 1, rxbuf, 2, tmo); // send PROM READ command
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

/* 
 * Threads
 */
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

			T1    = pow(dT, 2) / 2147483648u;
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
		
		//chprintf((BaseChannel *)&SD1, "Temperature: %d\r\n", T);
		//chprintf((BaseChannel *)&SD1, "Pressure: %d\r\n", P);
		//chprintf((BaseChannel *)&SD1, "Altitude: %d\r\n", ALT);
		
		chThdSleepMilliseconds(50);
	}
	
	return 0;
}

static WORKING_AREA(waThreadIMU, 256);
static msg_t ThreadIMU(void *arg) {
	(void)arg;
	while (TRUE) {
		int16_t accgyroval[6];
		int i;
		i2cAcquireBus(&I2CD1);
		mpu_i2c_read_data(0x3B, 14, &accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]); /* Read accelerometer, temperature and gyro data */
		i2cReleaseBus(&I2CD1);
		for(i = 0; i<6; i++) {
			if(i < 3) {
				values[i] = (float) accgyroval[i];
			} else {
				values[i] = ((float) accgyroval[i]) / 16.4f; // NOTE: this depends on the sensitivity chosen
			}
		}
		chThdSleepMilliseconds(50);
	}
	
	return 0;
}

static WORKING_AREA(waThreadMagn, 256);
static msg_t ThreadMagn(void *arg) {
	(void)arg;
	while (TRUE) {
		i2cAcquireBus(&I2CD1);
		hmc5883_getValuesfloat(&values[6], &values[7], &values[8]);
		i2cReleaseBus(&I2CD1);
		
		//chprintf((BaseChannel *)&SD1, "Magnetometer: %d\t%d\t%d\r\n", values[0], values[1], values[2]);
		
		chThdSleepMilliseconds(50);
	}
	
	return 0;
}

static WORKING_AREA(waThreadQ, 512);
static msg_t ThreadQ(void *arg) {
	(void)arg;
	now = chTimeNow();
	while (TRUE) {
		now += MS2ST(100);
		sampleFreq = 10;

		// gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
		AHRSupdate(values[3] * M_PI/180, values[4] * M_PI/180, values[5] * M_PI/180, values[0], values[1], values[2], values[6], values[7], values[8]);

		chprintf((BaseChannel *)&SD1, "%f,%f,%f,%f\r\n", q0, q1, q2, q3);

		chThdSleepUntil(now);
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
		
		//int i;
		//for (i=0;i<9; i++) {
		//	chprintf((BaseChannel *)&SD1, "value[%d]: %f\r\n", i, values[i]);
		//}
		
		//chprintf((BaseChannel *)&SD1, "millis(): %ld\r\n", MS2ST(chTimeNow()));
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
	chThdSleepMilliseconds(10);
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
	set_mpu_gyro(XG_ST_DIS, YG_ST_DIS, ZG_ST_DIS, FS_SEL_2000);
	set_mpu_accel(XA_ST_DIS, YA_ST_DIS, ZA_ST_DIS, AFS_SEL_2g, ACCEL_HPF0);
	set_mpu_power_mgmt1(DEVICE_RESET_DIS, SLEEP_DIS, CYCLE_DIS, TEMPERATURE_EN, CLKSEL_XG);
	set_mpu_user_control(USER_FIFO_DIS, I2C_MST_DIS, I2C_IF_DIS, FIFO_RESET_DIS, I2C_MST_RESET_DIS, SIG_COND_RESET_DIS);

	write_mpu_power_mgmt1();
	write_mpu_int_cfg(); // enable I2C Auxiliary bypass on MPU
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
		//chprintf((BaseChannel *)&SD1, "C%d: %d\r\n", i, C[i]);
	} // read coefficients 
	n_crc=ms5611_crc4(C); // calculate the CRC
	/*
	if (n_crc == 0) {
		chprintf((BaseChannel *)&SD1, "MS5611 CRC OK - Calculated: %x\r\n", C[7]);
	} else {
		chprintf((BaseChannel *)&SD1, "MS5611 CRC NOT OK - Calculate: %x\r\n", C[7]);
	}
	*/

	/*
	 * HMC5883L
	 */
	hmc5883_init();
	hmc5883_calibrate(1);
	hmc5883_setMode(0);
	chThdSleepMilliseconds(6);
	hmc5883_setDOR(0b110);

	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

	/*
	 * FM24V10
	 */
	

	/*
	 * Creates the threads.
	 */
	chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO, ThreadLed, NULL);
	//chThdCreateStatic(waThreadBaro, sizeof(waThreadBaro), NORMALPRIO, ThreadBaro, NULL);
	//chThdCreateStatic(waThreadIMU, sizeof(waThreadIMU), NORMALPRIO, ThreadIMU, NULL);
	//chThdCreateStatic(waThreadMagn, sizeof(waThreadMagn), NORMALPRIO, ThreadMagn, NULL);
	//chThdCreateStatic(waThreadQ, sizeof(waThreadQ), NORMALPRIO, ThreadQ, NULL);

	return 0;
}
