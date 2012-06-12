#include "ch.h"
#include "hal.h"

#include "math.h"

#include "magn_hmc5883.h"
#include "imu_mpu6050.h"

#include "../main.h"
#include "../i2c_local.h"

extern imu_data_t imu_data;
extern EventSource imu_event;

void hmc5883_init(void) {
	uint8_t txbuf[2];

	x_scale=1.0F;
	y_scale=1.0F;
	z_scale=1.0F;
	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = 0x70; // 8 samples averaged, 75Hz frequency, no artificial bias.
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
	txbuf[0] = HMC58X3_R_CONFB;
	txbuf[1] = 0xA0;
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
	txbuf[0] = HMC58X3_R_MODE;
	txbuf[1] = 0x00;
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
}

void hmc5883_setGain(unsigned char gain) {
	uint8_t txbuf[2];

	// 0-7, 1 default
	if (gain > 7) return;
	txbuf[0] = HMC58X3_R_CONFB;
	txbuf[1] = gain << 5;
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
}

void hmc5883_setMode(unsigned char mode) {
	uint8_t txbuf[2];

	if (mode > 2) {
		return;
	}

	txbuf[0] = HMC58X3_R_MODE;
	txbuf[1] = mode;
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
	chThdSleepMilliseconds(100);
}

void hmc5883_getRaw(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t txbuf[1];
	uint8_t rxbuf[6];

	txbuf[0] = HMC58X3_R_XM; // will start from DATA X MSB and fetch all the others
	i2c_transmit(HMC58X3_ADDR, txbuf, 1, rxbuf, 6);

	// read out the 3 values, 2 bytes each.
	*x = (rxbuf[0] << 8) | rxbuf[1];
	*z = (rxbuf[2] << 8) | rxbuf[3];
	*y = (rxbuf[4] << 8) | rxbuf[5];
}

void hmc5883_getValuesfloat(void) {
  int16_t xr,yr,zr;

  hmc5883_getRaw(&xr, &yr, &zr);
  imu_data.mag_x = ((float) xr) / x_scale;
  imu_data.mag_y = ((float) yr) / y_scale;
  imu_data.mag_z = ((float) zr) / z_scale;
}

void hmc5883_calibrate(unsigned char gain) {
	x_scale=1; // get actual values
	y_scale=1;
	z_scale=1;
	uint8_t i;
	uint8_t txbuf[2];

	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = 0x010 + HMC_POS_BIAS; // Reg A DOR=0x010 + MS1,MS0 set to pos bias
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
	hmc5883_setGain(gain);
	float x, y, z, mx=0, my=0, mz=0, t=10;

	for (i=0; i<(int)t; i++) {
		hmc5883_setMode(1);
		hmc5883_getValuesfloat();
		x = imu_data.mag_x;
		y = imu_data.mag_y;
		z = imu_data.mag_z;
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
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
}

void hmc5883_setDOR(unsigned char DOR) {
	uint8_t txbuf[2];

	if (DOR>6) return;
	txbuf[0] = HMC58X3_R_CONFA;
	txbuf[1] = DOR<<2;
	i2c_transmit(HMC58X3_ADDR, txbuf, 2, NULL, 0);
}

/**
 * Polling thread
 */
static WORKING_AREA(PollMagnThreadWA, 512);
static msg_t PollMagnThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollMagn");

	struct EventListener self_el;
	chEvtRegister(&imu_event, &self_el, 1);

	while (TRUE) {
		chEvtWaitOne(EVENT_MASK(0));
		hmc5883_getValuesfloat();
		chEvtBroadcastFlags(&imu_event, EVENT_MASK(1));
		chThdSleepMilliseconds(15);
	}
	return 0;
}


void magn_hmc5883_start(void){
	hmc5883_init();
	hmc5883_calibrate(1);
	hmc5883_setMode(0);
	chThdSleepMilliseconds(6);
	hmc5883_setDOR(0b110);

	chThdCreateStatic(PollMagnThreadWA,
		sizeof(PollMagnThreadWA),
		I2C_THREADS_PRIO,
		PollMagnThread,
		NULL);
}
