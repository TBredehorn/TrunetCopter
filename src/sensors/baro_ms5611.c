#include "ch.h"
#include "hal.h"

#include "math.h"

#include "baro_ms5611.h"

#include "../main.h"
#include "../i2c_local.h"

uint16_t ms5611_c[PROM_NB];

extern baro_data_t baro_data;

static int8_t baro_ms5611_crc(uint16_t* prom) {
	int32_t i, j;
	uint32_t res = 0;
	uint8_t crc = prom[7] & 0xF;
	prom[7] &= 0xFF00;
	for (i = 0; i < 16; i++) {
		if (i & 1) res ^= ((prom[i>>1]) & 0x00FF);
		else res ^= (prom[i>>1]>>8);
		for (j = 8; j > 0; j--) {
			if (res & 0x8000) res ^= 0x1800;
			res <<= 1;
		}
	}
	prom[7] |= crc;
	if (crc == ((res >> 12) & 0xF)) return 0;
	else return -1;
}

void baro_ms5611_reset(void) {
	uint8_t txbuf[1];

	txbuf[0] = MS5611_RESET;
	i2c_transmit(MS5611_SLAVE_ADDR, txbuf, 1, NULL, 0);
	chThdSleepMicroseconds(2800);
}

uint16_t baro_ms5611_prom(int8_t coef_num) {
	uint16_t rC=0;
	uint8_t txbuf[1];
	uint8_t rxbuf[2];

	txbuf[0] = MS5611_PROM_RD+coef_num*2;
	i2c_transmit(MS5611_SLAVE_ADDR, txbuf, 1, rxbuf, 2); // send PROM READ command
	rC=256*rxbuf[0]; // read MSB
	rC=rC+rxbuf[1]; // read LSB
	return rC;
}

void baro_ms5611_init(void) {
	uint8_t i;
	int8_t n_crc = -1;

	while (n_crc != 0) {
		baro_ms5611_reset(); // soft reset
		for (i=0;i<PROM_NB;i++){
			ms5611_c[i]=baro_ms5611_prom(i);
		} // read all coefficients
		n_crc=baro_ms5611_crc(ms5611_c); //get crc
	}
}

uint32_t baro_ms5611_adc(uint8_t cmd) {
	uint8_t txbuf[1];
	uint8_t rxbuf[3];

	txbuf[0] = MS5611_ADC_CONV + cmd;
	i2c_transmit(MS5611_SLAVE_ADDR, txbuf, 1, NULL, 0); // send conversion command
	switch (cmd & 0x0f) {    // wait necessary conversion time
		case MS5611_ADC_256 : chThdSleepMicroseconds(900); break;
		case MS5611_ADC_512 : chThdSleepMilliseconds(3);   break;
		case MS5611_ADC_1024: chThdSleepMilliseconds(4);   break;
		case MS5611_ADC_2048: chThdSleepMilliseconds(6);   break;
		case MS5611_ADC_4096: chThdSleepMilliseconds(10);  break;
	}
	txbuf[0] = MS5611_ADC_READ;
	i2c_transmit(MS5611_SLAVE_ADDR, txbuf, 1, rxbuf, 3); // read ADC
	
	return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

/**
 * Polling thread
 */
static WORKING_AREA(PollBaroThreadWA, 512);
static msg_t PollBaroThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollBaro");

	while (TRUE) {
		uint32_t D1;    // ADC value of the pressure conversion
		uint32_t D2;    // ADC value of the temperature conversion

		D2=baro_ms5611_adc(MS5611_ADC_D2+MS5611_ADC_4096);   // read D2
		D1=baro_ms5611_adc(MS5611_ADC_D1+MS5611_ADC_4096);   // read D1

		int64_t dt, baroms, tempms, off, sens, t2, off2, sens2;
		/* difference between actual and ref temperature */
		dt = D2 - (int64_t)ms5611_c[5] * (1<<8);
		/* actual temperature */
		tempms = 2000 + ((int64_t)dt * ms5611_c[6]) / (1<<23);
		/* offset at actual temperature */
		off = ((int64_t)ms5611_c[2] * (1<<16)) + ((int64_t)ms5611_c[4] * dt) / (1<<7);
		/* sensitivity at actual temperature */
		sens = ((int64_t)ms5611_c[1] * (1<<15)) + ((int64_t)ms5611_c[3] * dt) / (1<<8);
		/* second order temperature compensation */
		if (tempms < 2000) {
			t2 = (dt*dt) / (1<<31);
			off2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<1);
			sens2 = 5 * ((int64_t)(tempms-2000)*(tempms-2000)) / (1<<2);
			if (tempms < -1500) {
				off2 = off2 + 7 * (int64_t)(tempms+1500)*(tempms+1500);
				sens2 = sens2 + 11 * ((int64_t)(tempms+1500)*(tempms+1500)) / (1<<1);
			}
			tempms = tempms - t2;
			off = off - off2;
			sens = sens - sens2;
		}
		/* temperature compensated pressure */
		baroms = (((int64_t)D1 * sens) / (1<<21) - off) / (1<<15);

		baro_data.ftempms = tempms / 100.;
		baro_data.fbaroms = baroms / 100.;
		baro_data.faltims = ((pow((SEA_LEVEL_PRESSURE / baro_data.fbaroms), 1/5.257) - 1.0) * (baro_data.ftempms + 273.15)) / 0.0065;
	}
	return 0;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
void baro_ms5611_start(void){
	baro_ms5611_init();

	chThdCreateStatic(PollBaroThreadWA,
		sizeof(PollBaroThreadWA),
		I2C_THREADS_PRIO,
		PollBaroThread,
		NULL);
}
