#ifndef BARO_MS56111_H
#define BARO_MS56111_H

#define MS5611_SLAVE_ADDR 0x77

#define MS5611_RESET    0x1E  // ADC reset command 
#define MS5611_ADC_READ 0x00  // ADC read command 
#define MS5611_ADC_CONV 0x40  // ADC conversion command 
#define MS5611_ADC_D1   0x00  // ADC D1 conversion 
#define MS5611_ADC_D2   0x10  // ADC D2 conversion 
#define MS5611_ADC_256  0x00  // ADC OSR=256 
#define MS5611_ADC_512  0x02  // ADC OSR=512 
#define MS5611_ADC_1024 0x04  // ADC OSR=1024 
#define MS5611_ADC_2048 0x06  // ADC OSR=2048 
#define MS5611_ADC_4096 0x08  // ADC OSR=4096 
#define MS5611_PROM_RD  0xA0  // Prom read command

#define PROM_NB                 8

typedef struct {
	float fbaroms;
	float ftempms;
	float faltims;
} baro_data_t;

void baro_ms5611_start(void);

#endif
