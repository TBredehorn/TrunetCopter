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

/* Project includes */
#include "eeprom.h"
static EepromFileStream Efs;

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
                           100000,
                           STD_DUTY_CYCLE
                           //FAST_DUTY_CYCLE_16_9
                          };

/*
 * BMP085
 */
static float barometric_altitude=0;
//static float sonar_altitude=0;
int temperature = 0;
int pressure = 0;

//#define bmp085_addr 0b1110111
//#define bmp085_addr 0xee
#define bmp085_addr 0x77

static struct {
                short ac1;
                short ac2;
                short ac3;
                unsigned short ac4;
                unsigned short ac5;
                unsigned short ac6;
                short b1;
                short b2;
                short mb;
                short mc;
                short md;
} BMP085_cfg;
//static float pressure_0=1;
const unsigned char bmp085_oversampling = 3;
const unsigned int bmp085_pressure_waittime[4] = { 5, 8, 14, 26 };

short bmp085ReadShort (unsigned char addr) {
  uint8_t raw[2];
  short data;
  msg_t status = RDY_OK;
  static i2cflags_t errors = 0;

  //outln("AQUI3");
  status = i2cMasterTransmit(&I2CD1, bmp085_addr, &addr, 1, raw, 2);
  //outln("AQUI4");
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
	out("Error:");
	outln((char *)errors);
	
	return 0;
  } else {
	data = (raw[0] << 8) + raw[1];
    //out("Read:");
    //outInt(data, 10000);
    //outln("");
	return data;
  }
}

unsigned short bmp085ReadTemp (void) {
  unsigned char data[2] = {0xf4, 0x2e};

  //outln("AQUI1");
  i2cMasterTransmit(&I2CD1, bmp085_addr, data, 2, NULL, 0);
  //outln("AQUI2");
  chThdSleepMilliseconds(5);
  
  return bmp085ReadShort(0xf6);
}

int bmp085ReadPressure (void) {
  unsigned char data[2] = {0xf4, 0x34+(bmp085_oversampling<<6)};
  unsigned char addr;
  uint8_t raw[3];
  msg_t status = RDY_OK;
  static i2cflags_t errors = 0;

  status = i2cMasterTransmit(&I2CD1, bmp085_addr, data, 2, NULL, 0);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
	out("Error Pressure 1:");
	outln((char *)errors);
  }
  chThdSleepMilliseconds(bmp085_pressure_waittime[bmp085_oversampling]);

  /*  
  addr = 0xf6;
  i2cMasterTransmit(&I2CD1, 0xee, &addr, 1, NULL, 0);
  i2cMasterReceive(&I2CD1, 0xee, &raw[0], 1);
  addr = 0xf7;
  i2cMasterTransmit(&I2CD1, 0xee, &addr, 1, NULL, 0);
  i2cMasterReceive(&I2CD1, 0xee, &raw[1], 1);
  addr = 0xf8;
  i2cMasterTransmit(&I2CD1, 0xee, &addr, 1, NULL, 0);
  i2cMasterReceive(&I2CD1, 0xee, &raw[2], 1);
  */
  addr = 0xf6;
  status = i2cMasterTransmit(&I2CD1, bmp085_addr, &addr, 1, raw, 3);
  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD1);
	out("Error Pressure 2:");
	outln((char *)errors);
  }

  return (int)(((int)raw[0]<<16) | ((int)raw[1]<<8) | ((int)raw[2]))>>(8-bmp085_oversampling);
}

void bmp085ReadTemperaturePressure (int* temperature, int* pressure) {
  int ut = bmp085ReadTemp();
  int up = bmp085ReadPressure();

  int x1, x2, x3, b3, b5, b6, p;
  unsigned int b4, b7;

  //calculate the temperature
  x1 = ((int)ut - BMP085_cfg.ac6) * BMP085_cfg.ac5 >> 15;
  x2 = ((int)BMP085_cfg.mc << 11) / (x1 + BMP085_cfg.md);
  b5 = x1 + x2;
  *temperature = (b5 + 8) >> 4;

  //calculate the pressure
  b6 = b5 - 4000;
  x1 = (BMP085_cfg.b2 * (b6 * b6 >> 12)) >> 11;
  x2 = BMP085_cfg.ac2 * b6 >> 11;
  x3 = x1 + x2;

  if (bmp085_oversampling == 3) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) << 1;
  if (bmp085_oversampling == 2) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2);
  if (bmp085_oversampling == 1) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) >> 1;
  if (bmp085_oversampling == 0) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) >> 2;

  x1 = BMP085_cfg.ac3 * b6 >> 13;
  x2 = (BMP085_cfg.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (BMP085_cfg.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> bmp085_oversampling);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *pressure = p + ((x1 + x2 + 3791) >> 4);
}
//static float BAR_ALT_SMOOTHING = 0.95;

/*
static WORKING_AREA(wa_BMP085_Thread, 512);
__attribute__((noreturn))
static msg_t BMP085_Thread(void *arg) {
  (void)arg;
  while (1) {
    i2cAcquireBus(&I2CD1);
    bmp085ReadTemperaturePressure(&temperature, &pressure);
    i2cReleaseBus(&I2CD1);
    barometric_altitude = BAR_ALT_SMOOTHING*barometric_altitude + (1-BAR_ALT_SMOOTHING)*(-44330.0*((float)pressure/pressure_0-1.0)*(1.0/5.255));
    //if (sonar_altitude>0.5 && sonar_altitude<5.0) { // baro alt seems to drift, adjust p0
    //  float dp0 = ((float)pressure)/(1-sonar_altitude*5.255/44300.0) - pressure_0;
    //  pressure_0 += 0.1*dp0;
    //}
    chThdSleepMilliseconds(1);
  }
}
*/

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThread1, 1024);
static msg_t Thread1(void *arg) {

  (void)arg;
  while (TRUE) {
    palClearPad(GPIOB, 1);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, 1);
    chThdSleepMilliseconds(500);

	//temperature = bmp085ReadTemp();
	//pressure = bmp085ReadPressure();
	i2cAcquireBus(&I2CD1);
	bmp085ReadTemperaturePressure(&temperature, &pressure);
	i2cReleaseBus(&I2CD1);

    int x1, x2, x3, b3, b5, b6, p;
    unsigned int b4, b7;

    //calculate the temperature
    x1 = ((int)temperature - BMP085_cfg.ac6) * BMP085_cfg.ac5 >> 15;
    x2 = ((int)BMP085_cfg.mc << 11) / (x1 + BMP085_cfg.md);
    b5 = x1 + x2;
    temperature = (b5 + 8) >> 4;

    //calculate the pressure
    b6 = b5 - 4000;
    x1 = (BMP085_cfg.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = BMP085_cfg.ac2 * b6 >> 11;
    x3 = x1 + x2;

    if (bmp085_oversampling == 3) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) << 1;
    if (bmp085_oversampling == 2) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2);
    if (bmp085_oversampling == 1) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) >> 1;
    if (bmp085_oversampling == 0) b3 = ((int32_t)BMP085_cfg.ac1 * 4 + x3 + 2) >> 2;

    x1 = BMP085_cfg.ac3 * b6 >> 13;
    x2 = (BMP085_cfg.b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (BMP085_cfg.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t)pressure - b3) * (50000 >> bmp085_oversampling);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4);

    //barometric_altitude = BAR_ALT_SMOOTHING*barometric_altitude + (1-BAR_ALT_SMOOTHING)*(-44330.0*((float)pressure/pressure_0-1.0)*(1.0/5.255));
    //barometric_altitude = 44330 * (1.0 - pow(pressure / 101325, 0.1903)); //101325=sealevelpressure
    float A = (float)pressure/101325;
	float B = 1/5.25588;
	float C = pow(A,B);
	C = 1 - C;
	C = C /0.0000225577;
	barometric_altitude = C;

	out("Altitude:");
    outInt(barometric_altitude, 100000);
    outln("");

    out("Pressure:");
    outInt(pressure, 100000);
    outln("");

    out("Temperature:");
    outInt(temperature, 100000);
    outln("");

    /*
     * EEPROM Read Test
     */
#define eeprom_split_addr(txbuf, addr){                                       \
  (txbuf)[0] = ((uint8_t)((addr >> 8) & 0xFF));                               \
  (txbuf)[1] = ((uint8_t)(addr & 0xFF));                                      \
}
    int i = 0;
    uint8_t data[1];
    uint8_t localtxbuf[2];
    outln("EEPROM Read...");
    EepromOpen(&Efs);
    chFileStreamSeek(&Efs, 0);
    chThdSleepMilliseconds(50);
    for (i=0; i<128; i++) {
      //data = EepromReadByte(&Efs);
      eeprom_split_addr(localtxbuf, i);
outln("AQUI1");
      i2cAcquireBus(&I2CD1);
outln("AQUI2");
      i2cMasterTransmit(&I2CD1, 0b1010000, localtxbuf, 2, data, 1);
outln("AQUI3");
	  i2cReleaseBus(&I2CD1);
outln("AQUI4");
      outInt(data[0], 100);
      out(" ");
    }
    outln("");
    chFileStreamClose(&Efs);
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

  palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  /*
   * BMP085 Initialization
   */
  chThdSleepMilliseconds(20); //(datasheet 10 ms)
  i2cAcquireBus(&I2CD1);
  BMP085_cfg.ac1 = bmp085ReadShort(0xaa);
  BMP085_cfg.ac2 = bmp085ReadShort(0xac);
  BMP085_cfg.ac3 = bmp085ReadShort(0xae);
  BMP085_cfg.ac4 = bmp085ReadShort(0xb0);
  BMP085_cfg.ac5 = bmp085ReadShort(0xb2);
  BMP085_cfg.ac6 = bmp085ReadShort(0xb4);
  BMP085_cfg.b1 = bmp085ReadShort(0xb6);
  BMP085_cfg.b2 = bmp085ReadShort(0xb8);
  BMP085_cfg.mb = bmp085ReadShort(0xba);
  BMP085_cfg.mc = bmp085ReadShort(0xbc);
  BMP085_cfg.md = bmp085ReadShort(0xbe);
  i2cReleaseBus(&I2CD1);

  /*
   * EEPROM Write Test
   */
  int i = 0;
  outln("EEPROM Write...");
  EepromOpen(&Efs);
  chFileStreamSeek(&Efs, 0);
  for (i=0; i<128; i++) {
        if (EepromWriteByte(&Efs, (uint8_t)i) != sizeof(uint8_t))
      outln("write failed");
  }
  chFileStreamClose(&Efs);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Create the BMP085 thread.
   */
  //chThdCreateStatic(wa_BMP085_Thread, sizeof(wa_BMP085_Thread), LOWPRIO, BMP085_Thread, NULL);

  return 0;
}
