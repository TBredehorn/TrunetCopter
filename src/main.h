#ifndef MAIN_H_
#define MAIN_H_

#define DEBUG
#define SERIAL_DEBUG SD1

#define I2C_BUS I2CD1
#define MAX_I2C_BUF 14

#define I2C_THREADS_PRIO          (NORMALPRIO + 2)
#define TIMEKEEPER_THREAD_PRIO    (I2C_THREADS_PRIO - 1)

#define GYRO_CAL_FLAG        (1UL << 0)
#define ACCEL_CAL_FLAG       (1UL << 1)
#define MAG_CAL_FLAG         (1UL << 2)
#define EEPROM_FAILED_FLAG   (1UL << 3)
#define POSTAGE_FAILED_FLAG  (1UL << 4)
#define I2C_RESTARTED_FLAG   (1UL << 5)

#define setGlobalFlag(flag)   {chSysLock(); GlobalFlags |= (flag); chSysUnlock();}
#define clearGlobalFlag(flag) {chSysLock(); GlobalFlags &= (~(flag)); chSysUnlock();}

#if (CH_FREQUENCY) >= 1000
#define TIME_BOOT_MS ((chTimeNow()) / ((CH_FREQUENCY) / 1000))
#endif

#define EEPROM_SETTINGS_START    8192
#define EEPROM_SETTINGS_SIZE     4096
#define EEPROM_SETTINGS_FINISH   (EEPROM_SETTINGS_START + EEPROM_SETTINGS_SIZE)

#endif /* MAIN_H_ */
