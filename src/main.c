/* ChibiOS includes */
#include "ch.h"
#include "hal.h"
#include "test.h"

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  while (TRUE) {
    palClearPad(GPIOB, 1);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, 1);
    chThdSleepMilliseconds(500);
  }
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

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   * GPIOB,8 is the button on the Maple Mini
   */
  while (TRUE) {
	if(palReadPad(GPIOB, 8))
      TestThread(&SD1);
    chThdSleepMilliseconds(500);
  }
}
