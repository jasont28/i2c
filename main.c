

#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board_io.h"
#include "common_macros.h"
#include "gpio.h"
#include "gpio_lab.h"

#include "periodic_scheduler.h"
#include "semphr.h"
#include "sj2_cli.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "pwm1.h"
#include "ssp2_lab.h"
//#include "uart.h"
#include "delay.h"
#include "lpc_peripherals.h"
#include "uart_lab.h"

#include "acceleration.h"
#include "event_groups.h"

#include "i2c_slave_init.h"

static void create_blinky_tasks(void);
static void create_uart_task(void);
static void blink_task(void *params);
static void uart_task(void *params);

//==========================================================
//           B O A R D   S E T U P   M O D E
//==========================================================
// Reference code from i2c.c file
#define I2C__ENABLE_TWO_BOARDS 0
#define I2C__ENABLE_SLAVE 0 // Only applies for two boards
#if I2C__ENABLE_TWO_BOARDS
#define i2c_slave I2C__2
#if I2C__ENABLE_SLAVE
#define I2C__SETUP()                                                                                                   \
  do {                                                                                                                 \
    TWO_BOARDS_SETUP();                                                                                                \
  } while (0)
#else
#define I2C__SETUP()
#endif
#else
#define i2c_slave I2C__1
#define I2C__SETUP()                                                                                                   \
  do {                                                                                                                 \
    ONE_BOARD_SETUP();                                                                                                 \
  } while (0)
#endif
//==========================================================

/*==========================================================
 *              S L A V E   F U N C T I O N S
 *==========================================================
 */
static volatile uint8_t slave_memory[256];
bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory) {
  if (memory_index < sizeof(slave_memory)) {
    // TODO: Read the data from slave_memory[memory_index] to *memory pointer
    // TODO: return true if all is well (memory index is within bounds)
    *(memory) = slave_memory[memory_index];
    fprintf(stderr, "reading into mem: %i %i\n", memory_index, *memory);
    return true;
  }
  return false;
}

bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value) {
  if (memory_index < sizeof(slave_memory)) {
    slave_memory[memory_index] = memory_value;
    fprintf(stderr, "writing into mem: %i %i\n", memory_index, memory_value);
    // TODO: Write the memory_value at slave_memory[memory_index]
    // TODO: return true if memory_index is within bounds
    return true;
  }
  return false;
}

/*==========================================================
 *        P I N   C O N F I G   F U N C T I O N S
 *==========================================================
 */

//==========PLEASE NOTE: THESE FUNCTIONS ARE ONLY USE FOR SINGLE BOARD=============
static void pin_config__as_open_drain_i2c__1(void) {
  LPC_IOCON->P0_0 |= (1 << 10);
  LPC_IOCON->P0_1 |= (1 << 10);
}
static void pin_config_as_i2c__1(void) {
  pin_config__as_open_drain_i2c__1(); // Required according to user manual pg. 616
  LPC_IOCON->P0_0 &= ~0b111;
  LPC_IOCON->P0_1 &= ~0b111;
  LPC_IOCON->P0_0 |= 0b11;
  LPC_IOCON->P0_1 |= 0b11;
}
//================================================================================

static void pin_config_as_PWM1(void) {
  LPC_IOCON->P2_0 &= ~0b111;
  LPC_IOCON->P2_1 &= ~0b111;
  LPC_IOCON->P2_2 &= ~0b111;
  LPC_IOCON->P2_0 |= 0b001;
  LPC_IOCON->P2_1 |= 0b001;
  LPC_IOCON->P2_2 |= 0b001;
}
/*==========================================================
 *               L E D    F U N C T I O N S
 *==========================================================
 */
static void set__RGB_LED_OFF(void) {
  for (uint8_t num = 0; num < 3; num++)
    pwm1__set_duty_cycle(num, 0);
}

static void init__RBG_LED(void) {
  pwm1__init_single_edge(1000);
  set__RGB_LED_OFF();
}

static void i2c__slave_LED_logic(void) {
  for (uint8_t address = 0; address < 3; address++) {
    pwm1__set_duty_cycle(address, slave_memory[address] * 100 / (sizeof(slave_memory) - 1));
  }
}
/*==========================================================
 *              T A S K    F U N C T I O N
 *==========================================================
 */
#define ENABLE_print_in_main 1 // Set to non-zero to enable print in main
#define Ticks_to_Delay_in_ms 500U

static void i2c__loop_task(void *p) {
  while (1) {
#if ENABLE_print_in_main
    fprintf(stderr, "Ticks: %i, In main\n", xTaskGetTickCount());
#endif
    i2c__slave_LED_logic();
    vTaskDelay(Ticks_to_Delay_in_ms);
  }
}
//=========================PART 0============================

/*==========================================================
 *         R E G U L A R   F U N C T I O N S
 *==========================================================
 */
//===============================Part 3a===========================================
#define slave_register_address 0xFE // Slave address

static void i2c_slave_enable(i2c_e i2c_) { i2c2__slave_init(i2c_, slave_register_address); }

//===============================Part 3c===========================================
#define i2c_speed_hz UINT32_C(400) * 1000 // Retreived from startup code

#define stack_size 1024 / sizeof(void *) // Stack_size allocated for task "i2c_loop_task"
#define I2C__DEBUG_SLAVE_DETECTION 1     // Set to non-zero to enable detection
/*Please note:
 * Enabling debug in i2c.c file won't recognize your desire slave address from startup
 * b/c the slave haven't been initialize
 */

void ONE_BOARD_SETUP(void) {
  pin_config_as_PWM1();
  pin_config_as_i2c__1();
  i2c__initialize(I2C__1, i2c_speed_hz, clock__get_peripheral_clock_hz());
  i2c_slave_enable(i2c_slave);
#if (I2C__DEBUG_SLAVE_DETECTION)
  fprintf(stderr, "%s at 0x2%02X\n",
          (i2c__detect(I2C__2, slave_register_address) ? "Slave SUCCESSFULLY detected" : "Slave FAILED to Detect"),
          slave_register_address);
#endif
  init__RBG_LED();
  xTaskCreate(i2c__loop_task, "i2c_loop_task", stack_size, NULL, 1, NULL);
}

void TWO_BOARDS_SETUP(void) {
  pin_config_as_PWM1();
  i2c_slave_enable(i2c_slave);
  init__RBG_LED();
  xTaskCreate(i2c__loop_task, "i2c_loop_task", stack_size, NULL, 1, NULL);
}
//==========================================================

int main(void) {
  // create_blinky_tasks();
  create_uart_task();
  puts("Starting RTOS");
  I2C__SETUP();
  vTaskStartScheduler(); // Ths function never returns unless RTOS scheduler runs out of memory and fails
  return 0;
}

static void create_blinky_tasks(void) {
  /**
   * Use '#if (1)' if you wish to observe how two tasks can blink LEDs
   * Use '#if (0)' if you wish to use the 'periodic_scheduler.h' that will spawn 4 periodic tasks, one for each LED
   */
#if (1)
  // These variables should not go out of scope because the 'blink_task' will reference this memory
  static gpio_s led0, led1;

  led0 = board_io__get_led0();
  led1 = board_io__get_led1();

  xTaskCreate(blink_task, "led0", configMINIMAL_STACK_SIZE, (void *)&led0, PRIORITY_LOW, NULL);
  xTaskCreate(blink_task, "led1", configMINIMAL_STACK_SIZE, (void *)&led1, PRIORITY_LOW, NULL);
#else
  const bool run_1000hz = true;
  const size_t stack_size_bytes = 2048 / sizeof(void *); // RTOS stack size is in terms of 32-bits for ARM M4 32-bit CPU
  periodic_scheduler__initialize(stack_size_bytes, !run_1000hz); // Assuming we do not need the high rate 1000Hz task
  UNUSED(blink_task);
#endif
}

static void create_uart_task(void) {
  // It is advised to either run the uart_task, or the SJ2 command-line (CLI), but not both
  // Change '#if (0)' to '#if (1)' and vice versa to try it out
#if (0)
  // printf() takes more stack space, size this tasks' stack higher
  xTaskCreate(uart_task, "uart", (512U * 8) / sizeof(void *), NULL, PRIORITY_LOW, NULL);
#else
  sj2_cli__init();
  UNUSED(uart_task); // uart_task is un-used in if we are doing cli init()
#endif
}

static void blink_task(void *params) {
  const gpio_s led = *((gpio_s *)params); // Parameter was input while calling xTaskCreate()

  // Warning: This task starts with very minimal stack, so do not use printf() API here to avoid stack overflow
  while (true) {
    gpio__toggle(led);
    vTaskDelay(500);
  }
}

// This sends periodic messages over printf() which uses system_calls.c to send them to UART0
static void uart_task(void *params) {
  TickType_t previous_tick = 0;
  TickType_t ticks = 0;

  while (true) {
    // This loop will repeat at precise task delay, even if the logic below takes variable amount of ticks
    vTaskDelayUntil(&previous_tick, 2000);

    /* Calls to fprintf(stderr, ...) uses polled UART driver, so this entire output will be fully
     * sent out before this function returns. See system_calls.c for actual implementation.
     *
     * Use this style print for:
     *  - Interrupts because you cannot use printf() inside an ISR
     *    This is because regular printf() leads down to xQueueSend() that might block
     *    but you cannot block inside an ISR hence the system might crash
     *  - During debugging in case system crashes before all output of printf() is sent
     */
    ticks = xTaskGetTickCount();
    fprintf(stderr, "%u: This is a polled version of printf used for debugging ... finished in", (unsigned)ticks);
    fprintf(stderr, " %lu ticks\n", (xTaskGetTickCount() - ticks));

    /* This deposits data to an outgoing queue and doesn't block the CPU
     * Data will be sent later, but this function would return earlier
     */
    ticks = xTaskGetTickCount();
    printf("This is a more efficient printf ... finished in");
    printf(" %lu ticks\n\n", (xTaskGetTickCount() - ticks));
  }
}
