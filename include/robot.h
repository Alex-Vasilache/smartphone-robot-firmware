#ifndef ROBOT_
#define ROBOT_

#include "hardware/i2c.h"
#include "serial_comm_manager.h"

#define CONVERSION_FACTOR _u(3).3f / (1 << 12)
#define GPIO_WIRELESS_AVAILABLE _u(4) // GPIO4
#define GPIO_OTG _u(5) // GPIO5
#define BATTERY_CHARGER_INTERRUPT_PIN _u(6) // GPIO6
#define SN74AHC125RGYR_GPIO1 _u(8) // GPIO8 The buffer in the DRV8830 sheet
#define SN74AHC125RGYR_GPIO2 _u(22) // GPIO22 The buffer in the rp2040 sheet
#define MAX77958_INTB _u(7) // GPIO7
#define BQ27742_G1_INTERRUPT_PIN _u(20) // GPIO20, RC2_3V3 from BQ27742 RC2 level shifter

#define DRV8830_FAULT1 _u(10) // GPIO10
#define DRV8830_FAULT2 _u(11) // GPIO11

// I2C defines
// Use I2C0 on GPIO0 (SDA) and GPIO1 (SCL) running at 400KHz.
#define I2C_SDA0 _u(0)
#define I2C_SCL0 _u(1)
#define I2C_SDA1 _u(2)
#define I2C_SCL1 _u(3)
// 20 ms, not 1 s. A transaction on this bus is a few bytes at 400 kHz, i.e.
// well under a millisecond; the BQ27742 may clock-stretch, but not for tens of
// milliseconds. With the old 1 s value a single NAK under motor noise froze
// the whole command loop -- measured from the phone on 2026-09-14, the reply
// to SET_MOTOR_LEVEL took 1.09 s (one timeout plus one normal service) every
// few seconds while the motors ran, and the wheels held a stale command for
// the duration. Three failures at 1 s each meant 3 s and then assert().
#define I2C_TIMEOUT _u(20000)

#define ADC0 _u(26)

#define WIRELESS_CHG_EN _u(9) // WRM483265-10F5-12V-G (U1) enable pin via GPIO9
#define WIRELESS_CHG_AVAILABLE _u(4)
#define WIRELESS_CHG_VRECT _u(27)
#define USB_VOLTAGE _u(26)
#define CHARGER_INT _u(6)
#define MODE _u(0x16) // CHG_CNFG_00

// Quad encoder pins
#define ENCODER_1_CHANNEL_A _u(12)
#define ENCODER_1_CHANNEL_B _u(13)
#define ENCODER_2_CHANNEL_A _u(14)
#define ENCODER_2_CHANNEL_B _u(15)

#define LED_EN_PIN _u(21) // GPIO21

// How often SET_MOTOR_LEVEL replies re-read the charger and fuel gauge. The
// values move on a timescale of seconds; the motor loop runs at 50 Hz.
#define TELEMETRY_PERIOD_US _u(500000)

#define MAX_RETRIES 3
#define RETRY_DELAY 10

typedef int32_t (*entry_func)(int32_t);
typedef struct
{
    entry_func func;
    int32_t data;
} queue_entry_t;

void on_start();
void i2c_start();
void bq27742_g1_init(uint gpio_interrupt);
void max77642_init();
void max77857_init();
void sn74ahc125rgyr_init();
void quad_encoders_init();
void blink_led(uint8_t blinkCnt, int onTime, int offTime);
void sample_adc_inputs();
void init_queues();
void i2c_read_error_handling(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);
void i2c_write_error_handling(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
void call_queue_try_add(entry_func func, int32_t arg);
bool call_queue_try_add_nonblocking(entry_func func, int32_t arg);
void results_queue_try_add(void *func, int32_t arg);
void set_motor_levels(RP2040_STATE *state);
void get_state(RP2040_STATE* state);
void get_battery_state(RP2040_STATE* state);

#endif
