#include <Servo.h>

#define pin_rx_tr 15
#define pin_esc_out 0
#define pin_led_status 25 
#define pin_led_armed 16

#define pwm_in_min 1000
#define pwm_in_max 2000
#define pwm_in_range (pwm_in_max - pwm_in_min)

#define esc_min 1000
#define esc_max 2000
#define esc_range (esc_max - esc_min)

#define dedband_us 30
#define slew_rate_us 10

#define filter_alpha 0.15f

#define failsafe_timeout_ms 250
#define arming_hold_ms 2000
#define arm_threshold_us 2000
#define  1000