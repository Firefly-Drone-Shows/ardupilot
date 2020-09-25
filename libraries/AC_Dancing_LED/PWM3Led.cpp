#include "PWM3Led.h"

extern const AP_HAL::HAL& hal;

PWM3Led::PWM3Led(AC_Dancing_LED &led) : Dancing_LED_Backend(led){}

bool PWM3Led::hw_init() {

    uint8_t _channel_r, _channel_g, _channel_b;

    if (!SRV_Channels::find_channel (SRV_Channel::k_color_r, _channel_r)) { return false; }
    if (!SRV_Channels::find_channel (SRV_Channel::k_color_g, _channel_g)) { return false; }
    if (!SRV_Channels::find_channel (SRV_Channel::k_color_b, _channel_b)) { return false; }

    _r = SRV_Channels::srv_channel(_channel_r);
    _g = SRV_Channels::srv_channel(_channel_g);
    _b = SRV_Channels::srv_channel(_channel_b);

    if (_r == nullptr || _g == nullptr || _b == nullptr) {
        return false;
    }

    return true;
}

bool PWM3Led::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) {

    uint16_t r_servo_pwm = (uint16_t)(2500 * r / 250);
    uint16_t g_servo_pwm = (uint16_t)(2500 * g / 250);
    uint16_t b_servo_pwm = (uint16_t)(2500 * b / 250);

    if (r_servo_pwm > 2500) {r_servo_pwm = 2500;}
    if (g_servo_pwm > 2500) {g_servo_pwm = 2500;}
    if (b_servo_pwm > 2500) {b_servo_pwm = 2500;}

    SRV_Channels::set_output_pwm (SRV_Channel::k_color_r, r_servo_pwm);
    SRV_Channels::set_output_pwm (SRV_Channel::k_color_g, g_servo_pwm);
    SRV_Channels::set_output_pwm (SRV_Channel::k_color_b, b_servo_pwm);

    return true;
}

void  PWM3Led::hw_get_rgb(uint8_t *r, uint8_t *g, uint8_t *b) {
    // it is not as accurate as we want it to be. :(
    *r = round((_r->get_output_pwm() * 250) / 2500);
    *g = round((_g->get_output_pwm() * 250) / 2500);
    *b = round((_b->get_output_pwm() * 250) / 2500);
}

