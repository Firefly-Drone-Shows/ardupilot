#include "AC_Dancing_LED.h"
#include "I2CLed.h"
#include "PWM3Led.h"

const AP_Param::GroupInfo AC_Dancing_LED::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Dancing Led type
    // @Description: Dancing Led type
    // @Values: 0:PWM_3,1:I2C_LED
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE",    0, AC_Dancing_LED, _type, LED_TYPE_PWM3),

    // @Param: I2C_ADR
    // @DisplayName: I2C Led address
    // @Description: I2C Led address
    // @Values: 8-bit address
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("I2C_ADR", 1, AC_Dancing_LED, _i2c_address, DEFAULT_DANCING_LED_I2C_ADDRESS),


    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AC_Dancing_LED::AC_Dancing_LED() : _initialized(false) {
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Dancing_LED::init() {

    if (_initialized)
        return;

    switch(_type) {
    case LED_TYPE_I2C :
        _backend = new I2CLed(*this);
        break;

    case LED_TYPE_PWM3 :
        _backend = new PWM3Led(*this);
        break;

    default:
        break;
    }

    if (_backend != nullptr) {
        _initialized = _backend -> init();
    }

    _start_time_ms = 0;

    // Switch off lighs on init
    set_color(0, 0, 0);
}


void AC_Dancing_LED::update() {

    if (!_initialized)
        return;

    if (_repeat == 0 && _servos_need_to_reinit) {
        _backend -> set_color(_initial_color_r, _initial_color_g, _initial_color_b);
        _servos_need_to_reinit = false;
    }

    if (_repeat == 0 || (AP_HAL::millis() - _start_time_ms) < _delay_ms) {
        return;
    }

    _start_time_ms = AP_HAL::millis();

    if (_repeat & 1) {
        _backend -> set_color(0, 0, 0);
    } else {
        _backend -> set_color(_current_color_r, _current_color_g, _current_color_b);
    }

    if (_repeat > 0) {
        _repeat--;
    } else {
        // toggle bottom bit so servos flip in value
        _repeat ^= 1;
    }
}

bool AC_Dancing_LED::set_color(uint8_t r, uint8_t g, uint8_t b, uint16_t duration, uint8_t blink_rate) {

    if (!_initialized)
        return false;

    // if _blink_rate <= 0 and _duration <= 0: simple set_servo forever
    if (blink_rate <= 0 && duration <= 0) {
        _repeat = 0;
        _servos_need_to_reinit = false;
        _backend -> set_color(r, g, b);
        return true;
    }

    // if no current bliniking process is active
    // save current servos to reinit them
    // after end of process
    if (_repeat == 0) {
        _backend -> get_color(&_initial_color_r, &_initial_color_g, &_initial_color_b);
        _servos_need_to_reinit = true;
    }

    _start_time_ms  = 0;
    _delay_ms = 0;
    _repeat = 0;

    // if _blink_rate <= 0 and _duration > 0: no blinking with duration = delay_ms
    if (blink_rate <= 0 && duration > 0) {
        _delay_ms = duration;
        _repeat = 2;
    }

    // if _blink_rate > 0 and _duration > 0: blinking with duration
    if (blink_rate > 0 && duration > 0) {
        _delay_ms = 1000 / blink_rate;
        _repeat = (duration / 1000) * blink_rate;
    }

    // if _blink_rate > 0 and _duration <= 0: blinking forever
    if (blink_rate > 0 && duration <= 0) {
        _delay_ms = 1000 / blink_rate;
        _repeat = -1;
        _servos_need_to_reinit = false;
    }

    _current_color_r = r;
    _current_color_g = g;
    _current_color_b = b;

    update();

    return true;
}

void AC_Dancing_LED::handle_led_control(mavlink_message_t *msg) {

}

Dancing_LED_Backend::Dancing_LED_Backend(AC_Dancing_LED &led):
    _led(led),
    _healthy(false)
{

}

bool Dancing_LED_Backend::init()
{
    _healthy = hw_init();

    return _healthy;
}


bool Dancing_LED_Backend::set_color(uint8_t r, uint8_t g, uint8_t b) {

    if (!_healthy)
        return false;

    if (r != _red_curr || g != _green_curr || b != _blue_curr ) {
        _red_curr = r;
        _green_curr = g;
        _blue_curr = b;

        return hw_set_rgb(r, g, b);
    }

    return false;
}

void Dancing_LED_Backend::get_color(uint8_t *r, uint8_t *g, uint8_t *b) {
    hw_get_rgb(r, g, b);
}

