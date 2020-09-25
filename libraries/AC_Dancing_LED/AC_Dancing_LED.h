#pragma once

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define LED_TYPE_PWM3 0
#define LED_TYPE_I2C  1

#define DEFAULT_DANCING_LED_I2C_ADDRESS 0x66


class Dancing_LED_Backend;

class AC_Dancing_LED {

public:

    friend class Dancing_LED_Backend;
    friend class I2CLed;
    friend class PWM3Led;

    static const struct AP_Param::GroupInfo var_info[];

    AC_Dancing_LED();

    void init();

    // 50hz update loop
    void update();

    bool set_color(uint8_t r, uint8_t g, uint8_t b, uint16_t duration, uint8_t blink_rate);

    bool set_color(uint8_t r, uint8_t g, uint8_t b) { return set_color(r, g, b, 0, 0); }

    void handle_led_control(mavlink_message_t *msg);

protected :
     AP_Int8 _type;
     AP_Int8 _i2c_address;

     Dancing_LED_Backend *_backend;

private:
     bool _initialized;

     // how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
     int16_t  _repeat;
     uint32_t _start_time_ms;
     uint32_t _delay_ms;
     bool _servos_need_to_reinit;
     uint8_t _initial_color_r, _initial_color_g, _initial_color_b;
     uint8_t _current_color_r, _current_color_g, _current_color_b;
};


class Dancing_LED_Backend
{
public:
    Dancing_LED_Backend(AC_Dancing_LED &led);

    bool set_color(uint8_t r, uint8_t g, uint8_t b);

    void get_color(uint8_t *r, uint8_t *g, uint8_t *b);

    bool init();
protected:
    // methods implemented in hardware specific classes
    virtual bool hw_init(void) = 0;

    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

    virtual void hw_get_rgb(uint8_t *r, uint8_t *g, uint8_t *b) = 0;
    
    uint8_t _red_curr, _green_curr, _blue_curr;

    AC_Dancing_LED _led;
    bool _healthy;
};


