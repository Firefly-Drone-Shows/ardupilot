/*
 * AP_ServoRelayEvent.h
 *
 * handle DO_SET_SERVO, DO_REPEAT_SERVO, DO_SET_RELAY and
 * DO_REPEAT_RELAY commands
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay.h>

class AP_ServoRelayEvents {
public:
    AP_ServoRelayEvents(AP_Relay &_relay) : 
    relay(_relay),
    mask(0),
    type(EVENT_TYPE_RELAY),
    start_time_ms(0),
    delay_ms(0),
    repeat(0),
    channel(0),
    servo_value(0),
    _initial_servo_value_r(0),
    _initial_servo_value_g(0),
    _initial_servo_value_b(0),
    _servos_need_to_reinit(false)
    {}

    // set allowed servo channel mask
    void set_channel_mask(uint16_t _mask) { mask = _mask; }

    bool do_set_servo(uint8_t channel, uint16_t pwm);
    bool do_set_relay(uint8_t relay_num, uint8_t state);
    bool do_repeat_servo(uint8_t channel, uint16_t servo_value, int16_t repeat, uint16_t delay_time_ms);
    bool do_repeat_rgb(uint16_t servo_value_r, uint16_t servo_value_g, uint16_t servo_value_b,
                       uint16_t duration, uint8_t blink_rate);
    bool do_repeat_relay(uint8_t relay_num, int16_t count, uint32_t period_ms);
    void update_events(void);

private:
    AP_Relay &relay;
    uint16_t mask;

    // event control state
    enum event_type { 
        EVENT_TYPE_RELAY=0,
        EVENT_TYPE_SERVO=1,
        EVENT_TYPE_RGB=2
    };

    enum event_type type;

	// when the event was started in ms
    uint32_t start_time_ms;

	// how long to delay the next firing of event in millis
    uint16_t delay_ms;

	// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
    int16_t repeat;

    // RC channel for servos, relay number for relays
    uint8_t channel;

	// PWM for servos
	uint16_t servo_value;

    // RGB channels
    uint8_t channel_r;
    uint8_t channel_g;
    uint8_t channel_b;

    // PWM for rgb
	uint16_t servo_value_r;
    uint16_t servo_value_g;
    uint16_t servo_value_b;

    // initial values to return when 
    // blink is over
    uint16_t _initial_servo_value_r;
    uint16_t _initial_servo_value_g;
    uint16_t _initial_servo_value_b;

    bool _servos_need_to_reinit;


};
