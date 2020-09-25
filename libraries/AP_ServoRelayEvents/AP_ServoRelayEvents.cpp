/*
   This program is free software: you can redistribute it and/or modify
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
/*
 *   AP_ServoRelayEvents - handle servo and relay MAVLink events
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_ServoRelayEvents.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

bool AP_ServoRelayEvents::do_set_servo(uint8_t _channel, uint16_t pwm)
{
    if (!(mask & 1U<<(_channel-1))) {
        // not allowed
        return false;
    }
    
    
    if (((type == EVENT_TYPE_SERVO) &&  channel == _channel) || 
         type == EVENT_TYPE_RGB) {
        // cancel previous repeat
        repeat = 0;
    }
    SRV_Channel *c = SRV_Channels::srv_channel(_channel-1);
    if (c == nullptr) {
        return false;
    }
    c->set_output_pwm(pwm);
    return true;
}

bool AP_ServoRelayEvents::do_set_relay(uint8_t relay_num, uint8_t state)
{
    if (!relay.enabled(relay_num)) {
        return false;
    }
    if (type == EVENT_TYPE_RELAY && 
        channel == relay_num) {
        // cancel previous repeat
        repeat = 0;
    }
    if (state == 1) {
        relay.on(relay_num);
    } else if (state == 0) {
        relay.off(relay_num);
    } else {
        relay.toggle(relay_num);
    }
    return true;
}

bool AP_ServoRelayEvents::do_repeat_servo(uint8_t _channel, uint16_t _servo_value, 
                                          int16_t _repeat, uint16_t _delay_ms)
{
    if (!(mask & 1U<<(_channel-1))) {
        // not allowed
        return false;
    }
    channel = _channel;
    type = EVENT_TYPE_SERVO;

    start_time_ms  = 0;
    delay_ms    = _delay_ms / 2;
    repeat      = _repeat * 2;
    servo_value = _servo_value;
    update_events();
    return true;
}

bool AP_ServoRelayEvents::do_repeat_rgb(uint16_t _servo_value_r, uint16_t _servo_value_g, uint16_t _servo_value_b,
                       uint16_t _duration, uint8_t _blink_rate)
{
    
    // if _blink_rate <= 0 and _duration <= 0: simple set_servo forever
    // if _blink_rate <= 0 and _duration > 0: no blinking with duration = delay_ms
    // if _blink_rate > 0 and _duration > 0: blinking with duration
    // if _blink_rate > 0 and _duration <= 0: blinking forever
    
    uint8_t _channel_r, _channel_g, _channel_b;
    
    if (!SRV_Channels::find_channel (SRV_Channel::k_color_r, _channel_r)) { return false; }
    if (!SRV_Channels::find_channel (SRV_Channel::k_color_g, _channel_g)) { return false; }
    if (!SRV_Channels::find_channel (SRV_Channel::k_color_b, _channel_b)) { return false; }

    hal.console->printf(">>> - %d %d %d\n",
                        _channel_r, _channel_g, _channel_b);   
 
    if (!((mask & 1U<<(_channel_r)) &&
          (mask & 1U<<(_channel_g)) &&
          (mask & 1U<<(_channel_b)))) {
        // not allowed
        return false;
    }
    
    SRV_Channel *r = SRV_Channels::srv_channel(_channel_r);
    SRV_Channel *g = SRV_Channels::srv_channel(_channel_g);
    SRV_Channel *b = SRV_Channels::srv_channel(_channel_b);

    if (r == nullptr || g == nullptr || b == nullptr) {
        return false;
    }
    
    channel_r = _channel_r;
    channel_g = _channel_g;
    channel_b = _channel_b;
    
    type = EVENT_TYPE_RGB;

    // if _blink_rate <= 0 and _duration <= 0: simple set_servo forever
    if (_blink_rate <= 0 && _duration <= 0) {
        repeat = 0;
        _servos_need_to_reinit = false;
        //do_set_servo count shannels from 1. So +1 to each. 
        do_set_servo(channel_r + 1, _servo_value_r);
        do_set_servo(channel_g + 1, _servo_value_g);
        do_set_servo(channel_b + 1, _servo_value_b);
        return true;
    }
    
    // if no current bliniking process is active
    // save current servos to reinit them 
    // after end of process
    if (repeat == 0) {
        _initial_servo_value_r = r->get_output_pwm();
        _initial_servo_value_g = g->get_output_pwm();
        _initial_servo_value_b = b->get_output_pwm();

        _servos_need_to_reinit = true;
    }

    start_time_ms  = 0;
    delay_ms = 0;
    repeat = 0;

    // if _blink_rate <= 0 and _duration > 0: no blinking with duration = delay_ms
    if (_blink_rate <= 0 && _duration > 0) {
        delay_ms = _duration;
        repeat = 2;
    }

    // if _blink_rate > 0 and _duration > 0: blinking with duration
    if (_blink_rate > 0 && _duration > 0) {
        delay_ms = 1000 / _blink_rate;
        repeat = (_duration / 1000) * _blink_rate;
    }
    
    // if _blink_rate > 0 and _duration <= 0: blinking forever
    if (_blink_rate > 0 && _duration <= 0) {
        delay_ms = 1000 / _blink_rate;
        repeat = -1;
        _servos_need_to_reinit = false;
    }

    servo_value_r = _servo_value_r;
    servo_value_g = _servo_value_g;
    servo_value_b = _servo_value_b;

    update_events();
    return true;
}

bool AP_ServoRelayEvents::do_repeat_relay(uint8_t relay_num, int16_t _repeat, uint32_t _delay_ms)
{
    if (!relay.enabled(relay_num)) {
        return false;
    }
    type = EVENT_TYPE_RELAY;
    channel = relay_num;
    start_time_ms  = 0;
    delay_ms        = _delay_ms/2; // half cycle time
    repeat          = _repeat*2;  // number of full cycles
    update_events();
    return true;
}


/*
  update state for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
*/
void AP_ServoRelayEvents::update_events(void)
{
    if (repeat ==0 && _servos_need_to_reinit) {
        SRV_Channel *r = SRV_Channels::srv_channel(channel_r);
        SRV_Channel *g = SRV_Channels::srv_channel(channel_g);
        SRV_Channel *b = SRV_Channels::srv_channel(channel_b);
        if (r != nullptr) {
            r->set_output_pwm(_initial_servo_value_r);
        }
        if (g != nullptr) {
            g->set_output_pwm(_initial_servo_value_g);
        }
        if (b != nullptr) {
            b->set_output_pwm(_initial_servo_value_b);
        }

        _servos_need_to_reinit = false;
    }

    if (repeat == 0 || (AP_HAL::millis() - start_time_ms) < delay_ms) {
        return;
    }

    start_time_ms = AP_HAL::millis();

    switch (type) {
    case EVENT_TYPE_SERVO: {
        SRV_Channel *c = SRV_Channels::srv_channel(channel-1);
        if (c != nullptr) {
            if (repeat & 1) {
                c->set_output_pwm(c->get_trim());
            } else {
                c->set_output_pwm(servo_value);
            }
        }
        break;
    }

    case EVENT_TYPE_RGB: {
        SRV_Channel *r = SRV_Channels::srv_channel(channel_r);
        SRV_Channel *g = SRV_Channels::srv_channel(channel_g);
        SRV_Channel *b = SRV_Channels::srv_channel(channel_b);
        if (r != nullptr) {
            if (repeat & 1) {
                r->set_output_pwm(r->get_trim());
            } else {
                r->set_output_pwm(servo_value_r);
            }
        }
        if (g != nullptr) {
            if (repeat & 1) {
                g->set_output_pwm(g->get_trim());
            } else {
                g->set_output_pwm(servo_value_g);
            }
        }
        if (b != nullptr) {
            if (repeat & 1) {
                b->set_output_pwm(b->get_trim());
            } else {
                b->set_output_pwm(servo_value_b);
            }
        }
        break;
    }
        
    case EVENT_TYPE_RELAY:
        relay.toggle(channel);
        break;
    }
    
    if (repeat > 0) {
        repeat--;
    } else {
        // toggle bottom bit so servos flip in value
        repeat ^= 1;
    }
}
