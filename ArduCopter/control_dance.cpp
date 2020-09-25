#include "Copter.h"

#include <stdio.h>

/*
 * Init and run calls for dance flight mode
 */

// dance_init - initialise dance controller flight mode
bool Copter::dance_init(bool ignore_checks)
{
    int16_t is_ready = dance_nav->ready_to_dance();
    if (is_ready != 0) {
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "DANCE: Not ready to dance! Code (%d)", is_ready);
        return false;
    }
    
    if (position_ok() || ignore_checks) {
        // initialize speeds and accelerations
        pos_control->set_speed_xy(wp_nav->get_speed_xy());
        pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
        pos_control->set_jerk_xy_to_default();
        //pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        //pos_control->set_accel_z(g.pilot_accel_z);
	pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
	pos_control->set_accel_z(wp_nav->get_accel_z());

        // initialise dance controller 
        dance_nav->init();

        dance_takeoff_alt = 0;
        dance_alt_delta = 50; // 50 cm
        dance_mode = Dance_WaitingToTakeoff;

        return true;
    } 
    else
    {
        return false;
    }
}

void Copter::dance_run()
{
    // call the correct auto controller
    switch (dance_mode) {

    case Dance_WaitingToTakeoff:
        dance_takeoff_start();
        break;

    case Dance_TakeOff:
        dance_takeoff_run();
        break;

    case Dance_Dance:
        dance_dance_run();
        break;

    case Dance_Land:
        dance_land_run();
    }
}


void Copter::dance_takeoff_start() 
{
    // waiting for the time to takeoff
    if (dance_nav->takeoff_is_allowed()) {
        dance_takeoff_alt = dance_nav->get_takeoff_altitude();
        if (guided_takeoff_start(dance_takeoff_alt)) {
                set_auto_armed(true);
                dance_mode = Dance_TakeOff;
        }
    }
    return ; 
}

//
//
void Copter::dance_takeoff_run() {
    if (fabsf(current_loc.alt - dance_takeoff_alt) > dance_alt_delta) {
        guided_takeoff_run();
    } else {
        dance_mode = Dance_Dance;
    }
}

//
//
void Copter::dance_land_run() {
    if (dance_nav->return_to_home_position()) {
        set_mode(RTL, MODE_REASON_MISSION_END);
    } else {
        set_mode(LAND, MODE_REASON_MISSION_END);
    }
}


// dance_dance_run - runs the dance flight mode
// should be called at 100hz or more
void Copter::dance_dance_run()
{
    float target_climb_rate = 0;

    // initialize speeds and accelerations
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_jerk_xy_to_default();
    //pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    //pos_control->set_accel_z(g.pilot_accel_z);
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        // To-Do: add some initialisation of position controllers
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control->set_alt_target_to_current_alt();

        set_throttle_takeoff();
        
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run dance controller
    dance_nav->update();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(dance_nav->get_roll(), dance_nav->get_pitch(), dance_nav->get_yaw(),true, get_smoothing_gain());
    

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();

    if (dance_nav->is_finished()) {
        if (dance_nav->land_is_allowed()) {
            dance_mode = Dance_Land;
        }
    };
}

// for telemetry
uint8_t Copter::get_dance_state() {
    // 0 - unknown
    // 1 - wait for takeoff
    // 2 - takeoff
    // 3 - wait for moving
    // 4 - moving & wait for dance
    // 5 - dancing
    uint8_t dm = 0;
    if (dance_nav->is_init()) {
        if (dance_mode == Dance_WaitingToTakeoff) {
            dm = 1;
        } else if (dance_mode == Dance_TakeOff) {
            dm = 2;
        } else if (dance_mode == Dance_Dance) {
            switch (dance_nav->get_dance_state()) {
                case 0:
                    dm = 3;
                    break;
                case 1:
                    dm = 4;
                    break;
                case 2:
                    dm = 5;
                    break;
            }
        }
    }
    return dm;
}
