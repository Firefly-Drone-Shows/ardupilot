#include <AP_HAL/AP_HAL.h>
#include "AC_Dance.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>


#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Dance::var_info[] = {
    // @Param: GPS_TIME
    // @DisplayName: Dance starting time
    // @Description: Defines the starting time of the Dance in GPS Time Units. Value must be represented as 132456: 13 hours 24 minutes 56 seconds in UTC.
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("GPS_TIME",  0,  AC_Dance, _starting_gps_time, 0),

    // @Param: CNT_LAT
    // @DisplayName: Dance center latitude
    // @Description: 
    // @Units: deg
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CNT_LAT",  1,  AC_Dance, _center_latitude, 0),

    // @Param: CNT_LON
    // @DisplayName: Dance center longitude
    // @Description: 
    // @Units: deg
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CNT_LON",  2,  AC_Dance, _center_longitude, 0),

    // @Param: CNT_AMSL
    // @DisplayName: Dance center altitude
    // @Description: 
    // @Units: meters, AMSL
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CNT_AMSL",  3,  AC_Dance, _center_altitude, 0),

    // @Param: CNT_HDG
    // @DisplayName: Dance center orientation
    // @Description: 
    // @Units: deg
    // @Range: 
    // @Increment: 
    // @User: Advanced
    AP_GROUPINFO("CNT_HDG",  4,  AC_Dance, _center_heading, 0),

    // @Param: MV_POS
    // @DisplayName: Seconds before start to move to first position
    // @Description: Defines the seconds before start when UAV starts to move to first position
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MV_POS",  5,  AC_Dance, _moving_to_position_offset, AC_DANCE_SECONDS_TO_FIRST_POSITION_BEFORE_START),

    // @Param: MAX_WAIT
    // @DisplayName: Max time to start dance in seconds
    // @Description: Defines the maximum time to start dance when Dance mode can be inited. In seconds.
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_WAIT",  6,  AC_Dance, _max_waiting_time, AC_DANCE_MAX_WAITING_TIME),

    // @Param: MIN_WAIT
    // @DisplayName: Min time to start dance in seconds
    // @Description: Defines the minimum time to start dance when Dance mode can be inited. In seconds.
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_WAIT",  7,  AC_Dance, _min_waiting_time, AC_DANCE_MIN_WAITING_TIME),

    // @Param: FPS
    // @DisplayName: FPS of dance (points per seconds)
    // @Description: Defines FPS of dance (points per seconds).
    // @Units: frames
    // @Range: 0 100000000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FPS",  8,  AC_Dance, _fps, AC_DANCE_DEFAULT_FPS),

    // @Param: DANCE GROUP
    // @DisplayName: Dance group number
    // @Description: Defines Dance group number
    // @Units: int
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GROUP",  9,  AC_Dance, _dance_group, AC_DANCE_DEFAULT_GROUP),

    // @Param: TK_OFF
    // @DisplayName: Seconds before start to make a takeoff
    // @Description: Defines the seconds before start when UAV starts to taking off
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TK_OFF",  10,  AC_Dance, _takeoff_offset, AC_DANCE_SECONDS_TO_TAKEOFF_BEFORE_START),

    // @Param: FNSH
    // @DisplayName: Finish mode
    // @Description: 0 - LAND after mission end, 1 - RTL aftre mission end
    // @Units: int
    // @Range: 0 1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FNSH",  11,  AC_Dance, _finish_mode, 0),

    // @Param: TK_ALT
    // @DisplayName: Takeoff altitude
    // @Description: Defines the altitude in meters to takeoff. If equal to 0, UAV will takeoff to altitude of first dance point
    // @Units: meters
    // @Range: 0 100000000
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("TK_ALT",  12,  AC_Dance, _takeoff_altitude, AC_DANCE_TAKEOFF_ALTITUDE),

    // @Param: MIN_ALT
    // @DisplayName: Minimal altitude
    // @Description: Defines the minimal altitude in meters.
    // @Units: meters
    // @Range: 0 100000000
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("MIN_ALT",  13,  AC_Dance, _min_altitude, AC_DANCE_MIN_ALTITUDE),

     // @Param: LN_OFF
    // @DisplayName: Seconds after finish to start landing or rtl sequence
    // @Description: Defines the seconds after finish to start landing or rtl sequence
    // @Units: time
    // @Range: 0 100000000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LN_OFF",  14,  AC_Dance, _land_offset, 0),

    // @Param: DANCE_TYPE
    // @DisplayName: Type of dance fromat. 0 = RGB, 1 = R + YAW in centidegrees.
    // @Description: Type of dance fromat. 0 = RGB, 1 = R + YAW in centidegrees.
    // @Units: int
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TYPE",  15,  AC_Dance, _path_type, 0),

    AP_GROUPINFO("TEL_MAIN",  16,  AC_Dance, _tel_main, 1),
    AP_GROUPINFO("TEL_GPS",  17,  AC_Dance, _tel_gps, 1),
    AP_GROUPINFO("TEL_SPD",  18,  AC_Dance, _tel_spd, 3),
    AP_GROUPINFO("TEL_PREF",  19,  AC_Dance, _tel_pref, 5),

    AP_GROUPINFO("MUTE",  20,  AC_Dance, _mute, 1),

    AP_GROUPINFO("TEL_MODE",  21,  AC_Dance, _tel_mode, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Dance::AC_Dance(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_Fence& fence, AC_Dancing_LED& dancing_led) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _fence(fence),
    _yaw(0.0f),
    _dance_started(false),
    _time_from_start_ms(0),
    _io_failure(false),
    _fd(-1),
    _path_length(0),
    _dance_state(0),
    _dancing_led(dancing_led),
    _path_in_fence(false)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (asprintf(&_file_path, "%s/MY.PATH", HAL_BOARD_PATH_DIRECTORY) <= 0) {
        _io_failure = true;
        _file_path = nullptr;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Cannot create path file name!");
    }

    read_path();
    
    _max_altitude_during_path = 0;
    _min_altitude_during_path = 0;
    _max_distance_from_center = 0;
    _max_altitude_during_path_timestamp = 0;
    _min_altitude_during_path_timestamp = 0;

    _max_h_point_distance = 0;
    _max_v_point_distance = 0;
    _max_angle_difference = 0;

    _previous_path_fitting_fps = 0;
    _previous_path_type = 0;
    
    calculate_path_fitting();
	
    _dance_info_sent_to_gcs = false;

    _set_color_time = 0;
    _set_color_duration = 0;
}


/// init - initialise Dance controller setting
void AC_Dance::init()
{

    if (!_path_successful_loaded) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Path was not loaded! Length: %d", _path_length);
        return;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Path was successfully loaded. Length: %d", _path_length);

    // set yaw
    _yaw = _ahrs.yaw * AC_DANCE_DEGX100;

    _time_from_start_ms = 0;   
    _dance_info_sent_to_gcs = false;
    _dance_started = false;
    //_start_time = hal.util->get_system_clock_ms();
    
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // if parameter lower that zero - set starting time in 10 seconds after switch to mode
    if (_starting_gps_time < 0) {
        int32_t hour, min, sec, ms;
        hal.util->get_system_clock_utc(hour, min, sec, ms);
        ms = 0;
        sec += 20;
        if (sec>59) {sec = sec - 60; min++;}
        if (min>59) {min = min - 60; hour++;}
        if (hour>23) {hour=0;}
        _starting_gps_time = sec + min*100 + hour*10000;
    }

	inited = true;
    _dance_state = 0;

}

int32_t AC_Dance::get_time_to_start()
{
    int32_t hour;
    
    AP_GPS gps = _ahrs.get_gps();

    const uint64_t gps_time = gps.time_epoch_usec();
    uint64_t utc_time = gps_time / 1000ULL;
    utc_time = utc_time % 86400000;
    hour = utc_time / 3600000;

    int16_t starting_hour = _starting_gps_time / 10000;
    int16_t starting_min = (_starting_gps_time % 10000) / 100;
    int16_t starting_sec = (_starting_gps_time % 100) ;
    
    // if hours difference is big - skip
    if (starting_hour < hour) {
        starting_hour += 24;
    }

    int32_t ret = (starting_hour * 3600000 + starting_min * 60000 + starting_sec * 1000) - utc_time;

    if (ret > 43200000) {
        ret = ret - 86400000;
    }

    return ret;
}

int32_t AC_Dance::get_dance_duration_ms() {
    float duration_seconds = static_cast<float>(get_path_length()) / get_fps();
    return static_cast<int32_t>(duration_seconds * 1000);
}

/// check if takeoff is allowed
bool AC_Dance::takeoff_is_allowed()
{
    if (!inited) {
		return false;
	}

    int32_t time_to_start = get_time_to_start();

    if (time_to_start > 0 && time_to_start < _takeoff_offset * 1000) {
        return true;
    }

    return false;
}

/// check if land is allowed
bool AC_Dance::land_is_allowed()
{
    if (!inited) {
		return false;
	}

    if (!is_finished()) {
        return false;
    } else if (_land_offset <= 0) {
        return true;
    } 

    return (get_seconds_to_land() < 0);
}

int32_t AC_Dance::get_seconds_to_land()
{
    int32_t time_after_start = -get_time_to_start();
    return  (get_dance_duration_ms() + (_land_offset * 1000) - time_after_start) / 1000;
}



/// update - update Dance controller
void AC_Dance::update()
{

	if (!inited) {
		return;
	}

    AP_GPS gps = _ahrs.get_gps();

    if (!_dance_started) {
        // get difference:
        int32_t time_to_start = get_time_to_start();

        if (!_dance_info_sent_to_gcs){
		   GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Time to start = %u sec.", time_to_start / 1000);
           _dance_info_sent_to_gcs = true;
           // fix current position to hold it during waiting
           takeoff_position = _pos_control.get_pos_target();
        }

        // check if we need to move UAVs to 1st position
        if (time_to_start > 0 && time_to_start < _moving_to_position_offset * 1000) {
            _start_time = (gps.time_epoch_usec() / 1000ULL) + time_to_start;   

            hal.console->printf("%d ms to start. Moving to start positions.\n", time_to_start);  
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%d ms to start. Moving to start positions.", time_to_start);
            
            _dance_started = true;
            _dance_state = 1;
        } else {
            // calculate dt
            float dt = _pos_control.time_since_last_xy_update();

            // update Dance position at poscontrol update rate
            if (dt >= _pos_control.get_dt_xy()) {
                // fight with possible drift
                // stay where we are now
                _pos_control.set_alt_target(takeoff_position.z);
                _pos_control.set_xy_target(takeoff_position.x, takeoff_position.y);

                // update position controller
                _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f, false);
                _pos_control.update_z_controller();
            }
        }
    }


    if (_dance_started) {
        // calculate dt
        float dt = _pos_control.time_since_last_xy_update();


        // update Dance position at poscontrol update rate
        if (dt >= _pos_control.get_dt_xy()) {

            // double check dt is reasonable
            if (dt >= 0.2f) {
                dt = 0.0f;
            }
        
            _time_from_start_ms = (gps.time_epoch_usec() / 1000ULL) - _start_time;

            if (_time_from_start_ms > 0) {
                _dance_state = 2;
            }

            Vector3f target;
            uint8_t color_r;
            uint8_t color_b;
            uint8_t color_g;
            uint16_t yaw;
            get_target_position_and_color((float)(_time_from_start_ms) / 1000.0f, target, yaw, color_r, color_g, color_b);
        
		if(_dance_state == 1) { 
           		_pos_control.set_alt_target_with_slew(target.z, dt);
		} else if(_dance_state == 2) {
           		_pos_control.set_alt_target(target.z);
		}

            // update position controller target
            _pos_control.set_xy_target(target.x, target.y);

            // update position controller
            _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f, false);
            _pos_control.update_z_controller();

            
            if (_path_type == 0) {
                // update LED
                set_color(color_r, color_g, color_b);
            } else if (_path_type == 1) {
                // update LED
                set_color(color_r, 0, 0);
                _yaw = yaw;
            }
        }
    }
}


void AC_Dance::set_color(uint8_t r, uint8_t g, uint8_t b) 
{

    // override dance value till _set_color_duration
    // this is for blinking is not to be overrided by dance colors
    if (_set_color_time > 0 && _set_color_time + _set_color_duration > AP_HAL::millis()) {
        return;
    }

    _dancing_led.set_color(r, g, b);
}


bool AC_Dance::is_finished() {
    if (!_dance_started) {
            return false;
    }

    uint32_t i = get_current_path_waypoint_number((float)(_time_from_start_ms) / 1000.0f);

    if (i >= (_path_length - 2)) {
        return true;
    }

    return false;
}


uint32_t AC_Dance::get_current_path_waypoint_number(float t) {

    if (t<=0) {
        return 0;
    }

    // calculate current path waypoint number
    // t > 0 here! 
    uint32_t i = (uint32_t)(t * _fps);

    // if waypoint number is larger thap path legth - set previous before last waypoint as active 
    if (i > (_path_length - 2)) {
        i = _path_length - 2;
    }

    return i;
}


void AC_Dance::get_target_position(float t, Vector3f &pos)
{
    uint8_t color_r;
    uint8_t color_b;
    uint8_t color_g;

    uint16_t yaw;

    get_target_position_and_color(t, pos, yaw, color_r, color_g, color_b);
}

// get target position and color
void AC_Dance::get_target_position_and_color(float t, Vector3f &pos, uint16_t &yaw, uint8_t &r, uint8_t &g, uint8_t &b)
{
    if (t > 0) {

        uint32_t i = get_current_path_waypoint_number(t);
        
        // sanity check
        // yeah, same check is inside get_current_path_waypoint_number()
        // but it may accidentally change in the future
        // and we have _traj[i+1] down below
        if (i > (_path_length - 2)) {
            i = _path_length - 2;
        }

        // make linear interpolation between current waypoint and next one
        float pt = (t * _fps) - i;
        if (pt > 1) {
            pt = 1;
        }

        // SIMPLE POSITION 
        pos.x = linear_interpolate(_traj[i].x, _traj[i+1].x, pt, 0, 1);
        pos.y = linear_interpolate(_traj[i].y, _traj[i+1].y, pt, 0, 1);
        pos.z = linear_interpolate(_traj[i].z, _traj[i+1].z, pt, 0, 1);
      
        r = linear_interpolate(static_cast<float>(_color_r[i]), static_cast<float>(_color_r[i+1]), pt, 0, 1);
        g = linear_interpolate(static_cast<float>(_color_g[i]), static_cast<float>(_color_g[i+1]), pt, 0, 1);
        b = linear_interpolate(static_cast<float>(_color_b[i]), static_cast<float>(_color_b[i+1]), pt, 0, 1);

        uint16_t current_yaw = get_angle_from_GB(_color_g[i], _color_b[i]);
        uint16_t next_yaw = get_angle_from_GB(_color_g[i+1], _color_b[i+1]);

        // check if we are going through zero
        if ((next_yaw > current_yaw) && ((next_yaw - current_yaw) > 18000)) {
            next_yaw = next_yaw - 36000;
        } else if ((next_yaw < current_yaw) && ((current_yaw - next_yaw) > 18000)) {
            current_yaw = current_yaw - 36000;
        }

        float yaw_inter = linear_interpolate(static_cast<float>(current_yaw), static_cast<float>(next_yaw), pt, 0, 1);
        
        if (yaw_inter < 0) {
            yaw_inter = 36000 + yaw_inter;
        }

        yaw = static_cast<uint16_t>(yaw_inter);

    } else {
        pos = _traj[0];
        r = _color_r[0];
        g = _color_g[0];
        b = _color_b[0];
        yaw = get_angle_from_GB(_color_g[0], _color_b[0]);
    }


    // calculate offset to dance center
    Vector2f offset_to_center = get_horizontal_offset_to_center();

    // add offset and heading
    if (_center_latitude != 0 && _center_longitude != 0) {
        const float a = radians(_center_heading);
        const float t_x = pos.x * cosf(a) - pos.y * sinf(a);
        const float t_y = pos.x * sinf(a) + pos.y * cosf(a);

        pos.x = t_x + (offset_to_center.x*100);
        pos.y = t_y + (offset_to_center.y*100);
    }


    // current relative position
    const Vector3f &curr_pos = _inav.get_position();

    // current absolute position for AMSL.
    Location curr_amsl_loc;
    //_inav.get_location(curr_amsl_loc);
    _ahrs.get_position(curr_amsl_loc);


    // target AMSL
    float target_amsl = _center_altitude*100 + pos.z;


    // calculate target relative alt to reach correct AMSL
    pos.z = curr_pos.z + target_amsl - curr_amsl_loc.alt;

}


Vector2f AC_Dance::get_horizontal_offset_to_center()
{
    const struct Location &ekf_origin = _inav.get_origin();

    Location dance_center;
    dance_center.lat = _center_latitude * 10 * 1000 * 1000L;
    dance_center.lng = _center_longitude * 10 * 1000 * 1000L;

    Vector2f ret = location_diff(ekf_origin, dance_center);

    return ret;

}


void AC_Dance::check_path_in_fence() {

    if (!_path_in_fence
            || _center_altitude != _prev_center_altitude
            || _center_latitude != _prev_center_latitude
            || _center_longitude != _prev_center_longitude
            || _center_heading != _prev_center_heading
            || _prev_fence_crc != _fence.calc_fence_crc()           
            ) {

        Location loc;
        const float a = radians(_center_heading);

        _path_in_fence = true;

        for (uint32_t i = 0; i < _path_length; i++)
        {
            loc.lat = _center_latitude * 10 * 1000 * 1000;
            loc.lng = _center_longitude * 10 * 1000 * 1000;

            float p_x = _traj[i].x / 100.0;
            float p_y = _traj[i].y / 100.0;

            const float t_x = p_x * cosf(a) - p_y * sinf(a);
            const float t_y = p_x * sinf(a) + p_y * cosf(a);

           location_offset(loc, t_x, t_y);

            // centimeters
            loc.alt = (_center_altitude * 100) + _traj[i].z;

            if (!_fence.check_destination_within_fence(loc))
            {
                _path_in_fence  = false;
                break;
            }
        }

        if (_path_in_fence) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "All points are in fence");
        }

        _prev_center_altitude = _center_altitude;
        _prev_center_heading = _center_heading;
        _prev_center_latitude = _center_latitude;
        _prev_center_longitude = _center_longitude;
        _prev_fence_crc = _fence.calc_fence_crc();
    }
}


Location AC_Dance::get_first_point_position()
{
    Location ret; 

    ret.lat = _center_latitude * 10 * 1000 * 1000;
    ret.lng = _center_longitude * 10 * 1000 * 1000;

    float p_x = _traj[0].x / 100.0;
    float p_y = _traj[0].y / 100.0;

    const float a = radians(_center_heading);
    const float t_x = p_x * cosf(a) - p_y * sinf(a);
    const float t_y = p_x * sinf(a) + p_y * cosf(a);
    
    location_offset(ret, t_x, t_y);

    // centimeters
    ret.alt = (_center_altitude * 100) + _traj[0].z;

    return ret;

}

// 0 - ready
// 1 - path length < 1
// 2 - center locations doesn' set
// 4 - onboad time > start time
int16_t AC_Dance::ready_to_dance() {
    
    if ((_fps != _previous_path_fitting_fps) || (_path_type != _previous_path_type)) {
        // calculate_path_fitting finds timestamps with minimum and maximum alt. So
        // when we change fps we need to recalculate timestamps. Also when we change type we
        // need to recalculate angular fitting
        calculate_path_fitting();
    }
    int16_t ret = 0; 

    if (!_path_successful_loaded) {
        ret += 1;
    }    

    if (_path_length < 1) {
        ret += 2;
    }
    
    if (_center_latitude == 0 || _center_longitude == 0) {
        ret += 4;
    }
    
    int32_t f_time = get_time_to_start();
    if (f_time > _max_waiting_time * 1000 || f_time < _min_waiting_time * 1000) {
        ret += 8;
    } 

    Vector3f f_target;
    float to_alt = get_takeoff_altitude();

    // min altitude during mission
    get_target_position(_min_altitude_during_path_timestamp, f_target);
    _min_altitude_during_path = f_target.z;
    if ((_min_altitude_during_path < _min_altitude * 100) || (to_alt < _min_altitude * 100)) {
        ret += 16;
    }
    
    // distance
    float dist_to_first_point = norm(f_target.x, f_target.y) * 0.01; // meters
    if (dist_to_first_point > _fence.get_radius()) {
        ret += 32;
    } 

    // fence enable (at last circle & alt) and fence_action > 0
    uint8_t fences_enabled = _fence.get_enabled_fences();
    if (
        (!(fences_enabled & AC_FENCE_TYPE_ALT_MAX && fences_enabled & AC_FENCE_TYPE_CIRCLE)) ||
        (!_fence.enabled() || !(_fence.get_action() > 0)) 
       ) {
        ret += 64;
    } 

    // max altitude during mission
    get_target_position(_max_altitude_during_path_timestamp, f_target);
    _max_altitude_during_path = f_target.z;
    if ((_max_altitude_during_path > _fence.get_safe_alt_max() * 100) || (to_alt > _fence.get_safe_alt_max() * 100))  {
        ret += 128;
    }

    // fps
    if (!(_fps > 0)) {
        ret += 256;
    }

    // chech if the path is in fence.
    if (_path_successful_loaded) {
        check_path_in_fence();
    }

    if (!_path_in_fence) {
        ret += 512;
    }

    // wrong path type
    if (_path_type != 0 && _path_type != 1 ) {
        ret += 1024;
    }

    return ret;
}

/*
  open the current path file
 */
void AC_Dance::open_file(void)
{
    
    if (_fd != -1) {
        // already open
        return;
    }

    if (_file_path == nullptr) {
        _io_failure = true;
        return;
    }

    if (_fd != -1) {
        ::close(_fd);
    }

    // only read for now
    _fd = ::open(_file_path, O_RDONLY);

    if (_fd == -1) {
       hal.console->printf("Path file open %s failed - %s\n",
                            _file_path, strerror(errno));
       GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Path file open failed");
        _io_failure = true;
        return;
    }
}


void AC_Dance::read_path(void)
{
    open_file();

    int16_t val;
    int s;
    _path_length = 0;

    // reed id
/*    s = ::read(_fd, &val, 2);
	if (s!=2) { 
	    hal.console->printf("Path file is corrupted. Cannot load ID.\n");    
	    ::close(_fd);
	    return;
	}
	_path_id = val; */

    while(_path_length < AC_DANCE_PATH_SIZE_MAX) {
        s = ::read(_fd, &val, 2);
        
        if (s!=2) { 
            hal.console->printf("!Successfull points loaded - %d\n",
                        _path_length);  
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                         "Path file is ok. Successfull points loaded - %d",
                         _path_length);
            _path_successful_loaded = true;
            ::close(_fd);  
            return;
        }
        _traj[_path_length].x = val;
       
        s = ::read(_fd, &val, 2);
        if (s!=2) { 
            _path_length--;
            hal.console->printf("Path file is corrupted. Successfull points loaded - %d\n",
                        _path_length);    
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Path file is corrupted. Successfull points loaded - %d\n",
                         _path_length);
            ::close(_fd);
            return;
        }
        _traj[_path_length].y = -val;
        
        s = ::read(_fd, &val, 2);
        if (s!=2) { 
            _path_length--;
            hal.console->printf("Path file is corrupted. Successfull points loaded - %d\n",
                        _path_length); 
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Path file is corrupted. Successfull points loaded - %d\n",
                         _path_length);   
            ::close(_fd);
            return;
        }
        _traj[_path_length].z = val;
        
        // COLOR

        s = ::read(_fd, &val, 2);
        if (s!=2) { 
            _path_length--;
            hal.console->printf("Path file is corrupted. Successfull points loaded - %d\n",
                        _path_length);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Path file is corrupted. Successfull points loaded - %d\n",
                         _path_length);
            ::close(_fd);
            return;
        }
        _color_r[_path_length] = val;
        
        s = ::read(_fd, &val, 2);
        if (s!=2) { 
            _path_length--;
            hal.console->printf("Path file is corrupted. Successfull points loaded - %d\n",
                        _path_length);    
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Path file is corrupted. Successfull points loaded - %d\n",
                         _path_length);
            ::close(_fd);
            return;
        }
        _color_g[_path_length] = val;

        s = ::read(_fd, &val, 2);
        if (s!=2) { 
            _path_length--;
            hal.console->printf("Path file is corrupted. Successfull points loaded - %d\n",
                        _path_length);    
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Path file is corrupted. Successfull points loaded - %d\n",
                         _path_length);
            ::close(_fd);
            return;
        }
        _color_b[_path_length] = val;


        _path_length++;
    }

    if (_path_length == AC_DANCE_PATH_SIZE_MAX) {
        _path_successful_loaded = true;
    }

    ::close(_fd);

}


void AC_Dance::handle_set_color(uint8_t r, uint8_t g, uint8_t b, uint16_t duration, uint8_t blink_rate, uint16_t group)
{
    // if message for wrong group - ignore
    // if group = 0, message is accessable by every vehicle
    if ((group != 0) && (_dance_group != group)) {
        return;
    }

    if (_dancing_led.set_color(r, g, b, duration, blink_rate)) {
        _set_color_time = AP_HAL::millis();
        _set_color_duration = duration;
    }
}


/* 
   handle path messages from GCS
 */
void AC_Dance::handle_data_v2(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_V2_EXTENSION) {
        
        mavlink_v2_extension_t packet;
        mavlink_msg_v2_extension_decode(msg, &packet);

        // path chunk
        if (packet.message_type == 9880) {

            // header description
            // bytes: descr
            // 0-1  : path id
            // 2-3  : size (number of points)
            // 4-7  : hash
            // 8-9  : chunk index
            // 10-11: chunk size

            int16_t h_path_id = (packet.payload[0] << 8) | packet.payload[1];
            int16_t h_path_size = (packet.payload[2] << 8) | packet.payload[3];
            int32_t h_path_hash = (packet.payload[4] << 24) | (packet.payload[5] << 16) | (packet.payload[6] << 8) | packet.payload[7];
            int16_t h_chunk_idx = (packet.payload[8] << 8) | packet.payload[9];
            int16_t h_chunk_size = (packet.payload[10] << 8) | packet.payload[11];

            // fist get chunk size. if size == 0 it means that this is the initial request for upload
            hal.console->printf("Path v2 req: %d, %d, %d, %d, %d \n", h_path_id, h_path_size, h_path_hash, h_chunk_idx, h_chunk_size);
            
            

            if (h_chunk_size == 0) {
                // request for path upload
                new_path_size = h_path_size;   // number of points in path
                current_chunk = 0;
                new_path_hash = h_path_hash;
                new_path_id = h_path_id;
                
                // TODO: swithc to const
                if ((new_path_size % AC_DANCE_POINTS_IN_ONE_CHUNK_V2) == 0) {
                    chunk_number = new_path_size / AC_DANCE_POINTS_IN_ONE_CHUNK_V2;  // number of mavlink messages 
                } else {
                    chunk_number = new_path_size / AC_DANCE_POINTS_IN_ONE_CHUNK_V2 + 1;
                }

                hal.console->printf(">>>> path_size=%d, chunk_number=%d \n", new_path_size, chunk_number);
                
                mkdir(HAL_BOARD_PATH_DIRECTORY, 0755);

                hal.console->printf("Sending request \n");

                // send first data request
                uint8_t payload[249] = {0};
                // payload[0] = 0;
                payload[1] = packet.payload[4];
                payload[2] = packet.payload[5];
                payload[3] = packet.payload[6];
                payload[4] = packet.payload[7];
                // payload[5] = 0;
                // payload[6] = 0;
                mavlink_msg_v2_extension_send(chan, 0, 0, 0, 9881, payload);
                hal.console->printf("Sended \n");

            } 
            
            else if (h_chunk_size < 255) {
                uint16_t chunk_size = h_chunk_size;
                current_chunk = h_chunk_idx;

                // TMP for test only
                //if (h_chunk_idx == 3 && !tmp_skip_26) {
                  //  tmp_skip_26 = true;
//                    return;
  //              }

                hal.console->printf("MESSAGE with data: chunk_id=%d, size=%d \n", h_chunk_idx, chunk_size);

                // write chunk
                FILE * p_file;

                if (_file_path == nullptr) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Error creating file, filepath null reference");   
                         return;
                }

                if (current_chunk==0) {
                    p_file = ::fopen(_file_path, "w");
                } else {
                    p_file = ::fopen(_file_path, "a");
                }

                ::fseek(p_file, current_chunk * AC_DANCE_POINTS_IN_ONE_CHUNK_V2 * AC_DANCE_POINT_BYTE_SIZE, SEEK_SET);
                ::fwrite(&packet.payload[12], sizeof(char), chunk_size, p_file); 
                ::fclose(p_file);

                current_chunk++;

                uint8_t payload[249] = {0};
                // payload[0] = 0;
                payload[1] = packet.payload[4];
                payload[2] = packet.payload[5];
                payload[3] = packet.payload[6];
                payload[4] = packet.payload[7];
                
                if (current_chunk >= chunk_number) {
                    // complete
                    payload[0] = 2;
                } else {
                    // new request
                    payload[0] = 0;
                    payload[5] = (current_chunk >> 8) & 0xFF;
                    payload[6] = current_chunk & 0xFF;
                }
                mavlink_msg_v2_extension_send(chan, 0, 0, 0, 9881, payload);

            } else if (h_chunk_size == 255) {

                // TODO: ABORT
                hal.console->printf("Path downloading abort \n");
            } 
        }
    }
}



/* 
   handle path messages from GCS (DATA96)
 */
void AC_Dance::handle_data_v3(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_DATA96) { 

        mavlink_data96_t packet;
        mavlink_msg_data96_decode(msg, &packet);

        // path chunk
        if (packet.type == 59) {

            // header description
            // bytes: descr
            // 0-1  : path id
            // 2-3  : size (number of points)
            // 4-7  : hash
            // 8-9  : chunk index
            // 10-11: chunk size

            int16_t h_path_id = (packet.data[0] << 8) | packet.data[1];
            int16_t h_path_size = (packet.data[2] << 8) | packet.data[3];
            int32_t h_path_hash = (packet.data[4] << 24) | (packet.data[5] << 16) | (packet.data[6] << 8) | packet.data[7];
            int16_t h_chunk_idx = (packet.data[8] << 8) | packet.data[9];
            int16_t h_chunk_size = (packet.data[10] << 8) | packet.data[11];

            // fist get chunk size. if size == 0 it means that this is the initial request for upload
            hal.console->printf("Path data96 req: %d, %d, %d, %d, %d \n", h_path_id, h_path_size, h_path_hash, h_chunk_idx, h_chunk_size);
         
            if (h_chunk_size == 0) {
                // request for path upload
                new_path_size = h_path_size;   // number of points in path
                current_chunk = 0;
                new_path_hash = h_path_hash;
                new_path_id = h_path_id;
                
                // TODO: swithc to const
                if ((new_path_size % AC_DANCE_POINTS_IN_ONE_CHUNK_DATA96) == 0) {
                    chunk_number = new_path_size / AC_DANCE_POINTS_IN_ONE_CHUNK_DATA96;  // number of mavlink messages 
                } else {
                    chunk_number = new_path_size / AC_DANCE_POINTS_IN_ONE_CHUNK_DATA96 + 1;
                }

                hal.console->printf(">>>> path_size=%d, chunk_number=%d \n", new_path_size, chunk_number);
                
                mkdir(HAL_BOARD_PATH_DIRECTORY, 0755);

                hal.console->printf("Sending request \n");

                // send first data request
                uint8_t payload[16] = {0};
               
                payload[1] = packet.data[4];
                payload[2] = packet.data[5];
                payload[3] = packet.data[6];
                payload[4] = packet.data[7];
               
                mavlink_msg_data16_send(chan, 60, 0, payload);
                hal.console->printf("Sended data16 req\n");

            } 
            
            else if (h_chunk_size < 255) {
                uint16_t chunk_size = h_chunk_size;
                current_chunk = h_chunk_idx;

                hal.console->printf("MESSAGE DATA96 with data: chunk_id=%d, size=%d \n", h_chunk_idx, chunk_size);

                // write chunk
                FILE * p_file;

                if (_file_path == nullptr) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,
                         "Error creating file, filepath null reference");   
                         return;
                }

                if (current_chunk==0) {
                    p_file = ::fopen(_file_path, "w");
                } else {
                    p_file = ::fopen(_file_path, "a");
                }

                ::fseek(p_file, current_chunk * AC_DANCE_POINTS_IN_ONE_CHUNK_DATA96 * AC_DANCE_POINT_BYTE_SIZE, SEEK_SET);
                ::fwrite(&packet.data[12], sizeof(char), chunk_size, p_file); 
                ::fclose(p_file);

                current_chunk++;

                uint8_t payload[16] = {0};
                // payload[0] = 0;
                payload[1] = packet.data[4];
                payload[2] = packet.data[5];
                payload[3] = packet.data[6];
                payload[4] = packet.data[7];
                
                if (current_chunk >= chunk_number) {
                    // complete
                    payload[0] = 2;
                } else {
                    // new request
                    payload[0] = 0;
                    payload[5] = (current_chunk >> 8) & 0xFF;
                    payload[6] = current_chunk & 0xFF;
                }
                mavlink_msg_data16_send(chan, 60, 0, payload);

            } else if (h_chunk_size == 255) {

                // TODO: ABORT
                hal.console->printf("Path downloading abort \n");
            } 
        }
    }
}

void AC_Dance::calculate_path_fitting()
{
    float max_a = -9999999;
    float min_a = 9999999;

    _max_h_point_distance = 0;
    _max_v_point_distance = 0;
    _max_angle_difference = 0;

    for (uint16_t i = 0; i < _path_length; i++){
        if (max_a < _traj[i].z) {
            max_a = _traj[i].z;
            _max_altitude_during_path_timestamp = (float)(i) / _fps;
        }
        if (min_a > _traj[i].z) {
            min_a = _traj[i].z;
            _min_altitude_during_path_timestamp = (float)(i) / _fps;
        }
        
        float d = norm(_traj[i].x, _traj[i].y);
        if (_max_distance_from_center < d) {
            _max_distance_from_center = d;
        }

        if (i > 0 && _fps > 0) {
            // max horizontal and vertical delta
            float d_h = norm((_traj[i].x - _traj[i-1].x), (_traj[i].y - _traj[i-1].y));
            float d_v = fabsf(_traj[i].z - _traj[i-1].z);
            if (_max_h_point_distance < d_h) {
                _max_h_point_distance = d_h;
            }

            if (_max_v_point_distance < d_v) {
                _max_v_point_distance = d_v;
            }

            // if we use path with YAW control
            if (_path_type == 1) {
                // angle delta
                float d_a = static_cast<float>(get_angle_from_GB(_color_g[i], _color_b[i])) - static_cast<float>(get_angle_from_GB(_color_g[i-1], _color_b[i-1]));

                if (d_a < -18000) {
                    d_a = 36000 + d_a;
                } else if (d_a > 18000) {
                    d_a = 36000 - d_a;
                }
                
                // to degrees
                d_a = fabsf(d_a / 100);

                if (_max_angle_difference < d_a) {
                    _max_angle_difference = d_a;
                }
            }
        }
    }

    _previous_path_fitting_fps = _fps;
    _previous_path_type = _path_type;

     hal.console->printf("FPS or TYPE is changed. Recalculate fitting. \n");
}

bool AC_Dance::return_to_home_position() 
{
    return (_finish_mode == 1);
}

int16_t AC_Dance::get_takeoff_offset() 
{
    return _takeoff_offset;
}

int16_t AC_Dance::get_moving_to_position_offset() 
{
    return _moving_to_position_offset;
}

float AC_Dance::get_takeoff_altitude() 
{
    if (_takeoff_altitude > 0) {
        return _takeoff_altitude * 100;
    }

    Vector3f f_target;
    get_target_position(0, f_target);

    return f_target.z;
}

bool AC_Dance::is_init() 
{
    return inited;
}

uint8_t AC_Dance::get_dance_state()
{
    return _dance_state;
}
