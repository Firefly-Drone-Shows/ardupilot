#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_Fence/AC_Fence.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AC_Dancing_LED/AC_Dancing_LED.h>

#define AC_DANCE_DEGX100           5729.57795f // constant to convert from radians to centi-degrees
#define AC_DANCE_POINTS_IN_ONE_CHUNK_V2     19
#define AC_DANCE_POINTS_IN_ONE_CHUNK_DATA96 7
#define AC_DANCE_POINT_BYTE_SIZE        12

#define AC_DANCE_SECONDS_TO_FIRST_POSITION_BEFORE_START  45
#define AC_DANCE_SECONDS_TO_TAKEOFF_BEFORE_START 60
#define AC_DANCE_TAKEOFF_ALTITUDE 5
#define AC_DANCE_MIN_ALTITUDE 1.5

#define AC_DANCE_PATH_SIZE_MAX 2000

#define AC_DANCE_DEFAULT_FPS 4
#define AC_DANCE_MAX_WAITING_TIME 600
#define AC_DANCE_MIN_WAITING_TIME 20

// zero means - no group
#define AC_DANCE_DEFAULT_GROUP 0

class AC_Dance
{
public:

    /// Constructor
    AC_Dance(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_Fence& fence, AC_Dancing_LED& led);

    /// init - initialise dance controller 
    void init();

    /// update - update dance controller
    void update();

    /// handle path data through v2_extension message
    void handle_data_v2(mavlink_channel_t chan, mavlink_message_t *msg);

    /// handle path data through data96 message
    void handle_data_v3(mavlink_channel_t chan, mavlink_message_t *msg);

    /// handle set_color command (analog of set_servo\repeat_servo for 3 servos)
    void handle_set_color(uint8_t r, uint8_t g, uint8_t b, uint16_t duration, uint8_t blink_rate, uint16_t group);

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); }
    int32_t get_pitch() const { return _pos_control.get_pitch(); }
    int32_t get_yaw() const { return _yaw; }

    int16_t get_path_id() const { return _path_id; }
    bool get_path_success() const { return _path_successful_loaded; }
    int16_t get_path_length() const { return _path_length; }
    int32_t get_dance_starting_time() const { return (int32_t)_starting_gps_time; }
    float get_max_altitude() const { return _max_altitude_during_path; }
    float get_min_altitude() const { return _min_altitude_during_path; }
    float get_max_distance_from_center() const { return _max_distance_from_center; }
    int16_t get_max_vertical_speed() const { return (int16_t)(_max_v_point_distance * _fps); }
    int16_t get_max_horizontal_speed() const { return (int16_t)(_max_h_point_distance * _fps); }
    uint16_t get_max_angular_speed() const { return (uint16_t)(_max_angle_difference * _fps); }
    float get_fps() const { return _fps; }
    float get_type() const { return _path_type; }
    int16_t get_land_offset() const { return _land_offset; }

    int8_t get_telemetry_main_rate() const { return _tel_main; }
    int8_t get_telemetry_gps_rate() const { return _tel_gps; }
    int8_t get_telemetry_speed_rate() const { return _tel_spd; }
    int8_t get_telemetry_preflight_rate() const { return _tel_pref; }
    int8_t get_telemetry_mode() const { return _tel_mode; }


    static const struct AP_Param::GroupInfo var_info[];

    bool is_init();

    bool is_finished();

    uint32_t get_current_path_waypoint_number(float t);

    void get_target_position_and_color(float t, Vector3f &pos, uint16_t &yaw, uint8_t &r, uint8_t &g, uint8_t &b);

    void get_target_position(float t, Vector3f &pos);

    void set_color(uint8_t r, uint8_t g, uint8_t b);

    int32_t get_time_to_start();

    int32_t get_seconds_to_land();

    int32_t get_dance_duration_ms();

    int16_t get_takeoff_offset();
    
    float get_takeoff_altitude();

    int16_t get_moving_to_position_offset();

    uint8_t get_dance_state();

    //int8_t get_finish_mode();

    int16_t ready_to_dance();

    bool takeoff_is_allowed();

    bool land_is_allowed();

    bool return_to_home_position();
        
    Location get_first_point_position();

    void check_path_in_fence();

private:

    uint8_t _dance_state;

    uint64_t _set_color_time;
    uint16_t _set_color_duration;

    bool _path_successful_loaded = false;

	bool inited = false;

    int16_t _path_id = -1;

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&       _inav;
    const AP_AHRS_View&         _ahrs;
    
    AC_Fence&                   _fence;
    AC_PosControl&              _pos_control;
    AC_Dancing_LED&             _dancing_led;

    // parameters
    AP_Int32 _starting_gps_time;        // gps time to start
    AP_Int32 _max_waiting_time;         // max seconds to start when dance can be inited
    AP_Int32 _min_waiting_time;         // min seconds to start when dance can be inited
    AP_Int16 _moving_to_position_offset;// seconds before start when UAV started to move to
                                        // first dance position
    AP_Int16 _takeoff_offset;           // seconds before start when UAV started to taking off
    AP_Int16 _land_offset;              // seconds after finish to start landing or rtl offsets
    AP_Int8 _finish_mode;               // 0 - LAND after dance, 1 - RTL after dance
    AP_Float _takeoff_altitude;         // relative takeoff altitude in meters
    AP_Float _min_altitude;             // relative min altitude in meters
    AP_Float _center_latitude;
    AP_Float _center_longitude;
    AP_Float _center_altitude;
    AP_Float _center_heading;
    AP_Int8 _mute;                       // mute all messages

    AP_Int8 _tel_main;
    AP_Int8 _tel_gps;
    AP_Int8 _tel_spd;
    AP_Int8 _tel_pref;
    AP_Int8 _tel_mode;                  // 1 means we run only EXTRA2 whatever rates were set for other streams


    AP_Int8 _path_type;                 // 0 - RGB type, R + YAW type.

    AP_Int16 _dance_group;

    AP_Float _fps;  // FPS. Default value = 4
    // previous fps value. we need to recalculate path_fitting when fps is changed.
    // we do it ready_to_dance function
    float _previous_path_fitting_fps;

    // previous path type value. we need to recalculate path_fitting when path type is changed.
    // we do it ready_to_dance function
    int _previous_path_type;

    float       _yaw;           // yaw heading 
    bool        _dance_started; 
    bool        _dance_info_sent_to_gcs;

    float       _max_altitude_during_path;
    float       _max_altitude_during_path_timestamp;
    float       _min_altitude_during_path;
    float       _min_altitude_during_path_timestamp;
    float       _max_distance_from_center;
    
    Vector3f _traj[AC_DANCE_PATH_SIZE_MAX]; 
    uint8_t _color_r[AC_DANCE_PATH_SIZE_MAX]; 
    uint8_t _color_g[AC_DANCE_PATH_SIZE_MAX]; 
    uint8_t _color_b[AC_DANCE_PATH_SIZE_MAX]; 

    int64_t _time_from_start_ms;
    uint64_t _start_time; // calculated start_time

    Vector3f takeoff_position;

    // open file handle on path file
    int _fd;
    // file path
    char *_file_path = nullptr;
    // do we have an IO failure
    volatile bool _io_failure;
    // number of points in path
    uint32_t _path_length;

    // max path speed
    float _max_h_point_distance;
    float _max_v_point_distance;
    // max angle delta int degrees
    float _max_angle_difference;
    
    // 
    uint32_t new_path_size;
    uint32_t new_path_hash;
    uint32_t current_chunk;
    uint32_t chunk_number;
    uint16_t new_path_id;

    // Previous values kept to track changes
    float _prev_center_latitude;
    float _prev_center_longitude;
    float _prev_center_altitude;
    float _prev_center_heading;
    uint16_t _prev_fence_crc;

    bool   _path_in_fence;

    Vector2f get_horizontal_offset_to_center();

    void calculate_path_fitting();

    void open_file();
    void read_path();

    uint16_t get_angle_from_GB(uint8_t g, uint8_t b) {return (b << 8) | g; }
};
