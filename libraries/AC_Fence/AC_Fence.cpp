#include <AP_HAL/AP_HAL.h>
#include "AC_Fence.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Fence::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Fence,   _enabled,   0),

    // @Param: TYPE
    // @DisplayName: Fence Type
    // @Description: Enabled fence types held as bitmask
    // @Values: 0:None,1:Altitude,2:Circle,3:Altitude and Circle,4:Polygon,5:Altitude and Polygon,6:Circle and Polygon,7:All
    // @Bitmask: 0:Altitude,1:Circle,2:Polygon
    // @User: Standard
    AP_GROUPINFO("TYPE",        1,  AC_Fence,   _enabled_fences,  AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON),

    // @Param: ACTION
    // @DisplayName: Fence Action
    // @Description: What action should be taken when fence is breached
    // @Values: 0:Report Only,1:RTL or Land
    // @User: Standard
    AP_GROUPINFO("ACTION",      2,  AC_Fence,   _action,        AC_FENCE_ACTION_RTL_AND_LAND),

    // @Param: ALT_MAX
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: Meters
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MAX",     3,  AC_Fence,   _alt_max,       AC_FENCE_ALT_MAX_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: Circular Fence Radius
    // @Description: Circle fence radius which when breached will cause an RTL
    // @Units: Meters
    // @Range: 30 10000
    // @User: Standard
    AP_GROUPINFO("RADIUS",      4,  AC_Fence,   _circle_radius, AC_FENCE_CIRCLE_RADIUS_DEFAULT),

    // @Param: MARGIN
    // @DisplayName: Fence Margin
    // @Description: Distance that autopilot's should maintain from the fence to avoid a breach
    // @Units: Meters
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN",      5,  AC_Fence,   _margin, AC_FENCE_MARGIN_DEFAULT),

    // @Param: TOTAL
    // @DisplayName: Fence polygon point total
    // @Description: Number of polygon points saved in eeprom (do not update manually)
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("TOTAL",       6,  AC_Fence,   _total, 0),

    // @Param: ALT_MIN
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: Meters
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN",     7,  AC_Fence,   _alt_min,       AC_FENCE_ALT_MIN_DEFAULT, AP_PARAM_FRAME_SUB),

    // @Param: HRD_DIST
    // @DisplayName: Max distance to disarm.
    // @Description: Distance outside the fence, when disarm initialized. -1 - means infinit distance
    // @Units: Meters
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HRD_DIST",     8,  AC_Fence,   _hard_dist,    AC_FENCE_HARD_DIST_DEFAULT),

    // @Param: HRD_ENABLE
    // @DisplayName: If on, Hard fence is enabled. Dron will be disarmed in flight if went outside fence + FENCE_HRD_DIST
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HRD_ENABLE",     9,  AC_Fence,   _hard_fence_enable,    0),

    AP_GROUPEND
};

/// Default constructor.
AC_Fence::AC_Fence(const AP_AHRS& ahrs, const AP_InertialNav& inav) :
    _ahrs(ahrs),
    _inav(inav),
    _alt_max_backup(0),
    _circle_radius_backup(0),
    _alt_max_breach_distance(0),
    _circle_breach_distance(0),
    _home_distance(0),
    _breached_fences(AC_FENCE_TYPE_NONE),
    _breach_time(0),
    _breach_count(0),
    _manual_recovery_start_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly fence values
    if (_alt_max < 0.0f) {
        _alt_max.set_and_save(AC_FENCE_ALT_MAX_DEFAULT);
    }
    if (_circle_radius < 0) {
        _circle_radius.set_and_save(AC_FENCE_CIRCLE_RADIUS_DEFAULT);
    }
}

/// get_enabled_fences - returns bitmask of enabled fences
uint8_t AC_Fence::get_enabled_fences() const
{
    if (!_enabled) {
        return AC_FENCE_TYPE_NONE;
    }else{
        return _enabled_fences;
    }
}

/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AC_Fence::pre_arm_check(const char* &fail_msg) const
{
    fail_msg = nullptr;

    // if not enabled or not fence set-up always return true
    if (!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return true;
    }

    // check no limits are currently breached
    if (_breached_fences != AC_FENCE_TYPE_NONE) {
        fail_msg =  "vehicle outside fence";
        return false;
    }

    // if we have horizontal limits enabled, check inertial nav position is ok
    if ((_enabled_fences & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON))>0 && !_inav.get_filter_status().flags.horiz_pos_abs && !_inav.get_filter_status().flags.pred_horiz_pos_abs) {
        fail_msg = "fence requires position";
        return false;
    }

    // if we got this far everything must be ok
    return true;
}

/// check_fence - returns the fence type that has been breached (if any)
///     curr_alt is the altitude above home in meters
uint8_t AC_Fence::check_fence(float curr_alt)
{
    uint8_t ret = AC_FENCE_TYPE_NONE;

    // return immediately if disabled
    if (!_enabled || _enabled_fences == AC_FENCE_TYPE_NONE) {
        return AC_FENCE_TYPE_NONE;
    }

    // check if pilot is attempting to recover manually
    if (_manual_recovery_start_ms != 0) {
        // we ignore any fence breaches during the manual recovery period which is about 10 seconds
        if ((AP_HAL::millis() - _manual_recovery_start_ms) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN) {
            return AC_FENCE_TYPE_NONE;
        } else {
            // recovery period has passed so reset manual recovery time and continue with fence breach checks
            _manual_recovery_start_ms = 0;
        }
    }

    // altitude fence check
    if ((_enabled_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {

        // check if we are over the altitude fence
        if( curr_alt >= _alt_max ) {

            // record distance above breach
            _alt_max_breach_distance = curr_alt - _alt_max;

            // check for a new breach or a breach of the backup fence
            if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) == 0 || (!is_zero(_alt_max_backup) && curr_alt >= _alt_max_backup)) {

                // record that we have breached the upper limit
                record_breach(AC_FENCE_TYPE_ALT_MAX);
                ret |= AC_FENCE_TYPE_ALT_MAX;

                // create a backup fence 20m higher up
                _alt_max_backup = curr_alt + AC_FENCE_ALT_MAX_BACKUP_DISTANCE;
            }
        }else{
            // clear alt breach if present
            if ((_breached_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
                clear_breach(AC_FENCE_TYPE_ALT_MAX);
                _alt_max_backup = 0.0f;
                _alt_max_breach_distance = 0.0f;
            }
        }
    }

    // circle fence check
    if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) != 0 ) {

        // check if we are outside the fence
        if (_home_distance >= _circle_radius) {

            // record distance outside the fence
            _circle_breach_distance = _home_distance - _circle_radius;

            // check for a new breach or a breach of the backup fence
            if ((_breached_fences & AC_FENCE_TYPE_CIRCLE) == 0 || (!is_zero(_circle_radius_backup) && _home_distance >= _circle_radius_backup)) {

                // record that we have breached the circular distance limit
                record_breach(AC_FENCE_TYPE_CIRCLE);
                ret |= AC_FENCE_TYPE_CIRCLE;

                // create a backup fence 20m further out
                _circle_radius_backup = _home_distance + AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE;
            }
        }else{
            // clear circle breach if present
            if ((_breached_fences & AC_FENCE_TYPE_CIRCLE) != 0) {
                clear_breach(AC_FENCE_TYPE_CIRCLE);
                _circle_radius_backup = 0.0f;
                _circle_breach_distance = 0.0f;
            }
        }
    }

    // polygon fence check
    if ((_enabled_fences & AC_FENCE_TYPE_POLYGON) != 0 ) {
        // check consistency of number of points
        if (_boundary_num_points != _total) {
            _boundary_loaded = false;
        }
        // load fence if necessary
        if (!_boundary_loaded) {
            load_polygon_from_eeprom();
        } else if (_boundary_valid) {
            // check if vehicle is outside the polygon fence
            const Vector3f& position = _inav.get_position();
            Vector2f pos2(position.x, position.y);

            if (_poly_loader.boundary_breached(pos2, _boundary_num_points, _boundary, true)) {

                _poly_breach_distance = distance_to_poly_min(&pos2, _boundary_num_points, _boundary) / 100.;

                // check if this is a new breach
                if ((_breached_fences & AC_FENCE_TYPE_POLYGON) == 0) {
                    // record that we have breached the polygon
                    record_breach(AC_FENCE_TYPE_POLYGON);
                    ret |= AC_FENCE_TYPE_POLYGON;
                }
            } else {
                // clear breach if present
                if ((_breached_fences & AC_FENCE_TYPE_POLYGON) != 0) {
                    clear_breach(AC_FENCE_TYPE_POLYGON);
                    _poly_breach_distance = 0.0;
                }
            }
        }
    }

    // return any new breaches that have occurred
    return ret;
}

// returns true if the destination is within fence (used to reject waypoints outside the fence)
bool AC_Fence::check_destination_within_fence(const Location_Class& loc)
{
    // Altitude fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX)) {
        int32_t alt_above_home_cm;
        if (loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, alt_above_home_cm)) {
            if ((alt_above_home_cm * 0.01f) > _alt_max) {
                return false;
            }
        }
    }

    // Circular fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_CIRCLE)) {
        if ((get_distance_cm(_ahrs.get_home(), loc) * 0.01f) > _circle_radius) {
            return false;
        }
    }

    // polygon fence check
    if ((get_enabled_fences() & AC_FENCE_TYPE_POLYGON) && _boundary_num_points > 0) {
        // check ekf has a good location
        Location temp_loc;
        if (_inav.get_location(temp_loc)) {
            const struct Location &ekf_origin = _inav.get_origin();
            Vector2f position = location_diff(ekf_origin, loc) * 100.0f;
            if (_poly_loader.boundary_breached(position, _boundary_num_points, _boundary, true)) {
                return false;
            }
        }
    }

    return true;
}

/// record_breach - update breach bitmask, time and count
void AC_Fence::record_breach(uint8_t fence_type)
{
    // if we haven't already breached a limit, update the breach time
    if (_breached_fences == AC_FENCE_TYPE_NONE) {
        _breach_time = AP_HAL::millis();
    }

    // update breach count
    if (_breach_count < 65500) {
        _breach_count++;
    }

    // update bitmask
    _breached_fences |= fence_type;
}

/// clear_breach - update breach bitmask, time and count
void AC_Fence::clear_breach(uint8_t fence_type)
{
    // return immediately if this fence type was not breached
    if ((_breached_fences & fence_type) == 0) {
        return;
    }

    // update bitmask
    _breached_fences &= ~fence_type;
}

/// get_breach_distance - returns distance in meters outside of the given fence
float AC_Fence::get_breach_distance(uint8_t fence_type) const
{
    switch (fence_type) {
        case AC_FENCE_TYPE_ALT_MAX:
            return _alt_max_breach_distance;
            break;
        case AC_FENCE_TYPE_CIRCLE:
            return _circle_breach_distance;
            break;
        case AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE:
            return MAX(_alt_max_breach_distance, _circle_breach_distance);
            break;
        case AC_FENCE_TYPE_POLYGON:
            return _poly_breach_distance;
            break;
        case AC_FENCE_TYPE_POLYGON | AC_FENCE_TYPE_CIRCLE:
            return MAX(_poly_breach_distance, _circle_breach_distance);
            break;
        case AC_FENCE_TYPE_POLYGON | AC_FENCE_TYPE_ALT_MAX:
            return MAX(_poly_breach_distance, _alt_max_breach_distance);
            break;
        case AC_FENCE_TYPE_POLYGON | AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE:
            return MAX(_alt_max_breach_distance, MAX(_poly_breach_distance, _circle_breach_distance));
            break;
    }

    // we don't recognise the fence type so just return 0
    return 0;
}

/// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds
///     has no effect if no breaches have occurred
void AC_Fence::manual_recovery_start()
{
    // return immediate if we haven't breached a fence
    if (_breached_fences == AC_FENCE_TYPE_NONE) {
        return;
    }

    // record time pilot began manual recovery
    _manual_recovery_start_ms = AP_HAL::millis();
}

/// returns pointer to array of polygon points and num_points is filled in with the total number
Vector2f* AC_Fence::get_polygon_points(uint16_t& num_points) const
{
    num_points = _boundary_num_points;
    return _boundary;
}

/// returns true if we've breached the polygon boundary.  simple passthrough to underlying _poly_loader object
bool AC_Fence::boundary_breached(const Vector2f& location, uint16_t num_points, const Vector2f* points) const
{
    return _poly_loader.boundary_breached(location, num_points, points, true);
}

/// handler for polygon fence messages with GCS
void AC_Fence::handle_msg(mavlink_channel_t chan, mavlink_message_t* msg)
{
    // exit immediately if null message
    if (msg == nullptr) {
        return;
    }

    switch (msg->msgid) {
        // receive a fence point from GCS and store in EEPROM
        case MAVLINK_MSG_ID_FENCE_POINT: {
            mavlink_fence_point_t packet;
            mavlink_msg_fence_point_decode(msg, &packet);
            if (!check_latlng(packet.lat,packet.lng)) {
                GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_WARNING, chan, "Invalid fence point, lat or lng too large");
            } else {
                Vector2l point;
                point.x = packet.lat*1.0e7f;
                point.y = packet.lng*1.0e7f;
                if (!_poly_loader.save_point_to_eeprom(packet.idx, point)) {
                    GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_WARNING, chan, "Failed to save polygon point, too many points?");
                } else {
                    // trigger reload of points
                    _boundary_loaded = false;
                }
            }
            break;
        }

        // send a fence point to GCS
        case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
            mavlink_fence_fetch_point_t packet;
            mavlink_msg_fence_fetch_point_decode(msg, &packet);
            // attempt to retrieve from eeprom
            Vector2l point;
            if (_poly_loader.load_point_from_eeprom(packet.idx, point)) {
                mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, _total, point.x*1.0e-7f, point.y*1.0e-7f);
            } else {
                GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_WARNING, chan, "Bad fence point");
            }
            break;
        }

        default:
            // do nothing
            break;
    }
}

/// handler for polygon fence messages with GCS
bool AC_Fence::handle_msg_data32(mavlink_channel_t chan, const mavlink_data32_t &packet)
{

    // receive a fence point from GCS and store in EEPROM

    // check type 61
    if (packet.type != 61) {
        return false;
    }

    bool ret = true;


    for (uint8_t i = 0; i < 3; i++) {
        uint8_t offset = i * 9;
        int32_t lat = (packet.data[offset + 1] << 24) |
                        (packet.data[offset + 2] << 16) |
                        (packet.data[offset + 3] << 8) |
                        packet.data[offset + 4];
        
        int32_t lng = (packet.data[offset + 5] << 24) |
                        (packet.data[offset + 6] << 16) |
                        (packet.data[offset + 7] << 8) |
                        packet.data[offset + 8];
        
        uint8_t idx = packet.data[offset + 9];

        if (lat == 0 && lng == 0 && idx == 0) {
            continue;
        }

        if (!check_latlng(lat, lng)) {
            GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_WARNING, chan, "Invalid fence point, lat or lng too large");
            ret = false;
            break;
        } else {
            Vector2l point;
            point.x = lat;
            point.y = lng;
            if (!_poly_loader.save_point_to_eeprom(idx, point)) {
                GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_WARNING, chan, "Failed to save polygon point, too many points?");
                ret = false;
                break;
            } else {
                // trigger reload of points
                _boundary_loaded = false;
            }
        }
    }

    return ret;
}

/// load polygon points stored in eeprom into boundary array and perform validation
bool AC_Fence::load_polygon_from_eeprom(bool force_reload)
{
    // exit immediately if already loaded
    if (_boundary_loaded && !force_reload) {
        return true;
    }

    // check if we need to create array
    if (!_boundary_create_attempted) {
        _boundary = (Vector2f *)_poly_loader.create_point_array(sizeof(Vector2f));
        _boundary_create_attempted = true;
    }

    // exit if we could not allocate RAM for the boundary
    if (_boundary == nullptr) {
        return false;
    }

    // get current location from EKF
    Location temp_loc;
    if (!_inav.get_location(temp_loc)) {
        return false;
    }
    const struct Location &ekf_origin = _inav.get_origin();

    // sanity check total
    _total = constrain_int16(_total, 0, _poly_loader.max_points());

    // load each point from eeprom
    Vector2l temp_latlon;
    for (uint16_t index=0; index<_total; index++) {
        // load boundary point as lat/lon point
        _poly_loader.load_point_from_eeprom(index, temp_latlon);
        // move into location structure and convert to offset from ekf origin
        temp_loc.lat = temp_latlon.x;
        temp_loc.lng = temp_latlon.y;
        _boundary[index] = location_diff(ekf_origin, temp_loc) * 100.0f;
        _polygon_crc = 0;
    }
    _boundary_num_points = _total;
    _boundary_loaded = true;

    // update validity of polygon
    _boundary_valid = _poly_loader.boundary_valid(_boundary_num_points, _boundary, true);

    return true;
}


bool AC_Fence::hard_fence_breach() {

    if (_hard_fence_enable <= 0 || _hard_dist <= 0 || _breached_fences == AC_FENCE_TYPE_NONE) {
        return false;
    }

    if (get_breach_distance(_breached_fences) > _hard_dist) {

        if (!_hard_fence_breach) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Hard Fence breach. Count down before disarm!!!");
            _hard_fence_breach_time = AP_HAL::millis();
            _hard_fence_breach = true;
        }

        if ( AP_HAL::millis() - _hard_fence_breach_time > AC_FENCE_HARD_BREACH_TIMEOUT) {
            return true;
        } else {
            return false;
        }

    } else {
        _hard_fence_breach = false;
        return false;
    }
}


float AC_Fence::distance_to_poly_min(Vector2f *pos, uint8_t boundary_num, Vector2f *boundary) {

    Vector2f *startPoint = nullptr;
    Vector2f *endPoint = nullptr;
    float t = 0.0;
    float dist = 0.0;
    float min_dist = 9999999;

    for (uint8_t i = 0; i < boundary_num; i++) {

        if (i == 0) {
            startPoint = &boundary[i];
            continue;
        }

        endPoint = &boundary[i];

        t = (pos->x - startPoint->x)*(endPoint->x - startPoint->x) + (pos->y - startPoint->y)*(endPoint->y - startPoint->y);
        t /= ((endPoint->x - startPoint->x) * (endPoint->x - startPoint->x)) + ((endPoint->y - startPoint->y)*(endPoint->y - startPoint->y));

        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }

        dist = sqrt( sq(startPoint->x - pos->x + (endPoint->x - startPoint->x)*t) + sq(startPoint->y - pos->y + (endPoint->y - startPoint->y)*t) );

        if (min_dist > dist) {
            min_dist = dist;
        }

        startPoint = endPoint;
    }

    return min_dist;
}

uint16_t AC_Fence::calc_fence_crc() {
    uint16_t crc = 0;
    uint16_t alt_max_loc = 0;
    uint16_t circle_radius_loc = 0;

    if (!enabled()) {
        return 0;
    }

    // fence action check
    uint16_t crc_fence_action;
    crc_fence_action = _action;
    crc = crc16_ccitt((uint8_t*)&(crc_fence_action), 2, crc);

    uint16_t enabled_fences;
    // fence type check
    enabled_fences = get_enabled_fences();
    crc = crc16_ccitt((uint8_t*)&(enabled_fences), 2, crc);
        
    // fence alt check
    if ((enabled_fences & AC_FENCE_TYPE_ALT_MAX)) {
        alt_max_loc = _alt_max * 100;
        crc = crc16_ccitt((uint8_t*)&(alt_max_loc), 2, crc);
    } 
    
    // Circular fence check
    if ((enabled_fences & AC_FENCE_TYPE_CIRCLE)) {
         circle_radius_loc = _circle_radius * 100;
         crc = crc16_ccitt((uint8_t*)&(circle_radius_loc), 2, crc);
    }

    // Hard fence flag check
    if (_hard_fence_enable == 1) {
        uint16_t enabled_hard_fence = _hard_fence_enable;
        crc = crc16_ccitt((uint8_t*)&(enabled_hard_fence), 2, crc);

        // Hard fence distance check
        int16_t hard_dist = round(_hard_dist);
        crc = crc16_ccitt((uint8_t*)&(hard_dist), 2, crc);
    }

    // polygon fence check
    if ((enabled_fences & AC_FENCE_TYPE_POLYGON) && _boundary_num_points > 0 ) {

        if (_polygon_crc == 0) { // Recalculate polygon crc
            Vector2l temp_latlon;
            uint16_t polygon_crc = 0;
            // do not count zero point cause it is not polyfence point actually
            for (uint16_t index = 1; index < _boundary_num_points; index++) {
                // load boundary point as lat/lon point
                _poly_loader.load_point_from_eeprom(index, temp_latlon);
                // low the accuracy
                polygon_crc = crc16_ccitt((uint8_t*)&(temp_latlon.x), 4, polygon_crc);
                polygon_crc = crc16_ccitt((uint8_t*)&(temp_latlon.y), 4, polygon_crc);
            }
            _polygon_crc = polygon_crc;
        } else {
           crc = crc16_ccitt((uint8_t*)&(_polygon_crc), 2, crc);
        }
    }

    return crc;
}
