#include "Copter.h"

#if MODE_AUTO_ENABLED == ENABLED

/*
    calculate steering output given lateral_acceleration
*/
void Copter::ModeAuto::calc_steering_from_lateral_acceleration(float lat_accel, bool reversed)
{

//    float turn_angle;
//    float turn_max_g;
    //bool limit_steer_left;
    //bool limit_steer_right;
    //float rover_G_Dt;

    //turn_angle = 0.0;
    //turn_max_g = 0.0;
    //limit_steer_left = true;
    //limit_steer_right = true;
    //rover_G_Dt = 1.0;
     
    // add obstacle avoidance response to lateral acceleration target
    // ToDo: replace this type of object avoidance with path planning
//    if (!reversed) {
//        lat_accel += (turn_angle / 45.0f) * turn_max_g;
//    }

   
    // constrain to max G force
    lat_accel = constrain_float(lat_accel, -copter.g2.turn_max_g * GRAVITY_MSS, copter.g2.turn_max_g * GRAVITY_MSS);

    // send final steering command to motor library
    //const float steering_out = 0;
    const float steering_out = copter.g2.attitude_control_rover.get_steering_out_lat_accel(lat_accel,
                                                                           copter.g2.ugv_motors.limit.steer_left,
                                                                           copter.g2.ugv_motors.limit.steer_right,
                                                                           copter.G_Dt);
    //printf("steering_out= [%f]\n", steering_out );

    //g2.motors.set_steering(steering_out * 4500.0f);
    copter.g2.ugv_motors.set_steering(steering_out * 4500.0f);

}

void Copter::ModeAuto::calc_steering_to_waypoint(const struct Location &origin, const struct Location &destination, bool reversed)
{

    //uint32_t last_steer_to_wp_ms;
    //float _yaw_error_cd;

    // record system time of call
    //last_steer_to_wp_ms = AP_HAL::millis();

    // Calculate the required turn of the wheels
    // negative error = left turn
    // positive error = right turn
    copter.nav_controller->set_reverse(reversed);
    copter.nav_controller->update_waypoint(origin, destination, g2.waypoint_radius);
    float desired_lat_accel = copter.nav_controller->lateral_acceleration();
    float desired_heading = copter.nav_controller->target_bearing_cd();

//printf("desired_lat_accel=%f\n", desired_lat_accel);

    if (reversed) {
        desired_heading = wrap_360_cd(desired_heading + 18000);
        desired_lat_accel *= -1.0f;
    }
    //_yaw_error_cd = wrap_180_cd(desired_heading - ahrs.yaw_sensor);

//    if (copter.use_pivot_steering(_yaw_error_cd)) {
        // for pivot turns use heading controller
//        calc_steering_to_heading(desired_heading, g2.pivot_turn_rate);
//    } else {
        // call lateral acceleration to steering controller
        calc_steering_from_lateral_acceleration(desired_lat_accel, reversed);
//    }
}



// get default speed for this mode (held in (CRUISE_SPEED, WP_SPEED or RTL_SPEED)
float Copter::ModeAuto::get_speed_default(bool rtl) const
{
    //float g2_rtl_speed;
    //float g2_wp_speed;
    //float g_speed_cruise;

    //g2_rtl_speed = 1.0;
    //g2_wp_speed = 1.0;
    //g_speed_cruise = 1.0;
    
    if (rtl && is_positive(g2.rtl_speed)) {
        return g2.rtl_speed;
    } else if (is_positive(g2.wp_speed)) {
        return g2.wp_speed;
    } else {
        return g2.speed_cruise;
    }
}

// restore desired speed to default from parameter values (CRUISE_SPEED or WP_SPEED)
void Copter::ModeAuto::set_desired_speed_to_default(bool rtl)
{
    _desired_speed = get_speed_default(rtl);
}

float Copter::ModeAuto::calc_reduced_speed_for_turn_or_distance(float desired_speed)
{

//    float copter_g_waypoint_overshoot;
//    float g_turn_max_g;
//    float g2_turn_radius;
//    g2_turn_radius = 1.0;

//    g_turn_max_g = 1.0;

//    copter_g_waypoint_overshoot = 10;
    // reduce speed to zero during pivot turns
    //if (rover.use_pivot_steering(_yaw_error_cd)) {
    //    return 0.0f;
    //}

    // reduce speed to limit overshoot from line between origin and destination
    // calculate number of degrees vehicle must turn to face waypoint
    const float heading_cd = is_negative(desired_speed) ? wrap_180_cd(ahrs.yaw_sensor + 18000) : ahrs.yaw_sensor;
    const float wp_yaw_diff = wrap_180_cd(copter.nav_controller->target_bearing_cd() - heading_cd);
    const float turn_angle_rad = fabsf(radians(wp_yaw_diff * 0.01f));

    // calculate distance from vehicle to line + wp_overshoot
    const float line_yaw_diff = wrap_180_cd(get_bearing_cd(_origin, _destination) - heading_cd);
    const float lcrosstrack_error = copter.nav_controller->crosstrack_error();
    const float dist_from_line = fabsf(lcrosstrack_error);
    const bool heading_away = is_positive(line_yaw_diff) == is_positive(lcrosstrack_error);
    const float wp_overshoot_adj = heading_away ? -dist_from_line : dist_from_line;

    // calculate radius of circle that touches vehicle's current position and heading and target position and heading
    float radius_m = 999.0f;
    float radius_calc_denom = fabsf(1.0f - cosf(turn_angle_rad));
    if (!is_zero(radius_calc_denom)) {
        radius_m = MAX(0.0f, copter.g2.waypoint_overshoot + wp_overshoot_adj) / radius_calc_denom;
    }

    // calculate and limit speed to allow vehicle to stay on circle
    float overshoot_speed_max = safe_sqrt(g2.turn_max_g * GRAVITY_MSS * MAX(g2.turn_radius, radius_m));
    float speed_max = constrain_float(desired_speed, -overshoot_speed_max, overshoot_speed_max);

    // limit speed based on distance to waypoint and max acceleration/deceleration
    if (is_positive(_distance_to_destination) && is_positive(copter.g2.attitude_control_rover.get_decel_max())) {
        const float dist_speed_max = safe_sqrt(2.0f * _distance_to_destination * copter.g2.attitude_control_rover.get_decel_max() + sq(_desired_speed_final));
        speed_max = constrain_float(speed_max, -dist_speed_max, dist_speed_max);
    }

    // return minimum speed
    return speed_max;
}

void Copter::ModeAuto::calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled)
{

//    float copter_G_Dt;
//    copter_G_Dt = 1.0;
//    float  g2_motors_limit_throttle_lower;
//    float  g2_motors_limit_throttle_upper;
//    float  g_speed_cruise;
//    float  g_throttle_cruise;

//    g2_motors_limit_throttle_lower = 0.0;
//    g2_motors_limit_throttle_upper = 1.0;
//    g_speed_cruise = 0.5;
//    g_throttle_cruise = 0.5;

    // add in speed nudging
    //if (nudge_allowed) {
    //    target_speed = calc_speed_nudge(target_speed, g.speed_cruise, g.throttle_cruise * 0.01f);
    //}

    // get acceleration limited target speed
    target_speed = copter.g2.attitude_control_rover.get_desired_speed_accel_limited(target_speed, copter.G_Dt);

    // apply object avoidance to desired speed using half vehicle's maximum deceleration
//    if (avoidance_enabled) {
//        g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_decel_max(), ahrs.yaw, target_speed, rover.G_Dt);
//    }

    // call throttle controller and convert output to -100 to +100 range
    float throttle_out;

    // call speed or stop controller
    if (is_zero(target_speed)) {
        bool stopped;
        throttle_out = 100.0f * copter.g2.attitude_control_rover.get_throttle_out_stop(copter.g2.ugv_motors.limit.throttle_lower, copter.g2.ugv_motors.limit.throttle_upper, g2.speed_cruise, g2.throttle_cruise * 0.01f, copter.G_Dt, stopped);
    } else {
        throttle_out = 100.0f * copter.g2.attitude_control_rover.get_throttle_out_speed(target_speed, copter.g2.ugv_motors.limit.throttle_lower, copter.g2.ugv_motors.limit.throttle_upper, g2.speed_cruise, g2.throttle_cruise * 0.01f, copter.G_Dt);
    }

    // if vehicle is balance bot, calculate actual throttle required for balancing
//    if (rover.is_balancebot()) {
//        rover.balancebot_pitch_control(throttle_out, rover.arming.is_armed());
//    }

    // send to motor
//    g2.motors.set_throttle(throttle_out);

    copter.g2.ugv_motors.set_throttle(throttle_out - 100);
  //  printf("throttle_out= [%f]\n", throttle_out);

}

// set  x y to lat lng
bool Copter::ModeAuto::get_location(const Vector3f& offset, Location& location)
{
    Location destination_ned;
    // initialise destination to ekf origin
    if (!ahrs.get_origin(destination_ned)) {
        return false;
    }
    // apply offset
    
    location_offset(destination_ned, offset.x, offset.y);
    location = destination_ned;
    return true;
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::wp_rover_stop()
{
    copter.g2.ugv_motors.stop_motors();

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::wp_rover_run()
{


    

    //_reached_destination = false;
//    Vector3f origin;
//    Vector3f destination;
//    Vector3f current;

    _reversed = false;
    is_boat = true;
//    destination = copter.wp_nav->get_wp_destination();
//    origin = copter.wp_nav->get_wp_origin();

//    get_location(origin, _origin);
//    get_location(destination, _destination);


//printf("_origin.lat=%d, _origin.lng=%d\n", _origin.lat, _origin.lng);
//printf("_destination.lat=%d, _destination.lng=%d\n", _destination.lat, _destination.lng);


    _distance_to_destination = copter.wp_nav->get_wp_distance_to_destination();

    const bool near_wp = _distance_to_destination <= copter.g.k_param_waypoint_radius;

//printf("_distance_to_destination=%f, _reached_destination=%d\n", _distance_to_destination, _reached_destination);

    // check if we've reached the destination

    if (!_reached_destination && (near_wp || location_passed_point(copter.current_loc, _origin, _destination))) {
//printf("near_wp=[%d]\n", near_wp);
//printf("copter.current_loc[%d],[%d]\n", copter.current_loc.lat, copter.current_loc.lng);
//printf("_origin[%d],[%d]\n", _origin.lat, _origin.lng);
//printf("_destination[%d],[%d]\n", _destination.lat, _destination.lng);
//printf("location_passed_point[%d]\n", location_passed_point(copter.current_loc, _origin, _destination));

        // trigger reached
        _reached_destination = true;
    }

    // determine if we should keep navigating
    if (!_reached_destination || (is_boat && !near_wp)) {
        // continue driving towards destination
        calc_steering_to_waypoint(_reached_destination ? copter.current_loc : _origin, _destination, _reversed);
        calc_throttle(calc_reduced_speed_for_turn_or_distance(_reversed ? -_desired_speed : _desired_speed), true, false);
    } else {
        // we have reached the destination so stop
        //stop_vehicle();
    }

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

   
}

#endif

