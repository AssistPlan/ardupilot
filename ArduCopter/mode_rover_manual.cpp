#include "Copter.h"

#include <AP_Motors/AP_MotorsMulticopter.h>
// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Copter::ModeRoverManual::get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out)
{
    // do basic conversion
//    get_pilot_input(steering_out, throttle_out);

// get pilot's desired yaw rate
    steering_out = channel_yaw->get_control_in();

    // get pilot's desired throttle
    throttle_out = channel_throttle->get_control_in();


    // check for special case of input and output throttle being in opposite directions
    //float throttle_out_limited = g2.motors.get_slew_limited_throttle(throttle_out, rover.G_Dt);

//    if ((is_negative(throttle_out) != is_negative(throttle_out_limited)) &&
//        ((g.pilot_steer_type == PILOT_STEER_TYPE_DEFAULT) ||
//         (g.pilot_steer_type == PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING))) {
//        steering_out *= -1;
//    }
//    throttle_out = throttle_out_limited;
}
/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeRoverManual::init(bool ignore_checks)
{

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
printf("ap.land_complete = %d\n", ap.land_complete);
    if (ap.land_complete == false)
    {

        return false;
    }


    // Set wp navigation target to safe altitude above current position
//    copter.auto_rover_mode = true;
      copter.g2.ugv_motors.init();
//    copter.set_not_dis_arm_on_land(true);

    copter.g2.ugv_motors.stop_motors();
    copter.set_tilt_type(TILT_TYPE_UP);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    copter.sitl.set_sitl_rover_model(true);
#endif

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeRoverManual::run()
{

    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    float pilot_steering_scaled = (target_yaw_rate + 20250) / (20250*2);
    //float pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
    float pilot_throttle_scaled = channel_throttle->get_control_in() / 1000.0;


//    float desired_steering, desired_throttle, desired_lateral;
//    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
//    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
//    if (rover.is_balancebot()) {
//        rover.balancebot_pitch_control(desired_throttle, rover.arming.is_armed());
//    }

    // copy RC scaled inputs to outputs
    //g2.motors.set_throttle(desired_throttle);

//printf("pilot_throttle_scaled=%f\n", pilot_throttle_scaled);
//printf("pilot_steering_scaled=%f\n", pilot_steering_scaled);
    if(pilot_throttle_scaled < 0.05)
    {
        pilot_throttle_scaled = 0.0;
    }
    copter.g2.ugv_motors.set_throttle((pilot_throttle_scaled) * 200.0f - 100);
    //copter.g2.ugv_motors.set_steering((pilot_steering_scaled - 0.5) * 4500.0f);
    copter.g2.ugv_motors.set_steering((pilot_steering_scaled - 0.5) * 9000.0f);

//    g2.motors.set_steering(desired_steering, false);
//    g2.motors.set_lateral(desired_lateral);


/*
    //float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
    float pilot_steering_scaled;

   // float desired_steering, desired_throttle, desired_lateral;

// get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    pilot_steering_scaled = (target_yaw_rate + 20250) / (20250*2);
    //target_yaw_rate
    //  3375 1600 RC4
    //-16029 1100 RC4
    //     0 1500 RC4
    // get pilot's desired throttle
    rover_motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    rover_motors->set_spool_mode (AP_MotorsMulticopter::THROTTLE_UNLIMITED);
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

printf("target_yaw_rate[%f]\n", target_yaw_rate);
    // 0 -- > 1.0   pilot_throttle_scaled
    // 2000 ->> 1000  RC3

    // call attitude controller
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, target_yaw_rate);

    //attitude_control->set_throttle_out(pilot_throttle_scaled, true, 0);

    rover_motors->set_thrust_rear(pilot_throttle_scaled);
    rover_motors->set_thrust_staring(pilot_steering_scaled);

*/


}


