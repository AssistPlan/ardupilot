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
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

MultiCopter::MultiCopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(nullptr)
{
    mass = 1.5f;

    gripper.set_aircraft(this);

    frame = Frame::find_frame(frame_str);
    if (frame == nullptr) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    // initial mass is passed through to Frame for it to calculate a
    // hover thrust requirement.
    if (strstr(frame_str, "-fast")) {
        frame->init(gross_mass(), 0.5, 85, 4*radians(360));
    } else {
        frame->init(gross_mass(), 0.51, 15, 4*radians(360));
    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    max_speed =20;
    max_accel = 10;
    max_wheel_turn = 35;
    turning_circle = 1.8;
    skid_turn_rate = 140; // degrees/sec


}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
}
 


/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float MultiCopter::turn_circle(float steering)
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float MultiCopter::calc_yaw_rate(float steering, float speed)
{
//    if (skid_steering) {
//        return steering * skid_turn_rate;
//    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}


void MultiCopter::update_rover(const struct sitl_input &input)
{
    float steering, throttle;

    // if in skid steering mode the steering and throttle values are used for motor1 and motor2
//    if (skid_steering) {
//        float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
//        float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
//        steering = motor1 - motor2;
//        throttle = 0.5*(motor1 + motor2);
//    } else {
      steering = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
      //throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
      if(input.servos[3] > 1120)
      {
          throttle = ((input.servos[3]-1120)/500.0f);
      } else {
          throttle = 0;
      }
//    }

//printf("input.servos=%d,%d,%d,%d\n", input.servos[0], input.servos[1], input.servos[2], input.servos[3]);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

//printf("velocity_ef=%f,%f,%f\n", velocity_ef.x, velocity_ef.y, velocity_ef.z);

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    // target speed with current throttle
    float target_speed = throttle * max_speed;

//printf("target_speed=%f, throttle=%f, max_speed=%f\n", target_speed, throttle, max_speed);

    // linear acceleration in m/s/s - very crude model
    float accel = max_accel * (target_speed - speed) / max_speed;

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor
    accel_body = Vector3f(accel, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += velocity_ef * delta_time;

    position.z = 0.0f;//


//printf("position=%f,%f\n", position.x , position.y);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

}   
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{


    Vector3f rot_accel;
//printf("input.servos=%d,%d,%d,%d\n", input.servos[0], input.servos[1], input.servos[2], input.servos[3]);

//printf("MultiCopter::update rover_mode=%d\n", rover_mode);

    if(rover_mode == true)
    {
        update_rover(input);
    } else {
        // get wind vector setup
        update_wind(input);
        calculate_forces(input, rot_accel, accel_body);
        update_dynamics(rot_accel);

        // update lat/lon/altitude
        update_position();
        time_advance();

        // update magnetic field
        update_mag_field_bf();

        // update sprayer
        sprayer.update(input);

        // update gripper
        gripper.update(input);
        gripper_epm.update(input);
    }
}

float MultiCopter::gross_mass() const
{
    return Aircraft::gross_mass() + sprayer.payload_mass() + gripper.payload_mass();
}
