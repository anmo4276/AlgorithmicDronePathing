/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();


  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }



  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }


  // Constants, empirically found.
  const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.
  // Variables.
  double target_altitude = 2;  // The target altitude. Can be changed by the user.

  bool reachH= false;
  //Test1
  // bool reachRot[9] ={false,false,false,false,false,false,false,false,false};
  //Test2
  bool reachRot[11] ={false,false,false,false,false,false,false,false,false,false,false};
  double startX= wb_gps_get_values(gps)[0];
  double startY = wb_gps_get_values(gps)[2];
  //Test1 (START MAVIC AT 25,29.333)
  //double waypoints[9][2] ={{22.75919315571183, 24.76181447341475},{20.008727509627132, 20.47754805666288},{17.33580676972095, 16.14447816129085},{14.507980826369828, 11.910874444112023},{11.979646305033723, 10.999358031017705},{7.149095865842506, 12.607394863440875},{4.991523596966767, 7.9960087481195385}, {6.683559792458655, 3.194236834216781},{8.333333333333334, 2.6666666666666665}};
  //Test2 (START MAVIC AT 6.666,32.666)
  double waypoints[11][2] ={{11.053105738058802, 34.5440970329819},{16.10265598948042, 35.193744828612616},{20.95213009998395, 33.64371229619402},{23.597776430737113, 29.293935808537174},{25.820520680892123, 24.71360743620372},{27.815276680938158, 20.029490817215635},{32.52356716611721, 21.891800656711766}, {34.93299498956042, 19.913091883974175},{34.2288481888565, 14.870852571619155},{31.812899456078853, 10.389425151634525},{31.666666666666668, 12.666666666666666}};
  double targetx=waypoints[0][0];
  double targety=waypoints[0][1];
  double angle = atan2((targety - startY),(targetx - startX));
  double rotX = cos(angle);
  double rotY = sin(angle);
  int index=0;
  //Test1
  //double angleArr[9]={0,0,0,0,0,0,0,0,0};
  double angleArr[11]={0,0,0,0,0,0,0,0,0,0,0};
  angleArr[0] = angle;
  //Test1
  //bool xThere[9] = {false,false,false,false,false,false,false,false,false};
  //Test2
  bool xThere[11] = {false,false,false,false,false,false,false,false,false,false,false};

  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();  // in seconds.
    //printf("%f\n", targety);

    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[1];
    const double roll_acceleration = wb_gyro_get_values(gyro)[0];
    const double pitch_acceleration = wb_gyro_get_values(gyro)[1];

    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_acceleration);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_acceleration);

    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;

    // If the desired height is reached
    if(!reachH && wb_gps_get_values(gps)[1]>=(target_altitude-.05)){
        reachH = true;
    }

    // If the rotation has been made to go onto the next waypoint
    if(reachH && !reachRot[index] && index!=11){
      if(wb_compass_get_values(compass)[0]<=rotX+0.05 && wb_compass_get_values(compass)[0]>=rotX-.05 && wb_compass_get_values(compass)[1]<=rotY+.05 && wb_compass_get_values(compass)[1]>=rotY-.05){
        reachRot[index]=true;
          yaw_disturbance = 0;
      }

      // If not then keep rotating
      else{

        // If and else if statements to find out which way it should rotate based on the previous angle
          if(index==0){
            yaw_disturbance = .3;
          }
          else if(angleArr[index]>1.5 && angleArr[index-1]<-1.5){
            yaw_disturbance = -.3;
          }
          else if(angleArr[index]<-1.5 && angleArr[index-1]>1.5){
            yaw_disturbance = .3;
          }
          else if(angleArr[index]<angleArr[index-1]){
            yaw_disturbance = -.3;
          }
          else{
            yaw_disturbance = .3;
          }
      }
   }

   // If the rotation has been reached but the waypoint is still not reached
    if(reachRot[index] && !xThere[index] &&reachH){
      // If the waypoint has not been reached keep moving forward
        if(!(wb_gps_get_values(gps)[0]+.5>targetx) || !(wb_gps_get_values(gps)[0]-.5<targetx)){
          pitch_disturbance = 2.0;
        }
        // Once reached calculate the next angle for the next waypoint
        else{
          pitch_disturbance = 0;
          xThere[index] = true;
          index++;
          // Next waypoint x and y coords
          targetx=waypoints[index][0];
          targety=waypoints[index][1];
          printf("Waypoint %d\n",index);
          // Calculate the angle and store in the angle array
          angle = atan2((targety - wb_gps_get_values(gps)[2]),(targetx - wb_gps_get_values(gps)[0]));
          angleArr[index]=angle;
          rotX = cos(angle);
          rotY = sin(angle);
          }
    }

    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
