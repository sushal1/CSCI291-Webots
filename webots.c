#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Variables to track the brightest light source
double max_light_value = 0;
double max_light_x = 0;
double max_light_y = 0;
double max_light_z = 0;

// States for robot behavior
typedef enum { SEARCHING, NAVIGATING_TO_LIGHT, STOPPED } RobotState;

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag light_sensors[8];
  char light_sensor_name[50];
  for (int i = 0; i < 8; i++) {
    sprintf(light_sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(light_sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

  // Initializing the motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Setting initial positions & velocities for the motors
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag proximity_sensors[8];
  char proximity_sensor_name[50];
  for (int i = 0; i < 8; i++) {
    sprintf(proximity_sensor_name, "ps%d", i);
    proximity_sensors[i] = wb_robot_get_device(proximity_sensor_name);
    wb_distance_sensor_enable(proximity_sensors[i], TIME_STEP);
  }

  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;

  // Variables to track initial position and stopping condition
  double initial_x = 0, initial_y = 0, initial_z = 0;
  int initialized = 0;  // Flag to ensure initial position is recorded only once
  int steps_moved = 0;  // Count the number of steps the robot has moved

  RobotState state = SEARCHING;  // Initialize the state

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *position = wb_gps_get_values(gps);

    // Record the initial position
    if (initialized == 0) {
      initial_x = position[0];
      initial_y = position[1];
      initial_z = position[2];
      initialized = 1;
      printf("Initial position recorded: x = %f, y = %f, z = %f\n", initial_x, initial_y, initial_z);
    }

    if (state == SEARCHING) {
      double lightval = wb_light_sensor_get_value(light_sensors[7]);
      if (lightval > 4000 && lightval > max_light_value) {
        max_light_value = lightval;
        max_light_x = position[0];
        max_light_y = position[1];
        max_light_z = position[2];
        printf("Max light value: %lf at (%lf, %lf, %lf)\n", max_light_value, max_light_x, max_light_y, max_light_z);
      }

      // Increment the step counter
      steps_moved++;

      // Stop if the robot returns close to the initial position after moving sufficiently
      if (steps_moved > 500 && fabs(position[0] - initial_x) < 0.01 && fabs(position[2] - initial_z) < 0.01) {
        printf("Robot has returned to the initial position. Stopping.\n");
        left_speed = 0;
        right_speed = 0;
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);

        state = NAVIGATING_TO_LIGHT;  // Transition to next state
        continue;
      }

      // Sensor readings for wall detection
      bool left_wall = wb_distance_sensor_get_value(proximity_sensors[5]) > 100;
      bool left_corner = wb_distance_sensor_get_value(proximity_sensors[6]) > 100;
      bool front_wall = wb_distance_sensor_get_value(proximity_sensors[7]) > 100;

      // Movement logic (unchanged)
      // Movement logic
      if (front_wall) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
      } else if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else if (left_corner) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 5;
      } else {
        left_speed = MAX_SPEED / 5;
        right_speed = MAX_SPEED;
      }
  
      // Set motor speeds
      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);

    } else if (state == NAVIGATING_TO_LIGHT) {
      // Sensor readings for wall detection
      bool left_wall = wb_distance_sensor_get_value(proximity_sensors[5]) > 100;
      bool left_corner = wb_distance_sensor_get_value(proximity_sensors[6]) > 100;
      bool front_wall = wb_distance_sensor_get_value(proximity_sensors[7]) > 100;

      // Follow the maze path to the brightest light
          // Movement logic
      if (front_wall) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
      } else if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else if (left_corner) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 5;
      } else {
        left_speed = MAX_SPEED / 5;
        right_speed = MAX_SPEED;
      }

      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);

      // Stop when close to the light source
      double dx = max_light_x - position[0];
      double dz = max_light_z - position[2];
      double distance = sqrt(dx * dx + dz * dz);
      if (distance < 0.01) {
        printf("Reached the brightest light source. Stopping.\n");
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        state = STOPPED;  // Transition to stopped state
        break;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}



