#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/distance_sensor.h>

#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28


int main(int argc, char **argv) {
  wb_robot_init();
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
   
  
  // initialising the motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  // setting initial positions & velocities for the motors
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor,0.0);
  wb_motor_set_velocity(right_motor,0.0);
  
  WbDeviceTag proximity_sensors[8];
  char proximity_sensor_name[50];
  for (int i = 0; i<8; i++) {
    sprintf(proximity_sensor_name,"ps%d",i);
    proximity_sensors[i] = wb_robot_get_device(proximity_sensor_name);
    wb_distance_sensor_enable(proximity_sensors[i],TIME_STEP);
  }
  
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;

  while (wb_robot_step(TIME_STEP) != -1) {
  
    const double *position = wb_gps_get_values(gps);
    printf("Position: x = %f, y = %f, z = %f\n", position[0], position[1], position[2]);
    
    bool left_wall = wb_distance_sensor_get_value(proximity_sensors[5]) > 100;
    bool left_corner = wb_distance_sensor_get_value(proximity_sensors[6]) > 100;
    bool front_wall = wb_distance_sensor_get_value(proximity_sensors[7]) > 100;
    
    double leftwallsensor = wb_distance_sensor_get_value(proximity_sensors[5]);
    double leftcornersensor = wb_distance_sensor_get_value(proximity_sensors[6]);
    double frontwallsensor = wb_distance_sensor_get_value(proximity_sensors[7]);
    
    printf("Left wall: %lf\n",leftwallsensor);
    printf("Left corner: %lf\n",leftcornersensor);
    printf("Front wall: %lf\n",frontwallsensor);
    
    //int found = 0;
    //if ((position[0] == -1.11) && (position[2] == 0.08)) {
      //if (found != 0) {
        //printf("\noops\n");
        //left_speed = 0;
        //right_speed = 0;
      //} else {
        //found++;
        //wall_detector(left_speed, right_speed, left_wall,front_wall, left_corner);
      //}
    //} else {
      //wall_detector(left_speed, right_speed, left_wall, front_wall, left_corner);
    //}
    
    int found = 0;
    if ((position[0] == 1.111802) && (position[2] == 0.044266)) {
      if (found != 0) {
        printf("oops\n");
        left_speed = 0;
        right_speed = 0;
      } else {
        found++;
        if (front_wall == true) {
          left_speed = MAX_SPEED;
          right_speed = -MAX_SPEED;
          printf("Frontwall");
        } else {
          if (left_wall == true) {
            left_speed = MAX_SPEED;
            right_speed = MAX_SPEED;
            printf("Leftwall");
            
          } else if (left_corner == true) {
            left_speed = MAX_SPEED;
            right_speed = MAX_SPEED/5;
            printf("corner");
            
          } else {
            left_speed = MAX_SPEED/5;
            right_speed = MAX_SPEED;
            printf("else");
          }
        }
      }
    } else {
      if (front_wall == true) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
        printf("Frontwall");
      } else {
        if (left_wall == true) {
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED;
          printf("Leftwall");
          
        } else if (left_corner == true) {
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED/5;
          printf("corner");
          
        } else {
          left_speed = MAX_SPEED/5;
          right_speed = MAX_SPEED;
          printf("else");
        }
      }
    }
    
    //wall_detector(left_speed, right_speed, left_wall, front_wall, left_corner);
    
    wb_motor_set_velocity(left_motor,left_speed);
    wb_motor_set_velocity(right_motor,right_speed);
  };

  wb_robot_cleanup();

  return 0;
}