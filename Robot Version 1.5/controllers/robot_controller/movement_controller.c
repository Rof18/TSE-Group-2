#include "movement_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

/* Motor device */
static WbDeviceTag front_right_motor, back_right_motor, front_left_motor, back_left_motor;

/* robots speed in rad/s */
#define MAX_SPEED 6

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 2
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char* distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = { "ds_0", "ds_1" };


#define SENSOR_VALUE_DETECTION_THRESHOLD 900



static int TIME_STEP;

/* function to init robot controller stuff */
void robot_controller_init(int time_step)
{
	TIME_STEP = time_step;
	/* get a handler to the motors and set target position to infinity (speed control) */
	front_right_motor = wb_robot_get_device("fr_motor_1");
	back_right_motor = wb_robot_get_device("br_motor_3");

	front_left_motor = wb_robot_get_device("fl_motor_2");
	back_left_motor = wb_robot_get_device("bl_motor_4");

	wb_motor_set_position(front_right_motor, INFINITY);
	wb_motor_set_position(back_right_motor, INFINITY);

	wb_motor_set_position(front_left_motor, INFINITY);
	wb_motor_set_position(back_left_motor, INFINITY);

	wb_motor_set_velocity(front_right_motor, 0.0);
	wb_motor_set_velocity(back_right_motor, 0.0);

	wb_motor_set_velocity(front_left_motor, 0.0);
	wb_motor_set_velocity(back_left_motor, 0.0);

	/* get a handler to the sensors */
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
		distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
		wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
	}
}

/* function to stop the motor (set motor velocity to zero) */
void motor_stop() {
	wb_motor_set_velocity(front_right_motor, 0.0);
	wb_motor_set_velocity(back_right_motor, 0.0);

	wb_motor_set_velocity(front_left_motor, 0.0);
	wb_motor_set_velocity(back_left_motor, 0.0);
}

/* function to set motor velocity to move forward */
void motor_move_forward() {
	wb_motor_set_velocity(front_right_motor, MAX_SPEED);
	wb_motor_set_velocity(back_right_motor, MAX_SPEED);

	wb_motor_set_velocity(front_left_motor, MAX_SPEED);
	wb_motor_set_velocity(back_left_motor, MAX_SPEED);
}

/* function to set motor velocity to rotate right in place*/
void motor_rotate_right() {
	wb_motor_set_velocity(front_right_motor, -MAX_SPEED);
	wb_motor_set_velocity(back_right_motor, -MAX_SPEED);

	wb_motor_set_velocity(front_left_motor, MAX_SPEED);
	wb_motor_set_velocity(back_left_motor, MAX_SPEED);
}

/* function to set motor velocity to rotate left in place*/
void motor_rotate_left() {
	wb_motor_set_velocity(front_right_motor, MAX_SPEED);
	wb_motor_set_velocity(back_right_motor, MAX_SPEED);

	wb_motor_set_velocity(front_left_motor, -MAX_SPEED);
	wb_motor_set_velocity(back_left_motor, -MAX_SPEED);
}


/* function to get sensors condition
 * if sensor detect obstacle, then the condition is true
 * */
bool* get_sensors_condition()
{
	static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = { false };

	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) < SENSOR_VALUE_DETECTION_THRESHOLD) {
			sensors_condition[i] = true;
		}
		else {
			sensors_condition[i] = false;
		}
	}

	return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());

	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}

	printf("\n");

	printf("%s speed value ", wb_robot_get_name());
	printf("%d:%.3f ", front_right_motor,wb_motor_get_velocity(front_right_motor));
	printf("%d:%.3f ", front_left_motor, wb_motor_get_velocity(front_left_motor));
	printf("\n");
}