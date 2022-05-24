
#include <stdio.h>

#include <webots/robot.h>

#include "movement_controller.h"


int TIME_STEP;
static void init_robot() {
    /* necessary to initialize webots stuff */
    wb_robot_init();

    /* get simulator time step */
	TIME_STEP = 64;

    /* init the controller */
    robot_controller_init(TIME_STEP);
}

/* main function */
int main(int argc, char** argv) {
	init_robot();

	/* main loop */
	while (wb_robot_step(TIME_STEP) != -1) {
		get_distance_values();
		print_sensor_values();

		bool* is_sensors_active = get_sensors_condition();
		if (is_sensors_active[0] || is_sensors_active[1]) {
			motor_rotate_left_in_degrees(90);

		}
		else {
			motor_move_forward();
		}
	};

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();

	return 0;
}
