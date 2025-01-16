#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "Region-config.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <iostream> 
#include <string>

//global variables

bool isRed;
int reversed = 1;
const int arm_positions[] = {356, 328,  168}; // Adjust these values based on your arm's setup
const int num_positions = sizeof(arm_positions) / sizeof(arm_positions[0]);
int current_position_index = 0;
int target_position = 0;
bool move_arm_task_running = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
	
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
	pros::Task color_sort([&]() {
		optical_sensor.set_led_pwm(100);  // Adjust the sensor's LED power for better detection
		if (optical_sensor.get_rgb().blue > optical_sensor.get_rgb().red) {
			isRed = false;
		}
		else{
			isRed = true;
		}
		printf("Red Alliance: %d",isRed);
		if (isRed == true) {
			while (true){
				if (optical_sensor.get_hue() > 180) {
					pros::delay(300);
					reversed = 0;
					pros::delay(200);
					reversed = 1;
				}
			pros::delay(20);
			}
		}
		else{
			while (true) {
				if (optical_sensor.get_hue() < 30) {
					pros::delay(300);
					reversed = 0;
					pros::delay(200);
					reversed = 1;
					
				}
			pros::delay(20);
			}
		}
		pros::delay(20);
	});
	pros::Task armTask([&]() {
		const int tolerance = 1; // Degrees of error tolerance
    	const int max_speed = 127;
    	const double kP = 5; // Proportional constant
	
    while (true) {
        if (move_arm_task_running) {
            int current_position = wall_arm.get_position()/100;
            int error = target_position - current_position;
            // Stop motor when within tolerance
            if (std::abs(error) <= tolerance) {
				if (current_position_index == 2 && error < 1){
					current_position_index = (current_position_index + 2) % num_positions;
					target_position = arm_positions[current_position_index];
				}
				else{
					lift_motors.move_velocity(0);
                	move_arm_task_running = false; // Stop the task
				}
                
            } else {
                // Calculate motor speed
                int motor_speed = static_cast<int>(error * kP);
                motor_speed = std::clamp(motor_speed, -max_speed, max_speed);
                lift_motors.move_velocity(motor_speed);
            }
			
        }
        pros::delay(20); // Task delay
    }
	});
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	int auto_v; 
	auto_v = floor(auto_select.get_value() /1024);
	pros::lcd::print(3, "Pot auto:%d", auto_v);
	switch (auto_v){
		case 0: //blue left 3
			chassis.moveToPose(-3,-23, 32, 3000,{.forwards = false}, false);
			Claw.set_value(true);
			pros::delay(500);
			intake.move(-127);
			pros::delay(100);
			chassis.turnToHeading(99, 1000);
			chassis.moveToPoint(12.6, -24.5, 3000,{},false);
			pros::delay(300);
			chassis.turnToHeading(20, 1000,{},false);
			Blooper.set_value(true);
			chassis.moveToPose(28, 5, 20, 2500,{},false);
			chassis.turnToHeading(170, 1000,{},false);
			Blooper.set_value(false);
			intake.move(100);
			current_position_index = (current_position_index + 3) % num_positions;
            target_position = arm_positions[current_position_index];
            move_arm_task_running = true;
			chassis.moveToPose(-2, -29, 225, 3000);

			break;
		case 1: // blue right 2
			chassis.moveToPose(-3, -23, 32, 3000,{.forwards = false},false);
			Claw.set_value(true);
			pros::delay(500);
			intake.move(-127);
			pros::delay(200);
			chassis.turnToHeading(255, 1000);
			chassis.moveToPose(-26, -27.8, 255, 2500,{},false);
			pros::delay(300);
			chassis.turnToHeading(-155, 1000,{},false);
			chassis.moveToPose(-28, -43.25, -170, 3000,{},false);
			pros::delay(1000);
			chassis.turnToHeading(-130, 1000,{},false);
			chassis.moveToPose(-33, -43.25,-135, 1000,{},false);

	// 		break;
		// case 2: // red right 1
			chassis.moveToPose(3,-23, -32, 3000,{.forwards = false}, false);
			Claw.set_value(true);
			pros::delay(500);
			intake.move(-127);
			pros::delay(100);
			chassis.turnToHeading(-82, 1000);
			chassis.moveToPose(-12.6, -24.5,-99, 3000,{},false);
			pros::delay(300);
			chassis.turnToHeading(-20, 1000,{},false);
			Blooper.set_value(true);
			
			chassis.moveToPose(-28, 5, -20, 2500,{},false);
			intake.move(0);
			chassis.turnToHeading(-170, 1000,{.direction = AngularDirection::CW_CLOCKWISE},false);
			Blooper.set_value(false);
		
			break;
		case 3: //red left 0
		//move to first goal
			chassis.moveToPose(3, -23, -32, 3000,{.forwards = false},false);
			Claw.set_value(true);
			pros::delay(500);
			intake.move(-127);
			pros::delay(200);
			//turn to first ring
			chassis.turnToHeading(-255, 1000);
			//go to first ring
			chassis.moveToPose(26, -27.8, -255, 2500,{},false);
			pros::delay(300);
			//turn to mid
			chassis.turnToHeading(155, 1000,{},false);
			//move to mid
			chassis.moveToPose(28, -43.25, 170, 3000,{},false);
			pros::delay(1000);
			//backup
			chassis.moveToPoint(33, -30, 1000,{.forwards = false},false);
			//turn to ladder
			chassis.turnToHeading(275.75, 1000);
			current_position_index = (current_position_index + 3) % num_positions;
            target_position = arm_positions[current_position_index];
            move_arm_task_running = true;
			chassis.moveToPose(9.8, -40,300, 1000);
			break;
	}

			// 257.75
			// 9.8, -40
			// 		
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bool blooperDeploy = false;
	

	while (true) {
		int auto_v; 
		auto_v = floor(auto_select.get_value() /1365);

		
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
	
		pros::lcd::print(7, "auto: %d", auto_v);
	

		// get left y and right x positions
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// move the robot
		// chassis.curvature(leftY, rightX);
		chassis.arcade(leftY, rightX,false,.3);


		// intake control
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move(-127 * reversed);
		}
		
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(127 * reversed);
		}
		
		else {
			intake.move(0);
		}

		//claw control
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			Claw.set_value(true);
		}
		
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			Claw.set_value(false);
		}
		//arm control
		 if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            // Cycle to the next position
             
                // Cycle to the next position and set target
                current_position_index = (current_position_index + 1) % num_positions;
                target_position = arm_positions[current_position_index];
                move_arm_task_running = true;
        }
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			if(blooperDeploy){
				Blooper.set_value(false);
				blooperDeploy = false;
			}
			else{
				Blooper.set_value(true);
				blooperDeploy = true;
			}
		}
		
		//delay to save resources 
		pros::delay(20);                          
	}
}
