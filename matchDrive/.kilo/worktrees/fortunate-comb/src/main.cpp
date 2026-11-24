#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/screen.hpp"
#define DIGITAL_SENSOR_PORT 'A'

static bool autonLeftSide = true;
static bool autonRightSide = false;

void on_left_button() {
	pros::lcd::set_text(2, "Autonomous: Left Side");
	autonLeftSide = true;
	autonRightSide = false;
}

void on_right_button() {
	pros::lcd::set_text(2, "Autonomous: Right Side");
	autonRightSide = true;
	autonLeftSide = false;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Initialization Process");
	pros::lcd::set_text(2, "Autonomous: Left Side");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::set_text(1, "Disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::lcd::set_text(1, "Competition Initialization");
}

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
	if (autonLeftSide) {
		pros::lcd::set_text(1, "Left Side Autonomous");
		// Left side code here
	} else if (autonRightSide) {
		pros::lcd::set_text(1, "Right Side Autonomous");
		// Right side code here
	} else if (!autonLeftSide && !autonRightSide) {
		pros::lcd::set_text(1, "Autonomous Disabled");
	} else {
		pros::lcd::set_text(1, "Variables Mismatched");
	}
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

bool clawPistonValue = false;
pros::adi::DigitalOut clawPiston(DIGITAL_SENSOR_PORT);

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::lcd::set_text(1, "Driver Control");
	pros::MotorGroup left_mg({1, 7, 10});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-11, -17, -20});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

	static bool pistonPressValue = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = -master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = -master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		
		if (master.get_digital(DIGITAL_A) && !pistonPressValue) {
			clawPiston.set_value(clawPistonValue);
			clawPistonValue = !clawPistonValue;
		}
		pistonPressValue = master.get_digital(DIGITAL_A);

		pros::delay(20);
	}
	
	pros::delay(20);                               // Run for 20 ms then update
}
