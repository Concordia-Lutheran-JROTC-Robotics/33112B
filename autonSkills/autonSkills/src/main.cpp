#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cstdio>
#include <string>

#define DIGITAL_PISTON_PORT 'A'

static bool autonLeftSide = true;
static bool autonRightSide = false;

pros::MotorGroup left_mg({17, 14, 18});
pros::MotorGroup right_mg({-4, -8, -10});
pros::MotorGroup lift_mg({16, -6});
pros::MotorGroup intake_mg({-2, 12});

class Drivetrain {
    pros::MotorGroup left{1, 7, 10};
    pros::MotorGroup right{-11, -17, -20};
    pros::Imu imu{21};

    static constexpr double TICKS_PER_INCH = 40.1;  // ← CALIBRATE THIS

public:
    // ── Drive a distance in inches (negative = backward) ──
    void drive(double inches, int speed = 60) {
        int ticks = -inches * TICKS_PER_INCH;
        left.tare_position();
        right.tare_position();
        left.move_relative(ticks, speed);
        right.move_relative(ticks, speed);
        // Wait until both sides are near target
        int timeout = 0;
        while ((std::abs(left.get_position() - ticks) > 10 ||
                std::abs(right.get_position() - ticks) > 10) && timeout < 1500) { // 15 seconds max
            pros::delay(10);
            timeout++;
        }
        stop();
    }

    // ── Turn by degrees (positive = right, negative = left) ──
    void turn(double degrees, int speed = 150) {
        double start = imu.get_rotation();  // unbounded, doesn't wrap
        double target = start + degrees;

        left.move_velocity(-speed);
        right.move_velocity(speed);

        int timeout = 0;
        while (std::abs(imu.get_rotation() - target) > 1.0 && timeout < 1000) {
            pros::delay(10);
            timeout++;
        }

        stop();
    }

    // ── Stop ──
    void stop() {
        left.move_velocity(0);
        right.move_velocity(0);
    }

    // ── Calibrate the IMU (blocking, ~2 seconds) ──
    // Must be called while the robot is stationary before using imu.get_rotation().
    void calibrate() {
        imu.reset(true);
    }

    // ── Reset sensors at start of auton ──
    void tare() {
        left.tare_position();
        right.tare_position();
        imu.tare_rotation();
    }
};

Drivetrain drive;

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
    drive.calibrate();   // Blocking IMU calibration (~2 s) — required before turn() uses imu.get_rotation()
    drive.tare();
    drive.drive(12, 100);
    drive.turn(90, 100);
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
bool turnRightClaw = true;
double liftInitialPosition = lift_mg.get_position();

pros::Controller master(pros::E_CONTROLLER_MASTER);

void digital_l1_function() {
	while (master.get_digital(DIGITAL_L1)) {
		lift_mg.move(75);
		pros::delay(20);
	}
	lift_mg.move(0);
}

void digital_l2_function() {
	while (master.get_digital(DIGITAL_L2)) {
		if (lift_mg.get_position() <= liftInitialPosition) {
			lift_mg.move(0);
			break;
		}
		lift_mg.move(-75);
		pros::delay(20);
	}
	lift_mg.move(0);
}

void digital_down_function() {
	while (master.get_digital(DIGITAL_DOWN)) {
		intake_mg.move(-25);
		pros::delay(20);
	}
	intake_mg.move(0);
}

void digital_right_function() {
	while (master.get_digital(DIGITAL_RIGHT)) {
		intake_mg.move(100);
		pros::delay(20);
	}
	intake_mg.move(0);
}

void opcontrol() {
	
	pros::lcd::set_text(1, "Driver Control");
	
	// Record initial position so lift never goes back further than start
	lift_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage

		if (master.get_digital_new_press(DIGITAL_L1)) {
			// while (master.get_digital(DIGITAL_L1)) {
			// 	lift_mg.move(75);
			// 	pros::delay(20);
			// }
			// lift_mg.move(0);
			pros::Task digital_l1_task(digital_l1_function);
		}

		if (master.get_digital_new_press(DIGITAL_L2)) {
			// while (master.get_digital(DIGITAL_L2)) {
			// 	if (lift_mg.get_position() <= liftInitialPosition) {
			// 		lift_mg.move(0);
			// 		break;
			// 	}
			// 	lift_mg.move(-75);
			// 	pros::delay(20);
			// }
			// lift_mg.move(0);
			pros::Task digital_l2_task(digital_l2_function);
		}

		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			// while (master.get_digital(DIGITAL_DOWN)) {
			// 	clawUD.move(-50);
			// }
			// clawUD.move(0);
			pros::Task digital_down_task(digital_down_function);
		}

		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			// while (master.get_digital(DIGITAL_RIGHT)) {
			// 	clawUD.move(50);
			// }
			// clawUD.move(0);
			pros::Task digital_right_task(digital_right_function);
		}

		pros::delay(20);
	}
	
	pros::delay(20);                               // Run for 20 ms then update
}