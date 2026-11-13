#include "main.h"

  pros::MotorGroup leftmotor_group({14, 11, 18});
  pros::MotorGroup rightmotor_group({-4, -8, -10});
  pros::MotorGroup intake_motorgroup({2,12});

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

class Drivetrain {
    pros::MotorGroup left{14, 11, 18}; // Left drivetrain track ports
    pros::MotorGroup right{-4, -8, -10}; // Right drivetrain track ports
    pros::Imu imu{1}; // IMU port
    static constexpr double TICKS_PER_INCH = 40.1;  // CALIBRATE!!!

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

    // ── Reset sensors at start of auton ──
    void tare() {
        left.tare_position();
        right.tare_position();
        imu.tare_rotation();
    }
};

Drivetrain drive;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
    drive.tare();
    intake_motorgroup.move(100);
    drive.drive(12);
    drive.turn(90);
    intake_motorgroup.move(0);
    drive.turn(90);
    drive.drive(24);
    drive.turn(90);
    drive.drive(42.5);
    intake_motorgroup.move(0);
    drive.turn(180);
    drive.drive(36);
    intake_motorgroup.move(100);
    drive.turn(180);
    drive.drive(36);
    intake_motorgroup.move(0);
    drive.turn(180);
    drive.drive(36);

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


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

	}
}