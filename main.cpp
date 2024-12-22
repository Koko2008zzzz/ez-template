#include "main.h"
#include "autons.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-9, -8, -10},     // Left Chassis Ports (negative port will reverse it!)
    {18, 19, 20},  // Right Chassis Ports (negative port will reverse it!)

    4,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void set_lift(int input) {
  ladyBrown.move(input);
}
ez::PID liftPID{0.45, 0, 0, 0, "Lift"};

//colorSorter ///////////////////////////////////
//end of color sort//////////////////////////////
//testColorSort/////////////////////////////////
void driverColorSort() {
  while (1) {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (allianceColor ==0) {
        if (colorSorter.get_hue() > 329 || colorSorter.get_hue() < 25) {
          pros::delay(140);
          intake.move(-127);
          pros::delay(15);
          intake.brake();
          pros::delay(117);
          intake.move(127);
         /* pros::delay(5);
          intake.brake(); */
        } else if (colorSorter.get_hue() < 331 && colorSorter.get_hue() > 31) {
          intake.move(127);
        }
      } else if (allianceColor ==1) {
        if (colorSorter.get_hue() > 149 && colorSorter.get_hue() < 269) {
          pros::delay(140);
          intake.move(-127);
          pros::delay(15);
          intake.brake();
          pros::delay(117);
          intake.move(17);
        } else if (colorSorter.get_hue() < 151 || colorSorter.get_hue() > 271) {
          intake.move(127);
        }
      }
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
    } else {
      intake.brake();
    }
    pros::delay(11);
  }
}

void center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if(pressed) {
		allianceColor++;
	} else if (allianceColor > 1) {
		allianceColor = 0;
	}
}

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  colorSorter.set_led_pwm(100);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  ladyBrown.tare_position();

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("skills\n\nskills auton", rightSide4Ring),
      Auton("soloAWP\n\n2 stakes 3/3awp", soloAWP), 
      Auton("twoRedRight\n\ntwo stakes, 2/3AWP", twoRingRight), // stakes, 3/3AWP
      Auton("Red side 4 ring\n\none stake, four rings", leftSide4Ring),
      Auton("Blue side 4 ring\n\none stake 4 rings", rightSide4Ring),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
  ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::lcd::register_btn1_cb(center_button);
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
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
  /*chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  */
  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
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
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_HOLD;

  chassis.drive_brake_set(driver_preference_brake);
  
  //tasks create///////////
  //pros::Task colorSort(driverColorSort, TASK_PRIORITY_DEFAULT+2, TASK_STACK_DEPTH_DEFAULT, "driveColorSort");

 bool enableDoinker = false;
 int ladyBrownToggle =2; 
 //std::uint32_t now = pros::millis();
  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
     if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    } 

   chassis.opcontrol_arcade_standard(ez::SPLIT);

    //sets up controller for intake, r1 is intaking, r2 is outtaking
   if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		 intake.move(127);
	 } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
     intake.move(-127);
	 } else {
		 intake.brake();
	 } 

   // sets up three toggles for the Lady Brown mech, first is intaking position then to scoring position, lastly to rest
    if (master.get_digital_new_press(DIGITAL_L1)) {
      ladyBrownToggle++;
    } else if (master.get_digital(DIGITAL_L2)) {
      ladyBrownToggle =2;
    } else if (ladyBrownToggle >3) {
      ladyBrownToggle =0;
    } else if (master.get_digital(DIGITAL_DOWN)) {
      ladyBrownToggle = 1;
    } 

   if(ladyBrownToggle ==0) {
      liftPID.target_set(180);
   } else if (ladyBrownToggle ==1) {
      liftPID.target_set(732);
   } else if (ladyBrownToggle ==2) {
      liftPID.target_set(0);
   } else if (ladyBrownToggle ==3) {
      ladyBrown.move_relative(-50,100);
   }
   set_lift(liftPID.compute(ladyBrown.get_position())); 
   
     // sets up controllers for Mogo Mech, uses one toggle for enable and disable
	 if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		enableMogoMech = !enableMogoMech;
	 }
	 if (enableMogoMech) {
		mogoMech.set_value(true);
	 } else {
		mogoMech.set_value(false);
	 }

     // sets up controllers for Doinker, uses one toggle for enable and disable
   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    enableDoinker = !enableDoinker;
   }
   if(enableDoinker) {
    doinker.set_value(true);
   } else {
    doinker.set_value(false);
   }
    //colorSort.delay_until(&now,2);
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}