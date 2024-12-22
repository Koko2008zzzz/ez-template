#include "autons.hpp"
#include "main.h"
#include "pros/rtos.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(11, 0, 100);
  chassis.pid_turn_constants_set(3.9, 0.14, 23, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///

 void wait() {
  chassis.pid_wait();
 }

 void drive(double distance, int speed) {
  chassis.pid_drive_set(distance,speed);
 }

 void turn(double angle, int speed) {
  chassis.pid_turn_set(angle,speed);
 }

 void myTask() {
  while (1) {
    if (allianceColor ==0) {
      if(colorSorter.get_hue() >270 || colorSorter.get_hue() < 30) {
        pros::delay(65);
        intake.brake();
        pros::delay(50);
        intake.move(127);
      } else if (allianceColor ==1) {
        if(colorSorter.get_hue() > 150 && colorSorter.get_hue() < 270) {
          pros::delay(65);
          intake.brake();
          pros::delay(50);
        }
      }
    }
    pros::delay(10);
  }
}

///
// Two Ring Right, scores two top rings, 6 points, 2/3AWP
//
void twoRingRight() {
 drive(17,110);
 wait();
 turn(88,110);
 wait();
 drive(-8.9,110);
 wait();
 intake.move(127);
 pros::delay(500);
 intake.brake();
 drive(9.1,110);
 wait();
 turn(0,110);
 wait();
 drive(-12,110);
 wait();
 turn(-62.5,110);
 wait();
 drive(-37,45);
 wait();
 mogoMech.set_value(true);
 enableMogoMech = true;
 intake.move(-127);
 pros::delay(500);
 turn(-90,110);
 pros::delay(500);
 intake.brake();
 wait();
 drive(1,110);
 wait();
 turn(-178,110);
 intake.move(127);
 wait();
 drive(26,110);
 wait();
 pros::delay(1000);
 turn(-3,110);
 wait();
 drive(50,110);
}

///
// New skills auton, 14 points, work in progress
///
void skillsAuton() {
 intake.move(127);
 pros::delay(500);
 intake.brake();
 drive(13.5,110);
 wait();
 turn(90,110);
 wait();
 drive(-31.25,45);
 pros::delay(1000);
 mogoMech.set_value(true);
 pros::delay(100);
 turn(3,110);
 wait();
 intake.move(127);
 drive(30,110); 
 wait();
 //pros::delay(500);
 drive(-6,110);
 wait();
 turn(-87,110);
 //ladyBrown.move_relative(140, 100);
 wait();
 drive(36,110);
 wait();
 drive(-15,110);
 wait();
 turn(0,110);
 wait();
 drive(22,110);
 wait();
 //intake.brake();
 turn(-87,110);
 //ladyBrown.move_relative(243, 50);
 wait();
 intake.move(127);
 drive(17,110);
 wait();
 //ladyBrown.move_relative(360, 100);
 //pros::delay(500);
 drive(-14.5,110);
 wait();
 //ladyBrown.move_relative(-750, 100);
 //intake.brake();
 turn(-175,110);
 //pros::delay(100);
 intake.move(127);
 wait();
 drive(55,80);
 wait();
 pros::delay(100);
 drive(10,110);
 wait();
 pros::delay(100);
 drive(-10,110);
 wait();
 turn(-90,110);
 wait();
 drive(15,110);
 wait();
 turn(45,110);
 wait();
 drive(-5,110);
 wait();
 mogoMech.set_value(false);
 pros::delay(100);
 drive(9,110);
 wait();
 turn(-89,110);
 wait();
 drive(-50,110);
 wait();
 drive(-18,85);
 wait();
 mogoMech.set_value(true);
 pros::delay(100);
 intake.move(127);
 drive(5,110);
 wait();
 turn(89,65);
 wait();
 drive(30,110);
 wait();
 drive(-5,110);
 wait();
 drive(20,110);
 wait();
 drive(-5,110);
 wait();
 intake.brake();
 turn(-45,110);
 wait();
 drive(-8,110);
 wait();
 mogoMech.set_value(false);
 drive(20,110);
 wait();
 }

///
// Solo Auton Win Point, 7 points
///
void soloAWP() {
 //pros::Task  colorSort(myTask,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"colorSort");
 drive(-44,85);
 pros::delay(1260);
 mogoMech.set_value(true); 
 pros::delay(250);
 intake.move(127);
 drive(6.2,110);
 wait();
 pros::delay(200);
 turn(-88,110);
 wait();
 intake.move(120); //127 for bad motor, 80 for good motor
 drive(31,110);
 pros::delay(865);
 intake.brake();
 drive(-13,110); //-10
 wait();
 intake.brake();
 turn(69.5,80); // 70.5
 pros::delay(800);
 intake.move(127); 
 drive(35.5,110); //40
 wait();
 intake.brake(); // this line was added
 doinker.set_value(true);
 pros::delay(50); //50
 drive(-15,110);
 wait();
 doinker.set_value(false);
 pros::delay(100); //100
 turn(45,110);
 wait();
 intake.move(127);
 mogoMech.set_value(false);
 enableMogoMech = false;
 drive(32,110); //32
 pros::delay(860); //875
 intake.brake();
 turn(91,110);
 wait();
 drive(1.3,110);
 wait();
 turn(178.5,110); //178
 wait();
 drive(-7.1,110); //-3.5
 wait();
 intake.move(127);
 pros::delay(510);
 intake.brake();
 drive(35,110);
 intake.move(127);
 //colorSort.remove();
 wait();
}

void leftSide4Ring () {
 drive(-44,80);
 pros::delay(1260);
 mogoMech.set_value(true); 
 pros::delay(250);
 intake.move(127);
 drive(6.2,110);
 wait();
 turn(90,100);
 pros::delay(500);
 drive(28,110); // og 8
 wait();
 drive(-8,110);
 wait();
 turn(178,110);
 pros::delay(500);
 drive(15.2,110);
 wait();
 drive(-8,110);
 wait();
 turn(90,110);
 wait();
 drive(8.25,110);
 wait();
 turn(176,110);
 pros::delay(350);
 drive(11.9,110);
 wait();
 pros::delay(250);
 drive(-10,110);
 wait();
 enableMogoMech = true;
 //ladyBrown.move_relative(750,127);
 turn(270,110);
 wait();
 drive(25,110); 
}


void rightSide4Ring() {
 drive(-45,80);
 pros::delay(1260);
 mogoMech.set_value(true); 
 pros::delay(250);
 intake.move(127);
 drive(7.2,110);
 wait();
 turn(-90,100);
 pros::delay(500);
 drive(28.5,110); // og 8
 wait();
 drive(-8.5,110);
 wait();
 turn(-178,110);
 pros::delay(500);
 drive(15.28,110);
 wait();
 drive(-8,110);
 wait();
 turn(-90,110);
 wait();
 drive(8.3,110);
 wait();
 turn(-176,110);
 pros::delay(350);
 drive(11,110);
 wait();
 pros::delay(300);
 drive(-10,65);
 wait();
 //ladyBrown.move_relative(750,127);
 turn(-270,110);
 wait();
 drive(25,110);
}

///
// Og skils program, 19 points, does not work
///
void skillsAuton1() {
 intake.move(127);
 pros::delay(500);
 intake.brake();
 drive(12.5,110);
 wait();
 turn(89.5,110);
 wait();
 drive(-31.5,45);
 pros::delay(1000);
 mogoMech.set_value(true);
 pros::delay(100);
 turn(6,110);
 wait();
 intake.move(127);
 drive(30,110); 
 wait();
 pros::delay(500);
 drive(-5,110);
 wait();
 turn(0,110);
 wait();
 drive(-33,110);
 wait();
 turn(-90,110);
 wait();
 drive(25,110);
 wait();
 pros::delay(500);
 turn(55,110);
 wait();
 drive(-8,110);
 intake.brake();
 wait();
 mogoMech.set_value(false);
 drive(19,110);
 wait();
 turn(-88,110);
 wait();
 drive(-76,45);
 pros::delay(2675);
 mogoMech.set_value(true);
 pros::delay(100);
 turn(-10,110);
 wait();
 intake.move(127);
 drive(30,110);
 pros::delay(250);
 wait();
 turn(135,110);
 wait();
 doinker.set_value(true);
 drive(48,110);
 wait();
 turn(310,110);
 wait();
 drive(-5,110);
 wait();
 mogoMech.set_value(false);
 doinker.set_value(false);
 pros::delay(500);
 drive(10,110);
}


void testColorSort() {
  pros::Task  colorSort(myTask,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"colorSort");
  intake.move(127);
  pros::delay(13000);
  colorSort.remove();
}
///
// Swing Example
///

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .