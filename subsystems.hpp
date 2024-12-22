#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
 inline pros::Motor intake(2,pros::MotorGear::blue);
 inline pros::Motor ladyBrown(12,pros::MotorGear::red);

//Sensors
 inline pros::Rotation ladyBrownRotation(15);
 inline pros::Optical colorSorter(13);

//Pneumatics
 inline pros::adi::DigitalOut mogoMech ('a', false);
 inline pros::adi::DigitalOut doinker ('B', false);

 inline int allianceColor =0;
 inline bool enableMogoMech = false;
 inline bool autonStarted = false;