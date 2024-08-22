#include "main.h"
#include "lemlib/api.hpp"
#include "api.h"


#pragma once

extern pros::Motor intake;

extern pros::ADIDigitalOut Claw;
extern pros::ADIPotentiometer drive_select;
extern pros::ADIPotentiometer auto_select;


extern pros::MotorGroup Left_motors;
extern pros::MotorGroup Right_motors;

extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;



