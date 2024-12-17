#include "main.h"
#include "lemlib/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"


#pragma once

extern pros::Motor intake;
extern pros::MotorGroup lift_motors;
extern pros::adi::DigitalOut Claw;
extern pros::adi::DigitalOut Blooper;

extern pros::adi::Potentiometer auto_select;
extern pros::Optical optical_sensor;
extern pros::Rotation wall_arm;


extern pros::MotorGroup Left_motors;
extern pros::MotorGroup Right_motors;

extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;
