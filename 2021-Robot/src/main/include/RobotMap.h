//RobotMap: A collection of constant variables for declarations and settings.
//Significantly reduces the amount of time spent looking for and configuring.

#pragma once

#include <math.h>
#include <string>

#include "VectorDouble.h"

/*_____RoboRIO PWM Pin Declarations_____*/
const int R_PWMPortClimberMotorClimb     = 0;
const int R_PWMPortClimberServoLock      = 1;
const int R_PWMPortClimberMotorWheel     = 2;
const int R_PWMPortClimberMotorTranslate = 3;
const int R_PWMPortRightServo            = 4;
const int R_PWMPortLeftServo             = 5;
/*___End RoboRIO PWM Pin Declarations___*/

/*_____RoboRIO DIO Pin Declarations_____*/
const int R_DIOPortSwitchClimberBottom = 0;
const int R_DIOPortSwitchSwerveUnlock  = 1;
/*___End RoboRIO DIO Pin Declarations___*/

/*_____RoboRIO CAN Bus ID Declarations_____*/
const int R_CANIDZionFrontRightSwerve = 1;
const int R_CANIDZionFrontRightDrive  = 2;
const int R_CANIDZionFrontLeftSwerve  = 3;
const int R_CANIDZionFrontLeftDrive   = 4;
const int R_CANIDZionRearLeftDrive    = 5;
const int R_CANIDZionRearLeftSwerve   = 6;
const int R_CANIDZionRearRightDrive   = 7;
const int R_CANIDZionRearRightSwerve  = 8;

const int R_CANIDMotorIntake = 9;

const int R_CANIDMotorLauncherIndex  = 10;
const int R_CANIDMotorLauncherLaunchOne = 11;
const int R_CANIDMotorLauncherLaunchTwo = 12;

const int R_CANIDMotorClimberForward = 14;
const int R_CANIDMotorClimberRear = 13;
/*___End RoboRIO CAN Bus ID Declarations___*/

/*_____Controller Settings_____*/
const int R_controllerPortPlayerOne = 0;
const int R_controllerPortPlayerTwo = 1;
const int R_controllerPortPlayerThree = 2;

//This deadzone is used to determine when the controller is completely motionless
const double R_deadzoneController = .15;
//And this one is to determine when rotation is being induced, as simply operation
//of the controller often results in errant rotation. Due to how easy it is to
//drift, it is significantly higher.
const double R_deadzoneControllerZ = 0.15;
/*___End Controller Settings___*/

/*_____Global Robot Variable Settigns_____*/
//This is the highest decimal percentage of full speed that Zion can actually go.
const double R_executionCapZion = 1.00;
const double R_executionCapZionZ = .65;
//This is at what rate the regular execution cap is scaled for precision driving.
const double R_executionCapZionPrecision = .15 * R_executionCapZion;
//This one is for running the intake motors.
const double R_executionCapIntake = .75;

//This is the default launcher index speed.
const double R_launcherDefaultSpeedIndex = .65;
//And this the default launcher launch speed, for both distances and idle.
const double R_launcherDefaultSpeed = .76378;

//This is the speed for automatic lateral movement in autonomous.
const double R_zionAutoMovementSpeedLateral = .35;
//And for rotational movement.
const double R_zionAutoMovementSpeedRotational = .2;
//This is the tolerance for autonomous angle assumption in degrees.
const double R_zionAutoToleranceAngle = 10;
//This is how close to zero the Limelight's horizontal target offset can be
//in order to be considered centered.
const double R_zionAutoToleranceHorizontalOffset = .2;
//This is the number of digits past the decimal place that will be stored when
//being recorded.
const int R_zionAutoControllerRecorderPrecision = 5;
const int R_zionAutoControllerTotalDigits = R_zionAutoControllerRecorderPrecision + 2;
//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
//Often, a REV Rotation is referred to as a Nic, although they mean different things.
//Truly, a Nic is ~17.976 REV Rotation values.
const double R_nicsConstant = 17.9761447906494;
//The change in encoder output per full wheel rotation around the axle.
//This value can be used to move a certain distance using solely encoder values.
const double R_kuhnsConstant = 8.312033026;
//If an xy coordinate plane is centered at the middle of the drivetrain, this
//is the radian measure between the y-axis and the front right wheel. This is
//the basic unit of a non-moving center turn, and it is modified as the basis
//for moving and turning at the same time.
const double R_angleFromCenterToFrontLeftWheel = 45.;
const double R_angleFromCenterToFrontRightWheel = 315.;
const double R_angleFromCenterToRearLeftWheel = 135.;
const double R_angleFromCenterToRearRightWheel = 225.;

//These contants are used for the functions which provide assuming a position.
//See those functions for further detail.
const double R_swerveTrainAssumePositionTolerance = .25;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;

const double R_swerveTrainLimelightLockTolerance = 1;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorAt = 5.0;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorSpeed = .05;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorAt = 2.5;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorSpeed = .025;

const double R_swerveTrainHoldAngleTolerance = 1.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorAt = 10.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorSpeed = .075;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorAt = 5.0;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorSpeed = .075;

const double R_circumfrenceWheel = 4 * M_PI;
/*___End Global Robot Variable Settings___*/