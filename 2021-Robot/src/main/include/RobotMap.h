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
/*___End RoboRIO CAN Bus ID Declarations___*/

/*_____Controller Settings_____*/
const int R_controllerPortPlayerOne = 0;
const int R_controllerPortPlayerTwo = 1;

//This deadzone is used to determine when the controller is completely motionless
const double R_deadzoneController = .1;
//And this one is to determine when rotation is being induced, as simply operation
//of the controller often results in errant rotation. Due to how easy it is to
//drift, it is significantly higher.
const double R_deadzoneControllerZ = .1;
// This deadzone is for the maximum allowable Limelight offset.
const double R_deadzoneLimelightX = 0.75;

//And this is the execution cap for how fast manual zeroing can occur.
const double R_executionCapControllerZero = .1;

//These are the playerTwo raw controller buttons that are used for manually
//zeroing Zion one wheel at a time by holding them down.
const int R_zeroButtonFR = 0;
const int R_zeroButtonFL = 0;
const int R_zeroButtonRL = 0;
const int R_zeroButtonRR = 0;
/*___End Controller Settings___*/

/*_____Global Robot Variable Settigns_____*/
//This is the highest decimal percentage of full speed that Zion can actually go.
const double R_executionCapZion = .4;
//This is at what rate the regular execution cap is scaled for precision driving.
const double R_executionCapZionPrecision = .25 * R_executionCapZion;
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
const double R_zionAutoToleranceHorizontalOffset = .5;
//This is the number of digits past the decimal place that will be stored when
//being recorded.
const int R_zionAutoJoystickRecorderPrecision = 5;
const int R_zionAutoJoystickTotalDigits = R_zionAutoJoystickRecorderPrecision + 2;
const std::string R_zionAutoTestPath = "1.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000000.921880.992191.000000.867190.992191.000000.835940.992191.000000.820310.992191.000000.820310.992191.000000.820310.992191.000000.820310.992191.000000.796880.992191.000000.796880.992191.000000.750000.992191.000000.742190.992191.000000.742190.992191.000000.742190.992191.000000.742190.992191.000000.804690.992191.000000.812500.992191.000000.820310.992191.000000.835940.976561.000000.851560.968751.000000.851560.937501.000000.859380.921881.000000.859380.914061.000000.859380.906251.000000.859380.906251.000000.859380.898441.000000.859380.898441.000000.859380.898441.000000.859380.882811.000000.859380.882811.000000.859380.875001.000000.859380.875001.000000.859380.875001.000000.859380.875001.000000.859380.875001.000000.859380.875001.000000.859380.867191.000000.750000.867191.000000.671880.867191.000000.617190.867191.000000.593750.867191.000000.578120.867191.000000.570310.867191.000000.570310.867191.000000.554690.867191.000000.531250.867191.000000.492190.929691.000000.453120.960941.000000.445310.984381.000000.437500.992191.000000.437500.992191.000000.437500.992191.000000.437500.992191.000000.437500.992191.000000.437500.992191.000000.437500.992191.000000.429690.992191.000000.429690.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.421880.992191.000000.414060.992191.000000.414060.992191.000000.406250.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398441.000001.000000.398441.000001.000000.398441.000001.000000.398441.000001.000000.398441.015751.000000.398441.047241.000000.398441.078741.000000.398441.149611.000000.398441.204721.000000.398441.236221.000000.398441.259841.000000.398441.314961.000000.398441.362201.000000.398441.425201.000000.476561.456691.000000.492191.480311.000000.500001.488191.000000.507811.511811.000000.515621.511811.000000.515621.527561.000000.515621.535431.000000.523441.551181.000000.523441.551181.000000.523441.551181.000000.531251.551181.000000.531251.551181.000000.531251.566931.000000.531251.566931.000000.531251.496061.000000.531251.456691.000000.531251.456691.000000.429691.354331.000000.414061.314961.000000.406251.299211.000000.398441.291341.000000.390621.275591.000000.382811.267721.000000.382811.251971.000000.375001.244091.000000.375001.236221.000000.375001.236221.000000.367191.204721.000000.351561.181101.000000.335941.118111.000000.320311.062991.000000.296881.031501.000000.281251.007871.000000.273440.992191.000000.273440.992191.000000.250000.992191.000000.234380.992191.000000.226560.992191.000000.226560.992191.000000.218750.992191.000000.218750.992191.000000.203120.992191.000000.195310.992191.000000.171880.992191.000000.164060.992191.000000.164060.992191.000000.156250.992191.000000.156250.992191.000000.140620.992191.000000.140620.992191.000000.132810.992191.000000.132810.992191.000000.132810.992191.000000.132810.992191.000000.132810.992191.000000.132810.992191.000000.132811.070871.000000.132811.070871.000000.132811.094491.000000.132811.094491.000000.132811.094491.000000.132811.015751.000000.132811.015751.000000.132811.000001.000000.132810.992191.000000.132810.992191.000000.101560.992191.000000.046880.992191.000000.007810.992191.000000.000001.078741.000000.000001.165351.000000.000001.299211.000000.000001.401571.000000.000001.440941.000000.000001.511811.000000.000001.527561.000000.000001.527561.000000.000001.448821.000000.117191.362201.000000.210941.188981.000000.273441.055121.000000.335941.000001.000000.375000.992191.000000.382810.992191.000000.406250.992191.000000.453120.976561.000000.484380.921881.000000.546880.875001.000000.546880.875001.000000.609380.773441.000000.617190.687501.000000.625000.593751.000000.640620.546881.000000.640620.546881.000000.687500.476561.000000.703120.445311.000000.710940.398441.000000.718750.359381.000000.718750.359381.000000.726560.320311.000000.726560.320311.000000.726560.296881.000000.726560.273441.000000.726560.273441.000000.546880.242191.000000.546880.242191.000000.390620.242191.000000.265620.328121.000000.109380.429691.000000.039060.640621.000000.000000.867191.000000.000000.984381.000000.000000.992191.000000.000001.000001.000000.000001.047241.000000.000001.102361.000000.000001.196851.000000.000001.259841.000000.000001.267721.000000.000001.267721.000000.000001.283461.000000.000001.322831.000000.000001.354331.000000.093751.354331.000000.179691.354331.000000.390621.173231.000000.656251.047241.000000.812501.000001.000000.960940.992191.000001.000000.976561.000001.000000.890621.000001.000000.664061.000001.000000.343751.000001.000000.164061.000001.000000.015621.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.039370.000001.000001.086610.000001.000001.086610.000001.000001.228350.000001.000001.283460.000001.000001.307090.062501.000001.322830.125001.000001.362200.203121.000001.370080.242191.000001.370080.250001.000001.370080.250001.000001.370080.250001.000001.370080.250001.000001.370080.250001.000001.377950.250001.000001.377950.250001.000001.377950.312501.000001.377950.476561.000001.377950.593751.000001.377950.718751.000001.377950.757811.000001.299210.757811.000001.283460.609381.000001.244090.492191.000001.212600.351561.000001.125980.156251.000001.070870.039061.000001.007870.007811.000001.007870.007811.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.101561.000001.000000.101561.000001.000000.343751.000001.000000.515621.000001.000000.640621.000001.000000.703121.000001.000000.703121.000001.000000.835941.000001.000000.921881.000001.094490.960941.000001.196850.992191.000001.196850.992191.000001.283460.992191.000001.299210.992191.000001.307090.992191.000001.322830.992191.000001.322830.992191.000001.338580.992191.000001.354331.000001.000001.385831.007871.000001.425201.015751.000001.472441.015751.000001.496061.015751.000001.535431.015751.000001.543311.015751.000001.559061.015751.000001.574801.015751.000001.598431.015751.000001.637800.921881.000001.645670.882811.000001.661420.875001.000001.661420.875001.000001.661420.875001.000001.669290.875001.000001.669290.875001.000001.669290.968751.000001.669290.992191.000001.669290.992191.000001.669290.992191.000001.677170.992191.000001.685040.992191.000001.692910.992191.000001.700790.992191.000001.708660.992191.000001.724410.992191.000001.740160.992191.000001.755910.992191.000001.755910.992191.000001.771650.992191.000001.771650.906251.000001.779530.867191.000001.779530.828121.000001.779530.828121.000001.787400.804691.000001.787400.804691.000001.787400.929691.000001.685040.968751.000001.685040.968751.000001.535431.031501.000001.503941.157481.000001.433071.283461.000001.354331.488191.000001.314961.653541.000001.314961.653541.000001.267721.755911.000001.228351.913391.000001.181101.960631.000001.125981.992131.000001.094492.000001.000001.047242.000001.000001.015752.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.149612.000001.000001.244092.000001.000001.299212.000001.000001.354332.000001.000001.456692.000001.000001.574802.000001.000001.622052.000001.000001.677172.000001.000001.692912.000001.000001.692912.000001.000001.692912.000001.000001.692912.000001.000001.582682.000001.000001.456692.000001.000001.385832.000001.000001.291342.000001.000001.291342.000001.000001.133862.000001.000001.047242.000001.000001.007872.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000000.992192.000001.000000.976561.905511.000000.921881.842521.000000.867191.779531.000000.765621.732281.000000.742191.724411.000000.718751.724411.000000.710941.724411.000000.695311.787401.000000.679691.811021.000000.671881.842521.000000.671881.874021.000000.664061.905511.000000.656251.960631.000000.656251.976381.000000.648442.000001.000000.640622.000001.000000.609382.000001.000000.570311.889761.000000.531251.755911.000000.507811.637801.000000.492191.488191.000000.468751.409451.000000.453121.338581.000000.437501.299211.000000.421881.275591.000000.421881.275591.000000.406251.141731.000000.406251.078741.000000.406251.007871.000000.406250.992191.000000.406250.992191.000000.406250.992191.000000.476560.992191.000000.500000.992191.000000.500000.992191.000000.500000.992191.000000.539060.984381.000000.687500.906251.000000.757810.875001.000000.796880.843751.000000.828120.812501.000000.851560.765621.000000.859380.718751.000000.859380.703121.000000.859380.687501.000000.859380.671881.000000.859380.671881.000000.796880.671881.000000.796880.671881.000000.796880.671881.000000.796880.656251.000000.796880.625001.000000.796880.625001.000000.796880.453121.000000.882810.375001.000000.898440.343751.000000.906250.343751.000000.906250.343751.000000.828120.343751.000000.828120.343751.000000.742190.343751.000000.585940.343751.000000.460940.343751.000000.398440.343751.000000.312500.343751.000000.226560.343751.000000.164060.343751.000000.085940.429691.000000.039060.492191.000000.007810.539061.000000.000000.601561.000000.000000.601561.000000.000000.765621.000000.000000.828121.000000.000000.882811.000000.000000.906251.000000.000000.906251.000000.000000.945311.000000.000000.968751.000000.000000.984381.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000001.000001.000000.000001.000001.000000.000001.110241.000000.000001.228351.000000.000001.314961.000000.000001.330711.000000.000001.346461.000000.000001.346461.000000.000001.346461.000000.000001.346461.000000.000001.354331.000000.000001.354331.000000.000001.472441.000000.000001.559061.000000.000001.692911.000000.000001.826771.000000.000001.826771.000000.093751.960631.000000.109381.984251.000000.109381.984251.000000.109381.984251.000000.109381.984251.000000.117191.984251.000000.125001.866141.000000.156251.748031.000000.179691.637801.000000.234381.590551.000000.273441.527561.000000.328121.448821.000000.367191.393701.000000.382811.322831.000000.398441.267721.000000.398441.149611.000000.398441.047241.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.398440.992191.000000.312500.992191.000000.273440.992191.000000.257811.078741.000000.218751.141731.000000.218751.141731.000000.132811.244091.000000.085941.259841.000000.062501.267721.000000.046881.267721.000000.046881.267721.000000.023441.188981.000000.015621.173231.000000.000001.157481.000000.000001.125981.000000.000001.125981.000000.000001.110241.000000.000001.110241.000000.000001.204721.000000.000001.291341.000000.000001.503941.000000.000001.590551.000000.000001.661421.000000.000001.708661.000000.000001.732281.000000.000001.755911.000000.000001.771651.000000.000001.787401.000000.000001.795281.000000.000001.803151.000000.000001.811021.000000.000001.834651.000000.000001.866141.000000.000001.921261.000000.117191.960631.000000.164062.000001.000000.164062.000001.000000.281252.000001.000000.367192.000001.000000.492192.000001.000000.585942.000001.000000.710942.000001.000000.757812.000001.000000.796882.000001.000000.835942.000001.000000.867192.000001.000000.921882.000001.000000.968752.000001.000000.984382.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.015752.000001.000001.062992.000001.000001.118112.000001.000001.212601.905511.000001.259841.803151.000001.314961.740161.000001.346461.732281.000001.385831.724411.000001.401571.716541.000001.433071.716541.000001.448821.716541.000001.472441.716541.000001.472441.716541.000001.503941.708661.000001.511811.700791.000001.535431.692911.000001.559061.685041.000001.645671.645671.000001.692911.590551.000001.724411.559061.000001.732281.535431.000001.732281.535431.000001.732281.535431.000001.732281.535431.000001.740161.535431.000001.740161.535431.000001.740161.535431.000001.740161.535431.000001.740161.527561.000001.740161.464571.000001.740161.346461.000001.740161.346461.000001.740161.212601.000001.748031.188981.000001.755911.181101.000001.771651.173231.000001.771651.173231.000001.803151.141731.000001.818901.110241.000001.842521.062991.000001.881891.015751.000001.881891.015751.000001.952760.992191.000001.968500.992191.000001.984250.992191.000001.984250.992191.000001.984250.992191.000001.984250.992191.000001.984250.992191.000001.984250.992191.000001.842521.086611.000001.763781.149611.000001.724411.267721.000001.661421.338581.000001.598431.448821.000001.511811.535431.000001.433071.574801.000001.393701.622051.000001.330711.669291.000001.236221.708661.000001.157481.740161.000001.062991.740161.000001.023621.748031.000001.007871.748031.000001.007871.748031.000001.149611.629921.000001.244091.543311.000001.244091.543311.000001.409451.456691.000001.511811.299211.000001.661421.133861.000001.779531.031501.000001.779531.031501.000001.937010.992191.000001.984250.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.968751.000002.000000.929691.000002.000000.828121.000002.000000.750001.000002.000000.656251.000002.000000.601561.000002.000000.570311.000002.000000.492191.000002.000000.492191.000002.000000.359381.000002.000000.320311.000001.937010.304691.000001.913390.281251.000001.866140.242191.000001.850390.210941.000001.811020.203121.000001.803150.187501.000001.803150.179691.000001.803150.179691.000001.795280.171881.000001.795280.171881.000001.787400.164061.000001.771650.156251.000001.740160.109381.000001.708660.062501.000001.653540.031251.000001.614170.015621.000001.559060.007811.000001.535430.000001.000001.519690.000001.000001.480310.000001.000001.425200.000001.000001.377950.000001.000001.322830.000001.000001.291340.000001.000001.236220.000001.000001.157480.000001.000001.102360.000001.000001.047240.000001.000001.031500.000001.000001.007870.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000000.992190.000001.000000.992190.000001.000000.968750.000001.000000.929690.000001.000000.859380.000001.000000.796880.000001.000000.757810.000001.000000.710940.000001.000000.664060.000001.000000.625000.000001.000000.562500.000001.000000.523440.000001.000000.492190.000001.000000.468750.000001.000000.445310.000001.000000.390620.000001.000000.343750.093751.000000.281250.125001.000000.234380.179691.000000.203120.226561.000000.164060.265621.000000.117190.328121.000000.093750.375001.000000.054690.398441.000000.039060.445311.000000.023440.468751.000000.023440.492191.000000.007810.507811.000000.000000.523441.000000.000000.562501.000000.000000.609381.000000.000000.671881.000000.000000.710941.000000.000000.765621.000000.000000.789061.000000.000000.820311.000000.000000.828121.000000.000000.828121.000000.000000.828121.000000.000000.828121.000000.000000.835941.000000.000000.843751.000000.000000.843751.000000.000000.898441.000000.000000.906251.000000.000000.906251.000000.000000.914061.000000.000000.914061.000000.000000.914061.000000.000000.914061.000000.000000.914061.000000.000000.914061.000000.000000.914061.000000.000000.835941.000000.000000.781251.000000.000000.750001.000000.000000.695311.000000.000000.640621.000000.000000.617191.000000.000000.593751.000000.000000.593751.000000.000000.593751.000000.000000.593751.000000.000000.593751.000000.000000.593751.000000.000000.664061.000000.000000.718751.000000.000000.804691.000000.000000.875001.000000.000000.890621.000000.000000.921881.000000.000000.953121.000000.000000.953121.000000.000000.976561.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000001.000001.000000.000001.007871.000000.000001.031501.000000.000001.062991.000000.000001.110241.000000.000001.110241.000000.000001.181101.000000.000001.196851.000000.000001.204721.000000.000001.220471.000000.000001.220471.000000.000001.259841.000000.000001.267721.000000.000001.283461.000000.000001.330711.000000.000001.417321.000000.000001.511811.000000.000001.551181.000000.000001.582681.000000.000001.606301.000000.000001.661421.000000.000001.700791.000000.000001.740161.000000.093751.779531.000000.132811.842521.000000.210941.889761.000000.257811.952761.000000.304691.976381.000000.320312.000001.000000.367192.000001.000000.445312.000001.000000.500002.000001.000000.554692.000001.000000.570312.000001.000000.570312.000001.000000.570312.000001.000000.570312.000001.000000.570312.000001.000000.585942.000001.000000.617192.000001.000000.742192.000001.000000.742192.000001.000000.953121.842521.000001.000001.511811.000001.000001.173231.000001.000001.039371.000001.000001.039371.000001.000000.976561.000001.000000.937501.000000.929690.851561.000000.843750.781251.000000.843750.781251.000000.609380.625001.000000.507810.554691.000000.351560.523441.000000.234380.523441.000000.171880.585941.000000.117190.625001.000000.078120.671881.000000.054690.710941.000000.023440.796881.000000.007810.875001.000000.000000.960941.000000.000000.992191.000000.000000.992191.000000.000001.031501.000000.000001.125981.000000.000001.228351.000000.000001.393701.000000.000001.496061.000000.000001.637801.000000.000001.755911.000000.125001.818901.000000.203121.897641.000000.265621.960631.000000.382811.984251.000000.460941.984251.000000.570311.795281.000000.648441.606301.000000.671881.275591.000000.687501.110241.000000.687500.929691.000000.687500.929691.000000.593750.703121.000000.546880.453121.000000.523440.187501.000000.679690.062501.000000.679690.062501.000000.960940.000001.000000.992190.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.007870.000001.000001.015750.000001.000001.023620.000001.000001.023620.000001.000001.031500.000001.000001.055120.000001.000001.070870.000001.000001.110240.000001.000001.149610.000001.000001.188980.000001.000001.204720.000001.000001.204720.000001.000001.204720.000001.000001.204720.000001.000001.204720.000001.000001.133860.000001.000001.086610.000001.000001.015750.000001.000000.968750.000001.000000.914060.000001.000000.726560.000001.000000.484380.000001.000000.335940.132811.000000.203120.203121.000000.140620.328121.000000.062500.468751.000000.007810.648441.000000.000000.750001.000000.000000.812501.000000.000000.875001.000000.000000.929691.000000.000000.968751.000000.000000.992191.000000.000000.992191.000000.000000.992191.000000.000001.125981.000000.000001.346461.000000.000001.519691.000000.078121.748031.000000.078121.748031.000000.312501.960631.000000.382812.000001.000000.492192.000001.000000.632812.000001.000000.632812.000001.000000.812502.000001.000000.843752.000001.000000.843752.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.851562.000001.000000.859382.000001.000000.929692.000001.000001.015751.937011.000001.125981.637801.000001.417321.259841.000001.755911.070871.000001.913390.992191.000002.000000.992191.000002.000000.984381.000002.000000.960941.000002.000000.945311.000002.000000.945311.000002.000000.937501.000002.000000.929691.000002.000000.929691.000002.000000.929691.000002.000000.929691.000001.858271.448821.000001.629921.795281.000001.440941.874021.000001.291342.000001.000001.291342.000001.000001.133862.000001.000001.196852.000001.000001.354332.000001.000001.503942.000001.000001.503942.000001.000001.874021.669291.000001.944881.330711.000002.000001.047241.000002.000000.882811.000002.000000.750001.000002.000000.656251.000002.000000.617191.000002.000000.570311.000002.000000.484381.000002.000000.437501.000002.000000.390621.000002.000000.351561.000001.929130.328121.000001.913390.304691.000001.905510.304691.000001.905510.296881.000001.905510.296881.000001.905510.289061.000001.905510.289061.000001.897640.289061.000001.881890.265621.000001.834650.218751.000001.834650.218751.000001.811020.210941.000001.787400.171881.000001.787400.171881.000001.708660.093751.000001.700790.054691.000001.692910.046881.000001.685040.046881.000001.685040.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.677170.046881.000001.527560.023441.000001.401570.007811.000001.338580.000001.000001.204720.000001.000001.078740.000001.000001.015750.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.000001.000001.000000.078121.000001.000000.328121.000001.118110.570311.000001.291340.867191.000001.535431.015751.000001.708661.220471.000001.905511.362201.000001.984251.551181.000001.984251.582681.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.984251.590551.000001.992131.590551.000001.992131.590551.000001.992131.590551.000001.992131.590551.000001.992131.637801.000001.921261.716541.000001.826771.834651.000001.755911.881891.000001.748031.913391.000001.740161.913391.000001.803151.834651.000001.858271.629921.000001.952761.417321.000001.992131.133861.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.984381.000002.000000.937501.000002.000000.890621.000002.000000.843751.000002.000000.828121.000002.000000.828121.000002.000000.828121.000002.000000.828121.000002.000000.945311.000002.000000.945311.000002.000001.039371.000002.000001.133861.000002.000001.338581.000002.000001.582681.000001.913391.708661.000001.913391.708661.000001.708661.944881.000001.637801.976381.000001.598432.000001.000001.551182.000001.000001.551182.000001.000001.543312.000001.000001.543312.000001.000001.543312.000001.000001.543312.000001.000001.543312.000001.000001.614172.000001.000001.716541.842521.000001.826771.771651.000001.881891.653541.000001.952761.527561.000001.984251.417321.000002.000001.275591.000002.000001.212601.000002.000001.204721.000002.000001.204721.000002.000001.181101.000002.000001.133861.000002.000001.078741.000002.000001.039371.000002.000001.015751.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000001.000001.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000002.000000.992191.000001.929130.992191.000001.929130.992191.000001.913390.992191.000001.913390.992191.000001.881890.992191.000001.858270.992191.000001.811020.992191.000001.787400.992191.000001.787400.992191.000001.732280.992191.000001.724410.992191.000001.724410.992191.000001.724410.992191.000001.716540.992191.000001.708660.992191.000001.692910.992191.000001.692910.992190.929691.692910.992190.898441.692910.992190.843751.692910.992190.820311.692910.992190.796881.692910.992190.781251.692910.992190.773441.692910.992190.757811.692910.992190.757811.692910.992190.757811.692910.992190.757811.692910.992190.757811.692910.992190.757811.692910.992190.757811.755910.992190.757811.771650.992190.757811.779530.992190.757811.803150.992190.757811.834650.992190.757811.850390.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.757811.866140.992190.820311.866140.992190.820311.866140.992190.835941.779530.992190.843751.755910.992190.867191.740160.992190.882811.732280.992190.914061.732280.992190.921881.732280.992190.921881.732280.992190.929691.732280.992190.929691.732280.992190.929691.732280.992190.929691.732280.992190.929691.732280.992190.937501.732280.992190.937501.732280.992190.937501.732280.968750.937501.724410.914060.937501.716540.851560.937501.716540.828120.937501.708660.820310.937501.708660.812500.937501.708660.812500.937501.708660.812500.937501.708660.804690.937501.708660.796880.937501.708660.796880.937501.708660.757810.937501.708660.734380.937501.700790.734380.937501.685040.734380.937501.661420.734380.937501.622050.734380.937501.566930.734380.937501.535430.734380.937501.472440.734380.937501.472440.734380.937501.354330.734380.937501.322830.734380.875001.314960.734380.875001.314960.734380.867191.314960.734380.867191.307090.734380.867191.307090.734380.867191.307090.734380.867191.307090.734380.859381.307090.734380.859381.299210.734380.859381.299210.734380.859381.291340.734380.859381.291340.734380.859381.291340.734380.859381.291340.734380.859381.291340.734380.859381.291340.734380.859381.283460.734380.859381.283460.734380.859381.283460.734380.859381.346460.734380.859381.377950.734380.859381.377950.734380.859381.393700.734380.859381.409450.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.433070.734380.859381.440940.734380.859381.440940.734380.859381.440940.796880.859381.440940.804690.921881.440940.804690.937501.440940.804690.937501.440940.804690.945311.440940.820310.953121.448820.851560.976561.448820.851560.976561.464570.898441.000001.464570.937501.000001.464570.960941.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.992191.000001.464570.921881.000001.464570.828121.000001.472440.750001.000001.480310.648441.000001.488190.578121.000001.503940.531251.000001.503940.515621.000001.503940.492191.000001.503940.468751.000001.425200.421881.000001.362200.398441.000001.330710.375001.000001.322830.375001.000001.322830.375001.000001.322830.476561.000001.322830.585941.000001.322830.718751.000001.322830.773441.000001.322830.828121.000001.322830.859381.000001.322830.914061.000001.322830.968751.000001.322830.992191.000001.322830.992191.000001.322830.992191.000001.322830.992191.000001.299210.992191.000001.275590.992191.000001.244090.992191.000001.244090.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.212600.992191.000001.283460.992191.000001.307090.992191.000001.330710.992191.000001.330710.992191.000001.330710.992191.000001.330710.992191.000001.330710.992191.000001.330710.992191.000001.346460.992191.000001.362200.992191.000001.370080.992191.000001.385830.992191.000001.385830.992191.000001.385830.992191.000001.393700.992191.000001.409450.992191.000001.409450.992191.000001.409450.992191.000001.409450.992191.000001.409450.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.417320.992191.000001.425200.992191.000001.425200.992191.000001.425200.992191.000001.425200.992191.000001.425200.992191.000001.425200.992191.000001.425200.992191.000001.259840.992191.000001.118110.992191.000001.007870.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.000001.000000.992191.00000x";

//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
//Often, a REV Rotation is referred to as a Nic, although they mean different things.
//Truly, a Nic is ~17.976 REV Rotation values.
const double R_nicsConstant = 17.9761447906494;
//The change in encoder output per full wheel rotation around the axle.
//This value can be used to move a certain distance using solely encoder values.
const double R_kuhnsConstant = 8.3121115031;
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
const double R_swerveTrainAssumePositionTolerance = .1;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;
/*___End Global Robot Variable Settings___*/

//TODO: Why inverted?
const VectorDouble R_zionVectorForward(0, -1);
const VectorDouble R_zionVectorRight(-1,0);
const VectorDouble R_zionVectorBackward(0,1);
const VectorDouble R_zionVectorLeft(1,0);

const double R_circumfrenceWheel = 4 * M_PI;