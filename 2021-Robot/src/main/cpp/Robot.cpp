#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "Climber.h"
#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "Robot.h"
#include "RobotMap.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"
#include "auto/AutoStep.h"
#include "auto/AutoSequence.h"
#include "auto/steps/AssumeDirection.h"
#include "auto/steps/AssumeDistance.h"
#include "auto/steps/RunPrerecorded.h"
#include "Recorder.h"

Climber climber(R_PWMPortClimberMotorClimb, R_PWMPortClimberMotorTranslate, R_PWMPortClimberMotorWheel, R_PWMPortClimberServoLock, R_DIOPortSwitchClimberBottom);
frc::DigitalInput switchSwerveUnlock(R_DIOPortSwitchSwerveUnlock);
frc::XboxController *playerOne;
frc::XboxController *playerTwo;
Intake intake(R_CANIDMotorIntake);
Launcher launcher(R_CANIDMotorLauncherIndex, R_CANIDMotorLauncherLaunchOne, R_CANIDMotorLauncherLaunchTwo, R_PWMPortRightServo, R_PWMPortLeftServo);
Limelight limelight;
NavX navX(NavX::ConnectionType::kMXP);
Recorder recorder;
SwerveModule frontRightModule(R_CANIDZionFrontRightDrive, R_CANIDZionFrontRightSwerve);
SwerveModule frontLeftModule(R_CANIDZionFrontLeftDrive, R_CANIDZionFrontLeftSwerve);
SwerveModule rearLeftModule(R_CANIDZionRearLeftDrive, R_CANIDZionRearLeftSwerve);
SwerveModule rearRightModule(R_CANIDZionRearRightDrive, R_CANIDZionRearRightSwerve);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule, navX, recorder);
AutoSequence masterAuto;

void Robot::RobotInit() {

    playerOne = new frc::XboxController(R_controllerPortPlayerOne);
    playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);

    m_booleanClimberLock    = true;
    m_speedClimberClimb     = 0;
    m_speedClimberTranslate = 0;
    m_speedClimberWheel     = 0;
    m_speedIntake           = 0;
    m_speedLauncherIndex    = 0;
    m_speedLauncherLaunch   = 0;
    m_servoPosition         = 0;
    m_swerveBrake           = false;

    m_chooserAuto = new frc::SendableChooser<std::string>;
    m_chooserAuto->AddOption("Chooser::Auto::If-We-Gotta-Do-It", "dotl");
    m_chooserAuto->SetDefaultOption("Chooser::Auto::Run-PreRecorded", "prerec");
    frc::SmartDashboard::PutData(m_chooserAuto);

    frc::SmartDashboard::PutNumber("Field::Launcher::Speed-Index:", R_launcherDefaultSpeedIndex);
    frc::SmartDashboard::PutNumber("Field::Launcher::Speed-Launcher", R_launcherDefaultSpeed);
    frc::SmartDashboard::PutString("AutoStep::RunPrerecorded::Values", "");
    frc::SmartDashboard::PutString("Recorder::output_file_string", "");
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {

    //Set the zero position before beginning auto, as it should have been
    //calibrated before the match. This persists for the match duration unless
    //overriden.
    zion.setZeroPosition();
    navX.resetYaw();
    //Get which auto was selected to run in auto to test against.
    m_chooserAutoSelected = m_chooserAuto->GetSelected();
    
    //If-We-Gotta-Do-It simply drives off the line.
    if (m_chooserAutoSelected == "dotl") {

        masterAuto.AddStep(new AssumeDirection(zion, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDistance(zion, 30));
    }
    else if (m_chooserAutoSelected == "prerec") {

        //masterAuto.AddStep(new AssumeDistance(zion, 30));
        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "testingg"));
    }

    masterAuto.Init();
}
void Robot::AutonomousPeriodic() {

    //Lock the drive wheels before beginning for accuracy.
    zion.setDriveBrake(true);
    //Run the auto!
    if (masterAuto.Execute()) {

        zion.assumeNearestZeroPosition();
    }
}
void Robot::TeleopInit() {

    //To clean up adter auto, confirm the swerves are locked and unlock
    //the drive train, and go to the pre-calibrated zero position set up at the
    //beginning of auto to begin the match.
    //zion.setZeroPosition();
    zion.setSwerveBrake(true);
    zion.setDriveBrake(true);
}
void Robot::TeleopPeriodic() {

    if (playerOne->GetYButton()) {

        zion.setZeroPosition();
    }
    if (playerOne->GetBButton()) {

        navX.resetYaw();
    }
    if (playerOne->GetAButton()) {

        zion.assumeNearestZeroPosition();
    }
    else {
    
        zion.drive(playerOne->GetX(frc::GenericHID::kLeftHand), playerOne->GetY(frc::GenericHID::kLeftHand), playerOne->GetX(frc::GenericHID::kRightHand), playerOne->GetBumper(frc::GenericHID::kLeftHand), playerOne->GetBumper(frc::GenericHID::kRightHand));
    }


    //The second controller works in control layers on top of the basic
    //driving mode engaged with function buttons. If one of the functions
    //running under a button loses its button press, it will be overriden
    //by the regular mode. Useful for cancellation. Layers override each other.

    //The back button is "manual override" control layer. No auto, simply
    //writes unupdated values directly to motors, unlocking the climber,
    //with no execution caps or impediments. Overrides all other layers.
    if (playerTwo->GetBackButton()) {

        m_booleanClimberLock = false;
        m_speedClimberClimb = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate = playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel = playerTwo->GetX(frc::GenericHID::kRightHand);
        m_speedIntake = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedLauncherIndex = -playerTwo->GetY(frc::GenericHID::kLeftHand);
        m_speedLauncherLaunch = -playerTwo->GetY(frc::GenericHID::kRightHand);
    }
    else {

        m_booleanClimberLock = true;
        m_speedClimberClimb = 0;
        m_speedClimberTranslate = 0;
        m_speedClimberWheel = 0;
        m_speedIntake = 0;
        m_speedLauncherIndex = 0;
        m_speedLauncherLaunch = 0;
    }

    //The start button is "climber" control layer. Controls nothing but the
    //climber. Overrides the auto layer.
    if (!playerTwo->GetBackButton() && playerTwo->GetStartButton()) {

        m_speedClimberClimb = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate = playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel = playerTwo->GetX(frc::GenericHID::kRightHand);
        m_booleanClimberLock = !playerTwo->GetBumper(frc::GenericHID::kRightHand);
    }
    else {

        m_speedClimberClimb = 0;
        m_speedClimberTranslate = 0;
        m_speedClimberWheel = 0;
        m_booleanClimberLock = true;
    }

    //The center button is the "auto" control layer. Enables auto functions.
    //Overrides regular driving, but is overriden by all other layers.
    if (!playerTwo->GetBackButton() && !playerTwo->GetStartButton() && playerTwo->GetRawButton(9)) {}
    else {}

    //If no layers were engaged, regular driving can begin.
    if (!playerTwo->GetBackButton() && !playerTwo->GetStartButton() && !playerTwo->GetRawButton(9)) {

        m_speedIntake = (-playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand)) * R_executionCapIntake;

        if (playerTwo->GetAButton()) {

            m_speedLauncherIndex = frc::SmartDashboard::GetNumber("Field::Launcher::Speed-Index:", R_launcherDefaultSpeedIndex);
        }
        else {

            m_speedLauncherIndex = 0;
        }
        if (playerTwo->GetXButton()) {

            m_speedLauncherLaunch =frc::SmartDashboard::GetNumber("Field::Launcher::Speed-Launcher", R_launcherDefaultSpeed);
        }
        if (!playerTwo->GetXButton()) {

            m_speedLauncherLaunch = 0;
        }
        if (playerTwo->GetBumperPressed(frc::GenericHID::kLeftHand)) {

            m_servoPosition = m_servoPosition - 0.25;
        }
        else if (playerTwo->GetBumperPressed(frc::GenericHID::kRightHand)) {
           
            m_servoPosition = m_servoPosition + 0.25;
        }
    }

    //Once all layers have been evaluated, write out all of their values.
    //Doing this only once prevents weird bugs in which multiple different
    //values get set at different times in the loop.
    climber.lock(m_booleanClimberLock);
    climber.setSpeed(Climber::Motor::kClimb, m_speedClimberClimb);
    climber.setSpeed(Climber::Motor::kTranslate, m_speedClimberTranslate);
    climber.setSpeed(Climber::Motor::kWheel, m_speedClimberWheel);
    intake.setSpeed(m_speedIntake);
    launcher.setIndexSpeed(m_speedLauncherIndex);
    launcher.setLaunchSpeed(m_speedLauncherLaunch);
    launcher.setServo(Launcher::kSetFullRange, m_servoPosition);
}
void Robot::DisabledPeriodic() {

    //Whenever Zion is disabled, if the unlock swerve button is pressed and
    //held, unlock the swerves for zeroing. Once released, lock them again. The
    //switch is inverted by default, so no inversion is required. This is in
    //disabled on the off-chance that the switch got bumped during match play.
    if (switchSwerveUnlock.Get()) {

        if (!m_zeroButtonWasPressed) {

            m_zeroButtonWasPressed = true;
            m_swerveBrake = !m_swerveBrake;
            zion.setSwerveBrake(m_swerveBrake);
        }
    }
    else {

        m_zeroButtonWasPressed = false;
    }
    //Whenever Zion is on, allow control of the Limelight from P2. This permits
    //using it for manual alignment at any time, before or after the match.
    limelight.setLime(!m_swerveBrake);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
