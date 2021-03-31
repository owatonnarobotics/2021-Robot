#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Climber.h"
#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "Robot.h"
#include "RobotMap.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"
#include "Controller.h"

// Auto
#include "auto/AutoStep.h"
#include "auto/AutoSequence.h"
#include "auto/AsyncLoop.h"
#include "auto/steps/AssumeDirectionAbsolute.h"
#include "auto/steps/AssumeDistance.h"
#include "auto/steps/RunPrerecorded.h"
#include "auto/steps/SetLauncherRPM.h"
#include "auto/steps/SetIndexSpeed.h"
#include "auto/steps/AimLauncher.h"
#include "auto/steps/WaitSeconds.h"
#include "auto/steps/LimelightLock.h"
#include "auto/Recorder.h"

Climber climber(R_PWMPortClimberMotorClimb, R_PWMPortClimberMotorTranslate, R_PWMPortClimberMotorWheel, R_PWMPortClimberServoLock, R_DIOPortSwitchClimberBottom);
frc::DigitalInput switchSwerveUnlock(R_DIOPortSwitchSwerveUnlock);
frc::XboxController *playerOne;
frc::XboxController *playerTwo;
frc::Joystick *playerThree;
Intake intake(R_CANIDMotorIntake);
Launcher launcher(R_CANIDMotorLauncherIndex, R_CANIDMotorLauncherLaunchOne, R_CANIDMotorLauncherLaunchTwo, R_PWMPortRightServo, R_PWMPortLeftServo);
Limelight limelight;
NavX navX(NavX::ConnectionType::kMXP);
Recorder recorder;
SwerveModule frontRightModule(R_CANIDZionFrontRightDrive, R_CANIDZionFrontRightSwerve);
SwerveModule frontLeftModule(R_CANIDZionFrontLeftDrive, R_CANIDZionFrontLeftSwerve);
SwerveModule rearLeftModule(R_CANIDZionRearLeftDrive, R_CANIDZionRearLeftSwerve);
SwerveModule rearRightModule(R_CANIDZionRearRightDrive, R_CANIDZionRearRightSwerve);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule, navX);
AutoSequence masterAuto(false);

void Robot::RobotInit() {

    playerOne = new frc::XboxController(R_controllerPortPlayerOne);
    playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);
    playerThree = new frc::Joystick(R_controllerPortPlayerThree);

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
    m_chooserAuto->AddOption("Chooser::Auto::Path A Recorded", "Path A Recorded");
    m_chooserAuto->AddOption("Chooser::Auto::Path A Non-Pre-recorded", "Path A Non-Pre-recorded");
    m_chooserAuto->AddOption("Chooser::Auto::Path A Recorded and shoot", "Path A Recorded and shoot");
    m_chooserAuto->AddOption("Chooser::Auto::Path B Recorded", "Path B Recorded");
    m_chooserAuto->AddOption("Chooser::Auto::AutoNav Challenge::Barrel Racing Path", "brp");
    m_chooserAuto->AddOption("Chooser::Auto::AutoNav Challenge::Slalom Path", "sp");
    m_chooserAuto->AddOption("Chooser::Auto::AutoNav Challenge::Bounce Path", "bp");
    m_chooserAuto->AddOption("Chooser::Auto::Launch Power Cells", "Launch Power Cells");
    m_chooserAuto->SetDefaultOption("Chooser::Auto::Test Pre-recorded", "test pre-recorded");
    frc::SmartDashboard::PutData(m_chooserAuto);

    m_chooserController = new frc::SendableChooser<std::string>;
    m_chooserController->AddOption("Chooser::Controller::XboxController", "XboxController");
    m_chooserController->SetDefaultOption("Chooser::Controller::Joystick", "Joystick");
    frc::SmartDashboard::PutData(m_chooserController);

    frc::SmartDashboard::PutNumber("Field::Launcher::Speed-Index:", R_launcherDefaultSpeedIndex);
    frc::SmartDashboard::PutNumber("Field::Launcher::Speed-Launcher", R_launcherDefaultSpeed);
    frc::SmartDashboard::PutString("AutoStep::RunPrerecorded::Values", "");
    frc::SmartDashboard::PutString("Recorder::output_file_string", "");
    frc::SmartDashboard::PutNumber("LimelightLock end", .25);
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {

    masterAuto.Reset();
    //Set the zero position before beginning auto, as it should have been
    //calibrated before the match. This persists for the match duration unless
    //overriden.
    zion.SetZeroPosition();
    navX.resetYaw();
    //Get which auto was selected to run in auto to test against.
    m_chooserAutoSelected = m_chooserAuto->GetSelected();
    
    //If-We-Gotta-Do-It simply drives off the line.
    if (m_chooserAutoSelected == "dotl") {

        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDistance(zion, 30, SwerveTrain::ZionDirections::kLeft));
    }
    else if (m_chooserAutoSelected == "Path A Recorded") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "path-a"));
    }
    else if (m_chooserAutoSelected == "Path A Recorded and shoot") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "path-a"));
        
        AsyncLoop* spoolUp = new AsyncLoop;
        spoolUp->AddStep(new SetLauncherRPM(launcher, R_launcherDefaultSpeed, true));
        spoolUp->AddStep(new AimLauncher(launcher, limelight));
        spoolUp->AddStep(new LimelightLock(zion, limelight));
        spoolUp->AddStep(new WaitSeconds(5));
        masterAuto.AddStep(spoolUp);

        AutoSequence* indexLoop = new AutoSequence(true);
        indexLoop->AddStep(new SetIndexSpeed(launcher, R_launcherDefaultSpeedIndex));
        indexLoop->AddStep(new WaitSeconds(0.25));
        indexLoop->AddStep(new SetIndexSpeed(launcher, 0.0));
        indexLoop->AddStep(new WaitSeconds(1));
        
        AsyncLoop* loop = new AsyncLoop;
        loop->AddStep(indexLoop);
        loop->AddStep(new LimelightLock(zion, limelight));
        loop->AddStep(new AimLauncher(launcher, limelight));
        masterAuto.AddStep(loop);
    }
    else if (m_chooserAutoSelected == "Path A Non-Pre-recorded") {
    
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kRight));
        masterAuto.AddStep(new AssumeDistance(zion, 134, SwerveTrain::ZionDirections::kRight));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kBackward));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kBackward));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, new VectorDouble(143, 7)));
        masterAuto.AddStep(new AssumeDistance(zion, 143, new VectorDouble(143, 7)));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kBackward));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kBackward));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, new VectorDouble(60, -60)));
        masterAuto.AddStep(new AssumeDistance(zion, 84.85281374, new VectorDouble(60, -60)));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kRight));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kRight));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDistance(zion, 53, SwerveTrain::ZionDirections::kForward));
        masterAuto.AddStep(new AssumeDirectionAbsolute(zion, SwerveTrain::ZionDirections::kLeft));
        masterAuto.AddStep(new AssumeDistance(zion, 284, SwerveTrain::ZionDirections::kLeft));
    }
    else if (m_chooserAutoSelected == "Path B Recorded") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "path-b"));
    }
    else if (m_chooserAutoSelected == "brp") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "brp"));
    }
    else if (m_chooserAutoSelected == "sp") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "sp"));
    }
    else if (m_chooserAutoSelected == "bp") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "bp"));
    }
    else if (m_chooserAutoSelected == "Launch Power Cells") {

        AsyncLoop* spoolUp = new AsyncLoop;
        spoolUp->AddStep(new SetLauncherRPM(launcher, R_launcherDefaultSpeed, true));
        spoolUp->AddStep(new AimLauncher(launcher, limelight));
        spoolUp->AddStep(new LimelightLock(zion, limelight));
        spoolUp->AddStep(new WaitSeconds(5));
        masterAuto.AddStep(spoolUp);

        AutoSequence* indexLoop = new AutoSequence(true);
        indexLoop->AddStep(new SetIndexSpeed(launcher, R_launcherDefaultSpeedIndex));
        indexLoop->AddStep(new WaitSeconds(0.25));
        indexLoop->AddStep(new SetIndexSpeed(launcher, 0.0));
        indexLoop->AddStep(new WaitSeconds(1));
        
        AsyncLoop* loop = new AsyncLoop;
        loop->AddStep(indexLoop);
        loop->AddStep(new LimelightLock(zion, limelight));
        loop->AddStep(new AimLauncher(launcher, limelight));
        masterAuto.AddStep(loop);
    }
    else if (m_chooserAutoSelected == "test pre-recorded") {

        masterAuto.AddStep(new RunPrerecorded(zion, limelight, "test"));
    }

    masterAuto.Init();
}
void Robot::AutonomousPeriodic() {

    //Lock the drive and swerve wheels before beginning for accuracy.
    zion.SetSwerveBrake(true);
    zion.SetDriveBrake(true);
    //Run the auto!
    if (masterAuto.Execute()) {

        zion.AssumeZeroPosition();
    }
}
void Robot::TeleopInit() {

    //To clean up adter auto, confirm the swerves and drives are locked
    zion.SetSwerveBrake(true);
    zion.SetDriveBrake(true);
    navX.resetYaw();
}
void Robot::TeleopPeriodic() {
    
    zion.PrintDrivePositions();

    double x;
    double y;
    double z;
    if (m_chooserController->GetSelected() == "XboxController") {
        
        x = playerOne->GetX(frc::GenericHID::kLeftHand);
        y = playerOne->GetY(frc::GenericHID::kLeftHand);
        z = playerOne->GetX(frc::GenericHID::kRightHand);
    }
    else {

        x = playerThree->GetX();
        y = playerThree->GetY();
        z = playerThree->GetZ();
    }
    Controller::forceControllerXYZToZeroInDeadzone(x, y, z);
    z *= R_executionCapZionZ;

    if (m_chooserController->GetSelected() == "XboxController") {

        if (playerOne->GetYButton()) {

            zion.SetZeroPosition();
        }
        if (playerOne->GetBButton()) {

            navX.resetYaw();
        }
        if (playerOne->GetAButton()) {

            zion.AssumeZeroPosition();
        }
        else {

            zion.Drive(
                -x,
                -y,
                playerOne->GetBumper(frc::GenericHID::kLeftHand) ? limelight.CalculateLimelightLockSpeed() : z,
                playerOne->GetBumper(frc::GenericHID::kLeftHand),
                false,
                false
            );
        }
        if (playerOne->GetXButton()) {

            recorder.Record(x, y, z, false);
        }
        else {
           
            recorder.Publish();
        }
    }
    else {

        if (playerThree->GetRawButton(3)) {

            zion.SetZeroPosition();
        }
        if (playerThree->GetRawButton(4)) {

            navX.resetYaw();
        }
        if (playerThree->GetRawButton(12)) {

            zion.AssumeZeroPosition();
        }
        else {

            zion.Drive(
                -x,
                -y,
                playerThree->GetRawButton(6) ? limelight.CalculateLimelightLockSpeed() : z,
                playerThree->GetRawButton(5),
                playerThree->GetRawButton(7),
                playerThree->GetRawButton(2),
                -(((playerThree->GetThrottle() + 1.0) / 2.0) - 1.0)
            );
        }
        if (playerThree->GetRawButton(1)) {

            recorder.Record(x, y, z, false);
        }
        else {

            recorder.Publish();
        }
    }

    //The second controller works in control layers on top of the basic
    //driving mode engaged with function buttons. If one of the functions
    //running under a button loses its button press, it will be overriden
    //by the regular mode. Useful for cancellation. Layers override each other.

    //The back button is "manual override" control layer. No auto, simply
    //writes unupdated values directly to motors, unlocking the climber,
    //with no execution caps or impediments. Overrides all other layers.
    if (playerTwo->GetBackButton()) {

        m_booleanClimberLock =      false;
        m_speedClimberClimb =       -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate =    playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel =        playerTwo->GetX(frc::GenericHID::kRightHand);
        m_speedIntake =             -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedLauncherIndex =      -playerTwo->GetY(frc::GenericHID::kLeftHand);
        m_speedLauncherLaunch =     -playerTwo->GetY(frc::GenericHID::kRightHand);
    }
    else {

        m_booleanClimberLock =      true;
        m_speedClimberClimb =       0;
        m_speedClimberTranslate =   0;
        m_speedClimberWheel =       0;
        m_speedIntake =             0;
        m_speedLauncherIndex =      0;
        m_speedLauncherLaunch =     0;
    }

    //The start button is "climber" control layer. Controls nothing but the
    //climber. Overrides the auto layer.
    if (!playerTwo->GetBackButton() && playerTwo->GetStartButton()) {

        m_speedClimberClimb =       -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate =    playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel =        playerTwo->GetX(frc::GenericHID::kRightHand);
        m_booleanClimberLock =      !playerTwo->GetBumper(frc::GenericHID::kRightHand);
    }
    else {

        m_speedClimberClimb =       0;
        m_speedClimberTranslate =   0;
        m_speedClimberWheel =       0;
        m_booleanClimberLock =      true;
    }

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

            m_servoPosition = m_servoPosition - 90;
        }
        else if (playerTwo->GetBumperPressed(frc::GenericHID::kRightHand)) {
           
            m_servoPosition = m_servoPosition + 90;
        }
    }

    // Sets the servos to a position based on a quintic regression model with
    // respect to the area detected by the limelight
    double area = limelight.getTargetArea();
    m_servoPosition = -812.644 * pow(area, 6) + 7108.25 * pow(area, 5) - 24539.6 * pow(area, 4) + 41879.3 * pow(area, 3) - 35627.7 * pow(area, 2) + 12700.6 * area -679.787;
    m_servoPosition = (m_servoPosition < 0 || m_servoPosition > 180) ? (m_servoPosition < 0 ? 0 : 180) : m_servoPosition;

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
    launcher.setServo(Launcher::kSetAngle, m_servoPosition);
}
void Robot::DisabledPeriodic() {

    //Whenever Zion is disabled, check if the lock switch has been pressed. If
    //so, toggle the current swerve module lock state. This is useful when
    //zeroing the wheels (yay zero team).
    if (switchSwerveUnlock.Get()) {

        if (!m_zeroButtonWasPressed) {

            m_zeroButtonWasPressed = true;
            m_swerveBrake = !m_swerveBrake;
            zion.SetSwerveBrake(m_swerveBrake);
        }
    }
    else {

        m_zeroButtonWasPressed = false;
    }
    //Whenever Zion is on, allow control of the Limelight from P2. This permits
    //using it for manual alignment at any time, before or after the match.
    //Also turn on if the swerve modules are in coast.
    limelight.setLime(!m_swerveBrake || playerTwo->GetBumper(frc::GenericHID::kLeftHand));
    limelight.setProcessing(playerTwo->GetBumper(frc::GenericHID::kLeftHand));
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
