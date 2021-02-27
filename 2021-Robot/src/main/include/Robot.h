#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {

    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void DisabledPeriodic() override;

    private:
        frc::SendableChooser<std::string> *m_chooserAuto;
        frc::SendableChooser<std::string> *m_chooserController;
        std::string m_chooserAutoSelected;

        //These are used such that each speed is only set once for P2.
        //Prevents weird assignment bugs with motor speeds.
        bool m_booleanClimberLock;
        double m_speedClimberClimb;
        double m_speedClimberTranslate;
        double m_speedClimberWheel;
        double m_speedIntake;
        double m_speedLauncherIndex;
        double m_speedLauncherLaunch;
        double m_servoPosition;
        double m_zeroButtonWasPressed;
        double m_swerveBrake;
};
