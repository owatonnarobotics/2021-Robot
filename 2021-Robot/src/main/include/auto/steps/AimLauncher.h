#ifndef AIMLAUNCHER_H
#define AIMLAUNCHER_H

#include "auto/AutoStep.h"
#include "Launcher.h"
#include "Limelight.h"

class AimLauncher : public AutoStep {

    public:
        AimLauncher(Launcher &refLauncher, Limelight &refLimelight) : AutoStep("AimLauncher") {

            m_launcher = &refLauncher;
            m_limelight = &refLimelight;
        }

        void Init() {}

        bool Execute() {

            double area = m_limelight->getTargetArea();
            double servoPosition = -812.644 * pow(area, 6) + 7108.25 * pow(area, 5) - 24539.6 * pow(area, 4) + 41879.3 * pow(area, 3) - 35627.7 * pow(area, 2) + 12700.6 * area -679.787;
            //frc::SmartDashboard::PutNumber("RAW SERVO POSITION", m_servoSpeed);
            servoPosition = (servoPosition < 0 || servoPosition > 180) ? (servoPosition < 0 ? 0 : 180) : servoPosition;
            m_launcher->setServo(Launcher::kSetAngle, servoPosition);
            return true;
        }

    private:
        Launcher* m_launcher;
        Limelight* m_limelight;
};

#endif