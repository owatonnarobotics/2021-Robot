#ifndef RUNPRERECORDED_H
#define RUNPRERECORDED_H

#include <string>
#include <frc/SmartDashboard.h>

class RunPrerecorded : public AutoStep {

    public:
        RunPrerecorded(SwerveTrain &refZion) : AutoStep("PreRecorded") {

            m_zion = &refZion;
        }

        void _Init() {

            m_virtualValues = frc::SmartDashboard::GetString("AutoStep::RunPrerecorded::Values", "\0")
        }

        bool _Execute() {

            return true;
        }

        void _Cleanup() {

            m_zion->setSwerveSpeed();
            m_zion->setDriveSpeed();
        }

    private:
        SwerveTrain* m_zion;
        std::string m_virtualValues;
};

#endif