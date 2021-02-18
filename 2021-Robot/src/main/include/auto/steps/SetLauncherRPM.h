#ifndef SETLAUNCHERRPM_H
#define SETLAUNCHERRPM_H

#include "Launcher.h"
#include "RobotMap.h"

class SetLauncherRPM : public AutoStep {

    public:
        SetLauncherRPM(Launcher &launcherToSet, const double rpmToSet, const bool async) : AutoStep("SetLauncherRPM") {

            m_launcher = &launcherToSet;
            m_rpm = rpmToSet;
            m_async = async;
        }

        void _Init() {

            m_launcher->SetRPM(m_rpm);
        }

        bool _Execute() {

            if (m_async) {

                return true;
            }
            else {

                return abs(m_launcher->GetRPM() - m_rpm) <= R_launcherSetRPMTolerance;
            }
        }

    private:
        Launcher* m_launcher;
        bool m_async;
        double m_rpm;
};

#endif