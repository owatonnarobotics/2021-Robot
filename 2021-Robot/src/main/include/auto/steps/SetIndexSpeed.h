#ifndef SETINDEXSPEED_H
#define SETINDEXSPEED_H

#include "Launcher.h"
#include "RobotMap.h"

class SetIndexSpeed : public AutoStep {

    public:
        SetIndexSpeed(Launcher &launcherToSet, const double speedToSet) : AutoStep("SetIndexSpeed") {

            m_launcher = &launcherToSet;
            m_speed = speedToSet;
        }

        void Init() {

            m_launcher->setIndexSpeed(m_speed);
        }

        bool Execute() {

            return true;
        }

    private:
        Launcher* m_launcher;
        double m_speed;
};

#endif