#ifndef WAITSECONDS_H
#define WAITSECONDS_H

#include "auto/AutoStep.h"

class WaitSeconds : public AutoStep {

    public:
        WaitSeconds(const double &seconds) : AutoStep("WaitSeconds") {

            m_secondsToWait = seconds;
        }

        void Init() {

            m_initialTime = frc::GetTime();
        }

        bool Execute() {

            return frc::GetTime() - m_initialTime >= m_secondsToWait;
        }

    private:
        double m_secondsToWait;
        double m_initialTime;
};

#endif