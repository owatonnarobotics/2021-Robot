#ifndef RECORDER_H
#define RECORDER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>

class Recorder {

    public:

        Recorder() {}

        void Record(const double x, const double y, const double z) {

            m_log << std::setprecision(R_zionAutoJoystickRecorderPrecision) << std::fixed << x << y << z; 
            SetStatus("Recording in progress...");
        }

        void Publish() {

            SetStatus(m_log.str());
        }

        void SetStatus(std::string status) {

            frc::SmartDashboard::PutString("Recorder::m_log", status);
        }

    private:
        std::stringstream m_log;
};

#endif