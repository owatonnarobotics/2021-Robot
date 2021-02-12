#ifndef RECORDER_H
#define RECORDER_H

#include <sstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <frc/smartdashboard/SmartDashboard.h>

class Recorder {

    public:

        Recorder() {

            m_old = "";
        }

        void Record(const double x, const double y, const double z) {

            m_log << std::setprecision(R_zionAutoJoystickRecorderPrecision) << std::fixed << x + 1 << y + 1 << z + 1; 
            SetStatus("Recording in progress...");
        }

        void Publish() {

            std::string newStr = m_log.str();
            if (newStr != m_old) {
                
                m_old = newStr;
                SetStatus(newStr);
                std::ofstream usb;
                usb.open("/u/data.txt");
                usb << m_log.str();
                usb.close();
            }
        }

        void SetStatus(std::string status) {

            frc::SmartDashboard::PutString("Recorder::m_log", status);
        }

    private:
        std::stringstream m_log;
        std::string m_old;
};

#endif