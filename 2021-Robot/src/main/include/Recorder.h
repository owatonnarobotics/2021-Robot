#ifndef RECORDER_H
#define RECORDER_H

#include <sstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
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
                std::string fullPath = "/u/" + frc::SmartDashboard::GetString("Recorder::output_file_string", "unknown");
                remove(fullPath.c_str());
                std::fstream file;
                file.open(fullPath, std::ios::out);
                file << m_log.str() << "x";
                file.close();
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