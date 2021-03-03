#ifndef RECORDER_H
#define RECORDER_H

#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

class Recorder {

    public:

        Recorder() {

            m_log.str("");
            m_log.clear();
            m_counter = 0;
        }

        void Record(const double x, const double y, const double z, const bool precision) {

            m_log << std::setprecision(R_zionAutoControllerRecorderPrecision) << std::fixed << x + 1 << y + 1 << z + 1;// << (precision ? 1 : 0) << (limelightLock ? 1 : 0);
            SetStatus("Recording in progress...");
            m_counter++;
        }

        void Publish() {

            std::string newStr = m_log.str();
            if (newStr != "") {
                
                SetStatus(newStr);
                std::string outputString = frc::SmartDashboard::GetString("Recorder::output_file_string", "unknown");
                std::string fullPath = "/u/" + outputString;
                std::string logAsStr = m_log.str() + "x";
                std::ofstream myFile(fullPath, std::ios::out | std::ios::trunc);
                //myFile.flush();
                if (myFile.is_open()) {
                
                    frc::DriverStation::ReportError("About to write "/*the following to \"" + fullPath + "\": " + logAsStr + " with "*/ + std::to_string(m_counter) + " updates");
                    myFile << logAsStr;
                    myFile.close();
                }
                else {

                    frc::DriverStation::ReportError("Unable to open " + fullPath + " for recording");
                }
                frc::SmartDashboard::PutString("Recorder::output_file::" + outputString, logAsStr);
                m_log.str("");
                m_log.clear();
                m_counter = 0;
            }
        }

        void SetStatus(std::string status) {

            frc::SmartDashboard::PutString("Recorder::m_log", status);
        }

    private:
        std::stringstream m_log;
        int m_counter;
};

#endif