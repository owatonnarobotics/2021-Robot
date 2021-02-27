#ifndef AUTOSTEP_H
#define AUTOSTEP_H

#include <string>
#include <frc/DriverStation.h>

class AutoStep {
    
    public:
        AutoStep(std::string name) {

            m_name = name;
        }

        virtual void Init() = 0;
        virtual bool Execute() = 0;

        void Log(std::string message) {

            frc::DriverStation::ReportError(message);
        }

    private:
        std::string m_name;
};

#endif