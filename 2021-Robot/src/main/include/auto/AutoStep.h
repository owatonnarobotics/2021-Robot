#ifndef AUTOSTEP_H
#define AUTOSTEP_H

#include <string>
#include <frc/DriverStation.h>

class AutoStep {
    
    public:
        AutoStep(std::string name) {

            m_name = name;
        }

        void Init() {

            //wpi::outs() << m_name << " init\n";
            _Init();
        }

        bool Execute() {

            //wpi::outs() << m_name << " execute\n";
            return _Execute();
        }

        void Cleanup() {

            //wpi::outs() << m_name << " cleanup\n";
            _Cleanup();
        }

        void Log(std::string message) {

            frc::DriverStation::ReportError(message);
        }

    private:
        virtual void _Init() = 0;
        virtual bool _Execute() = 0;
        virtual void _Cleanup() = 0;

        std::string m_name;
};

#endif