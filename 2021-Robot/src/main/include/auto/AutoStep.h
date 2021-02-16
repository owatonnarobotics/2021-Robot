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

            //std::cout << m_name << " init" << std::endl;
            _Init();
        }

        bool Execute() {

            //std::cout << m_name << " execute" << std::endl;
            return _Execute();
        }

        void Cleanup() {

            //std::cout << m_name << " cleanup" << std::endl;
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