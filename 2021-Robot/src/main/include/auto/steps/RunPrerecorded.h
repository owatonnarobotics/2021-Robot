#ifndef RUNPRERECORDED_H
#define RUNPRERECORDED_H

#include <iostream>
#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>

#include "SwerveTrain.h"
#include "RobotMap.h"
#include "Limelight.h"

class RunPrerecorded : public AutoStep {

public:
    RunPrerecorded(SwerveTrain& refZion, Limelight &limeToSet, std::string values) : AutoStep("PreRecorded") {

        m_zion = &refZion;
        m_strValues = values;
        m_limelight = &limeToSet;
    }

    void _Init() {

        std::string stringValues = m_strValues;//frc::SmartDashboard::GetString("AutoStep::RunPrerecorded::Values", "") + "x";
        bool done = false;
        int pos = 0;
        if (stringValues.length() >= R_zionAutoJoystickTotalDigits * 3) {

            while (!done) {

                if (stringValues.at(pos * (R_zionAutoJoystickTotalDigits * 3)) == 'x') {

                    done = true;
                }
                else {

                    ControllerState tempState;
                    tempState.x = std::stod(stringValues.substr(pos * (R_zionAutoJoystickTotalDigits * 3), R_zionAutoJoystickTotalDigits)) - 1;
                    tempState.y = std::stod(stringValues.substr(pos * (R_zionAutoJoystickTotalDigits * 3) + R_zionAutoJoystickTotalDigits, R_zionAutoJoystickTotalDigits)) - 1;
                    tempState.z = std::stod(stringValues.substr(pos * (R_zionAutoJoystickTotalDigits * 3) + R_zionAutoJoystickTotalDigits * 2, R_zionAutoJoystickTotalDigits)) - 1;
                    m_values.push_back(tempState);
                    pos++;
                }
            }
            if (!m_values.empty()) {

                m_currentValue = m_values.begin();
                m_endValue = m_values.end();
                //wpi::outs() << "Length of values: " << m_values.size() << "\n";
                //wpi::outs() << "This routine should take " << m_values.size() / 20.0 << " seconds" << "\n";
            }
        }
    }

    bool _Execute() {

        if (!m_values.empty()) {

            // If we are at the end of the file
            if (m_currentValue == m_endValue) {

                m_zion->setSwerveSpeed(0);
                m_zion->setDriveSpeed(0);
                m_limelight->setLime(true);
                return true;
            }
            else {

                double x = m_currentValue->x;
                double y = m_currentValue->y;
                double z = m_currentValue->z;
                m_zion->driveController(x, y, z, false, false);
                m_currentValue++;
                return false;
            }
        }
        else {

            m_zion->setSwerveSpeed(0);
            m_zion->setDriveSpeed(0);
            return true;
        }
    }

    void _Cleanup() {}

    struct ControllerState {

        double x;
        double y;
        double z;
    };

private:
    SwerveTrain* m_zion;
    std::vector<ControllerState> m_values;
    std::vector<ControllerState>::iterator m_currentValue;
    std::vector<ControllerState>::iterator m_endValue;
    std::string m_strValues;
    Limelight* m_limelight;
};

#endif