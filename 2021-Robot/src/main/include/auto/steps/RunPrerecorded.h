#ifndef RUNPRERECORDED_H
#define RUNPRERECORDED_H

#include <iostream>
#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <fstream>
#include <sstream>
#include <frc/DriverStation.h>

#include "SwerveTrain.h"
#include "RobotMap.h"
#include "Limelight.h"

class RunPrerecorded : public AutoStep {

public:
    RunPrerecorded(SwerveTrain& refZion, Limelight &limeToSet, std::string pathToValues) : AutoStep("PreRecorded") {

        m_zion = &refZion;
        m_path = pathToValues;
        m_limelight = &limeToSet;
    }

    void _Init() {

        std::ifstream valuesFile("/u/" + m_path);
        if (valuesFile.is_open()) {
            
            std::ostringstream oss;
            oss << valuesFile.rdbuf();
            std::string stringValues = oss.str();
            if (stringValues.length() >= R_zionAutoControllerTotalDigits * 3) {
                
                if (stringValues.at(stringValues.length() - 1) == 'x') {

                    bool done = false;
                    int pos = 0;
                    while (!done) {

                        if (stringValues.at(pos * (R_zionAutoControllerTotalDigits * 3)) == 'x') {

                            done = true;
                        }
                        else {

                            ControllerState tempState;
                            tempState.x = std::stod(stringValues.substr(pos * (R_zionAutoControllerTotalDigits * 3), R_zionAutoControllerTotalDigits)) - 1;
                            tempState.y = std::stod(stringValues.substr(pos * (R_zionAutoControllerTotalDigits * 3) + R_zionAutoControllerTotalDigits, R_zionAutoControllerTotalDigits)) - 1;
                            tempState.z = std::stod(stringValues.substr(pos * (R_zionAutoControllerTotalDigits * 3) + R_zionAutoControllerTotalDigits * 2, R_zionAutoControllerTotalDigits)) - 1;
                            m_values.push_back(tempState);
                            pos++;
                        }
                    }
                    if (!m_values.empty()) {

                        m_currentValue = m_values.begin();
                        m_endValue = m_values.end();
                        _Log("File successfully parsed! The run should take " + std::to_string(m_values.size() / 20.0) + " seconds");
                    }
                    else {

                        _Log("File parsed was empty...?");
                    }
                }
                else {

                    _Log("Could not find EOF in recorded values");
                }
            }
            else {

                _Log("Not enough data");                
            }
        }
        else {

            _Log("Unable to open values file");
        }
    }

    bool _Execute() {

        if (!m_values.empty()) {

            // If we are at the end of the file
            if (m_currentValue == m_endValue) {

                m_zion->setSwerveSpeed(0);
                m_zion->setDriveSpeed(0);
                _Log("Finished executing recording");
                return true;
            }
            else {

                double x = m_currentValue->x;
                double y = m_currentValue->y;
                double z = m_currentValue->z;
                m_zion->drive(x, y, z, false, false);
                m_currentValue++;
                return false;
            }
        }
        else {

            m_zion->setSwerveSpeed(0);
            m_zion->setDriveSpeed(0);
            _Log("Finished executing recording; there was no data");
            return true;
        }
    }

    void _Cleanup() {}

    void _Log(std::string message) {

        Log("[" + m_path + "] " + message);
    }

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
    std::string m_path;
    Limelight* m_limelight;
};

#endif