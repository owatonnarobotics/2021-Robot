#ifndef RUNPRERECORDED_H
#define RUNPRERECORDED_H

#include <iostream>
#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>

class RunPrerecorded : public AutoStep {

    public:
        RunPrerecorded(SwerveTrain &refZion) : AutoStep("PreRecorded") {

            m_zion = &refZion;
        }

        void _Init() {

            std::string stringValues = frc::SmartDashboard::GetString("AutoStep::RunPrerecorded::Values", "\0");
            const int totalLength = stringValues.length() + 1;
            m_virtualValues = new char[totalLength];
            memcpy(m_virtualValues, stringValues.c_str(), sizeof(char) * totalLength);
            m_currentChar = m_virtualValues;
        }

        bool _Execute() {

            // If we are at the end of the file
            if (*m_currentChar == '\0') {

                m_zion->setSwerveSpeed();
                m_zion->setDriveSpeed();
                return true;
            }
            else {


                double x = getNextDouble();
                double y = getNextDouble();
                double z = getNextDouble();

                m_zion->driveController(nullptr, false, true, x, y, z);
                return false;
            }
            return true;
        }

        double getNextDouble() {

            const char* m_searchChar = m_currentChar;
            int counter = 0;
            bool searching = true;
            while (searching) {

                if (*m_searchChar == '-' || *m_searchChar == '\0') {

                    searching = false;
                }
                else {

                    counter++;
                    m_searchChar++;
                }
            }
            char* stringDouble = new char[counter];
            memcpy(stringDouble, m_currentChar, sizeof(char) * counter);
            m_currentChar += counter + 1;
            char* endPtr;
            double toReturn = strtod(stringDouble, &endPtr);
            return toReturn;
        }

        void _Cleanup() {

            delete[] m_virtualValues;
        }

    private:
        SwerveTrain* m_zion;
        char* m_virtualValues;
        const char* m_currentChar;
};

#endif