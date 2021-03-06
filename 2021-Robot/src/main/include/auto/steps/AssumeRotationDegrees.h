#ifndef ASSUMEROTATIONDEGREES_H
#define ASSUMEROTATIONDEGREES_H

#include <string>

#include "SwerveTrain.h"
#include "Limelight.h"
#include "NavX.h"

class AssumeRotationDegrees : public AutoStep {

    public:
        AssumeRotationDegrees(
            SwerveTrain &refZion,
            Limelight &refLimelight,
            NavX &refNavX,
            const double &degreesToRotate
        ) : AutoStep("AssumeRotationDegrees") {

            m_zion = &refZion;
            m_limelight = &refLimelight;
            m_navX = &refNavX;
            m_targetDegreesToRotate = degreesToRotate;
        }

        void Init() {

            mInitalAngle = m_navX->getAngle();
            m_resultingAngle = m_utilityVarOne + degreesToRotate;
        }

        bool Execute() {

            //Set the wheels to their diagonal positions (at incremental 45*
            //angles found with radians converted into Nics). This works as
            //this section of the code also runs as a loop, and these functions
            //handle setting values down to zero once correct...
            if (m_zion->AssumeTurnAroundCenterPositions()) {

                //If we're not within tolerance for meeting the goal angle...
                if (abs(m_resultingAngle - m_navX->getAngle()) > R_zionAutoToleranceAngle) {

                    //If rotation needs to be clockwise (goal is greater than init)
                    //set the turning speed to be positive, otherise, set it
                    //negative (due to the way the wheels align, these values are
                    //inverted)...
                    m_zion->SetDriveSpeed(m_resultingAngle > mInitalAngle ? -R_zionAutoMovementSpeedLateral : R_zionAutoMovementSpeedLateral);
                }
                //If we were within tolerance that iteration...
                else {

                    //Stop moving, reset the wheels, clean up, and return true.
                    m_zion->Stop();
                    return true;
                }
            }
            else {

                m_zion->SetDriveSpeed();
            }
            return false;
        }

    private:
        SwerveTrain* m_zion;
        Limelight* m_limelight;
        NavX* m_navX;
        double m_targetDegreesToRotate;
        double mInitalAngle;
        double m_resultingAngle;
};

#endif