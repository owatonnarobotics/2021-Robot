#include "SwerveTrain.h"
#include "Limelight.h"
#include "NavX.h"

class AssumeRotationDegrees : public AutoStep {

    public:
        AssumeRotationDegrees(SwerveTrain &refZion, Limelight &refLimelight, NavX &refNavX, const double &degreesToRotate) {

            m_zion = &refZion;
            m_limelight = &refLimelight;
            m_navX = &refNavX;
            m_targetDegreesToRotate = degreesToRotate;
        }

        void Init() {

            m_initalAngle = m_navX->getAngle();
            m_resultingAngle = m_utilityVarOne + degreesToRotate;
        }

        bool Execute() {

            //Set the wheels to their diagonal positions (at incremental 45*
            //angles found with radians converted into Nics). This works as
            //this section of the code also runs as a loop, and these functions
            //handle setting values down to zero once correct...
            m_zion->m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
            m_zion->m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
            m_zion->m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
            m_zion->m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);

            //If we're not within tolerance for meeting the goal angle...
            if (abs(m_resultingAngle - m_navX->getAngle()) > R_zionAutoToleranceAngle) {

                //If rotation needs to be clockwise (goal is greater than init)
                //set the turning speed to be positive, otherise, set it
                //negative (due to the way the wheels align, these values are
                //inverted)...
                m_zion->setDriveSpeed(m_resultingAngle > m_initalAngle ? -R_zionAutoMovementSpeedLateral : R_zionAutoMovementSpeedLateral);
            }
            //If we were within tolerance that iteration...
            else {

                //Stop moving, reset the wheels, clean up, and return true.
                m_zion->setDriveSpeed();
                //Since this is a core auto function, it must be run as
                //a loop, so do so and then continue with the rest of cleanup.
                //TODO: why was this here?: while (!zionAssumeDirection(ZionDirections::kForward));
                return true;
            }
            //If we made it to here, we didn't succeed, so return false for
            //another go at it.
            return false;
        }

    private:
        SwerveTrain* m_zion;
        Limelight* m_limelight;
        NavX* m_navX;
        double m_targetDegreesToRotate;
        double m_initalAngle;
        double m_resultingAngle;
};