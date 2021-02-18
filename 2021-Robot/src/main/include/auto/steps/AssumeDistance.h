#ifndef ASSUMEDISTANCE_H
#define ASSUMEDISTANCE_H

#include <string>

#include "SwerveTrain.h"
#include "RobotMap.h"

class AssumeDistance : public AutoStep {

    public:
        AssumeDistance(SwerveTrain &refZion, const double& distanceToAssume) : AutoStep("AssumeDistance") {

            m_zion = &refZion;
            m_targetDistance = distanceToAssume;
        }

        void _Init() {

            //At the first iteration, set the starting and goal values to
            //memory for comparison once operating (since we're translating
            //in a lateral direction, we only have to care about one encoder
            //value)...
            m_initialFrontRightDrivePosition = m_zion->m_frontRight->getDrivePosition();
            //Calculate the end goal encoder value with circumference and the
            //known amount of encoder values per rotation...
            m_targetEncoderPosition = m_initialFrontRightDrivePosition + ((m_targetDistance / R_circumfrenceWheel) * R_kuhnsConstant);
        }

        bool _Execute() {

            //If we're not in tolerance for meeting the goal value (since
            //going to a distance generates no oscillation, zero can be
            //used as a tolerance)...
            if (m_targetEncoderPosition - m_zion->m_frontRight->getDrivePosition() > 0) {

                m_zion->setDriveSpeed(R_zionAutoMovementSpeedLateral);
                //If we made it to here, we didn't succeed, so return false for
                //another go at it.
                return false;
            }
            //If we were...
            else {

                //Stop moving, clean up, and return true.
                m_zion->setDriveSpeed();
                return true;
            }
            
        }

        void _Cleanup() {}

    private:
        SwerveTrain* m_zion;
        double m_initialFrontRightDrivePosition;
        double m_targetEncoderPosition;
        double m_targetDistance;
};

#endif