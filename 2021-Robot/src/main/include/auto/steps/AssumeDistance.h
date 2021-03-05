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

        void Init() {

            //At the first iteration, set the starting and goal values to
            //memory for comparison once operating (since we're translating
            //in a lateral direction, we only have to care about one encoder
            //value)...
            mInitialFrontRightDrivePosition = m_zion->m_frontRight->GetDrivePosition();
            //Calculate the end goal encoder value with circumference and the
            //known amount of encoder values per rotation...
            m_targetEncoderPosition = mInitialFrontRightDrivePosition + (m_targetDistance * R_kuhnsConstant) / R_circumfrenceWheel;
            frc::SmartDashboard::PutNumber("START", mInitialFrontRightDrivePosition);
            frc::SmartDashboard::PutNumber("END EXPECTED", m_targetEncoderPosition);
        }

        bool Execute() {

            //If we're not in tolerance for meeting the goal value (since
            //going to a distance generates no oscillation, zero can be
            //used as a tolerance)...
            frc::SmartDashboard::PutNumber("END ACTUAL", m_zion->m_frontRight->GetDrivePosition());
            double delta = m_targetEncoderPosition - m_zion->m_frontRight->GetDrivePosition();
            if (abs(delta) > R_kuhnsConstant * .1) {

                m_zion->SetDriveSpeed(.15);
                //If we made it to here, we didn't succeed, so return false for
                //another go at it.
                return false;
            }
            //If we were...
            else {

                //Stop moving, clean up, and return true.
                m_zion->Stop();
                return true;
            }
            
        }

    private:
        SwerveTrain* m_zion;
        double mInitialFrontRightDrivePosition;
        double m_targetEncoderPosition;
        double m_targetDistance;
};

#endif