#ifndef ASSUMEDISTANCE_H
#define ASSUMEDISTANCE_H

#include <string>

#include "SwerveTrain.h"
#include "VectorDouble.h"
#include "RobotMap.h"

class AssumeDistance : public AutoStep {

    public:
        AssumeDistance(SwerveTrain &refZion, const double& distanceToAssume, const int &directionToMove) : AutoStep("AssumeDistance") {

            m_zion = &refZion;
            m_targetDistance = distanceToAssume;
            switch (directionToMove) {

                case SwerveTrain::ZionDirections::kForward: m_direction = new VectorDouble(0, 1); break;
                case SwerveTrain::ZionDirections::kRight: m_direction = new VectorDouble(1, 0); break;
                case SwerveTrain::ZionDirections::kBackward: m_direction = new VectorDouble(0, -1); break;
                case SwerveTrain::ZionDirections::kLeft: m_direction = new VectorDouble(-1, 0); break;
            }
        }
        AssumeDistance(SwerveTrain &refZion, const double& distanceToAssume, VectorDouble* vectorToGoTo) : AutoStep("AssumeDistance") {

            m_zion = &refZion;
            m_targetDistance = distanceToAssume;
            m_direction = vectorToGoTo->toStandard();
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
        }

        bool Execute() {

            //If we're not in tolerance for meeting the goal value (since
            //going to a distance generates no oscillation, zero can be
            //used as a tolerance)...
            double delta = m_targetEncoderPosition - m_zion->m_frontRight->GetDrivePosition();
            if (abs(delta) > R_kuhnsConstant * .1) {

                m_zion->Drive(m_direction->i, m_direction->j, 0, false, false, false);
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
        VectorDouble *m_direction;
};

#endif