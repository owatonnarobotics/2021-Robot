#ifndef ASSUMEDIRECTION_H
#define ASSUMEDIRECTION_H

#include <string>

#include "VectorDouble.h"
#include "SwerveTrain.h"
#include "RobotMap.h"

class AssumeDirection : public AutoStep {

    public:
        AssumeDirection(SwerveTrain &refZion, const int &directionToMove) : AutoStep("AssumeDirection") {

            m_zion = &refZion;
            m_directionToMove = directionToMove;
        }

        void _Init() {

            switch (m_directionToMove) {

                case SwerveTrain::ZionDirections::kForward: m_targetVector = &R_zionVectorForward; break;
                case SwerveTrain::ZionDirections::kRight: m_targetVector = &R_zionVectorRight; break;
                case SwerveTrain::ZionDirections::kBackward: m_targetVector = &R_zionVectorBackward; break;
                case SwerveTrain::ZionDirections::kLeft: m_targetVector = &R_zionVectorLeft; break;
            }
        }

        bool _Execute() {

            m_zion->setZionMotorsToVector(*m_targetVector);
            return m_zion->zionMotorsAreAtVector(*m_targetVector);
        }

        void _Cleanup() {}

    private:
        SwerveTrain* m_zion;
        int m_directionToMove;
        const VectorDouble* m_targetVector;
};

#endif