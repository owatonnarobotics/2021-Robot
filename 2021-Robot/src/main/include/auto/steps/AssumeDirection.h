#ifndef ASSUMEDIRECTION_H
#define ASSUMEDIRECTION_H

#include <string>

#include "SwerveTrain.h"
#include "RobotMap.h"

class AssumeDirection : public AutoStep {

    public:
        AssumeDirection(SwerveTrain &refZion, const int &directionToMove) : AutoStep("AssumeDirection") {

            m_zion = &refZion;
            m_directionToMove = directionToMove;
        }

        void _Init() {

            m_initialSwervePosition = m_zion->m_frontRight->getSwervePosition();
            m_targetRevRotationsFromCenter = m_zion->getClockwiseREVRotationsFromCenter(
                m_directionToMove == SwerveTrain::ZionDirections::kBackward
                ?
                R_zionVectorBackward : m_directionToMove == SwerveTrain::ZionDirections::kLeft
                ?
                R_zionVectorLeft : R_zionVectorRight
            );
        }

        bool _Execute() {

            switch (m_directionToMove) {

                case SwerveTrain::ZionDirections::kForward: m_zion->assumeNearestZeroPosition(); break;
                case SwerveTrain::ZionDirections::kRight:  m_zion->setZionMotorsToVector(R_zionVectorRight); break;
                case SwerveTrain::ZionDirections::kBackward:  m_zion->setZionMotorsToVector(R_zionVectorBackward); break;
                case SwerveTrain::ZionDirections::kLeft:  m_zion->setZionMotorsToVector(R_zionVectorLeft); break;
            }

            if (m_directionToMove != SwerveTrain::ZionDirections::kForward) {

                if (m_initialSwervePosition + m_targetRevRotationsFromCenter - m_zion->m_frontRight->getSwervePosition() < .25) {

                    m_zion->m_frontRight->setSwerveSpeed();
                    m_zion->m_frontLeft->setSwerveSpeed();
                    m_zion->m_rearLeft->setSwerveSpeed();
                    m_zion->m_rearRight->setSwerveSpeed();
                    return true;
                }
                else {

                    return false;
                }
            }

            if ((m_zion->m_frontRight->getSwervePosition() - m_zion->m_frontRight->getSwerveZeroPosition()) < .1 && (m_zion->m_frontLeft->getSwervePosition() - m_zion->m_frontLeft->getSwerveZeroPosition()) < .1 && (m_zion->m_rearLeft->getSwervePosition() - m_zion->m_rearLeft->getSwerveZeroPosition()) < .1 && (m_zion->m_rearRight->getSwervePosition() - m_zion->m_rearRight->getSwerveZeroPosition()) < .1) {

                m_zion->m_frontRight->setSwerveSpeed();
                m_zion->m_frontLeft->setSwerveSpeed();
                m_zion->m_rearLeft->setSwerveSpeed();
                m_zion->m_rearRight->setSwerveSpeed();
                return true;
            }
            else {

                return false;
            }
        }

        void _Cleanup() {}

    private:
        SwerveTrain* m_zion;
        double m_initialSwervePosition;
        double m_targetRevRotationsFromCenter;
        int m_directionToMove;
};

#endif