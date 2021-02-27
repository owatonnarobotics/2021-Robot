#ifndef AssumeDirectionAbsolute_H
#define AssumeDirectionAbsolute_H

#include <string>

#include "VectorDouble.h"
#include "SwerveTrain.h"
#include "RobotMap.h"

class AssumeDirectionAbsolute : public AutoStep {

    public:
        AssumeDirectionAbsolute(SwerveTrain &refZion, const int &directionToMove) : AutoStep("AssumeDirectionAbsolute") {

            m_zion = &refZion;
            switch (directionToMove) {

                case SwerveTrain::ZionDirections::kForward: m_targetVector = new VectorDouble(0, 1); break;
                case SwerveTrain::ZionDirections::kRight: m_targetVector = new VectorDouble(1, 0); break;
                case SwerveTrain::ZionDirections::kBackward: m_targetVector = new VectorDouble(0, -1); break;
                case SwerveTrain::ZionDirections::kLeft: m_targetVector = new VectorDouble(-1, 0); break;
            }
        }

        AssumeDirectionAbsolute(SwerveTrain &refZion, VectorDouble* vectorToGoTo) : AutoStep("AssumeDirectionAbsolute") {

            m_zion = &refZion;
            m_targetVector = vectorToGoTo;
        }

        void Init() {}

        bool Execute() {

            return m_zion->setZionMotorsToVector(*m_targetVector);
        }

    private:
        SwerveTrain* m_zion;
        VectorDouble* m_targetVector;
};

#endif