#ifndef LIMELIGHTLOCK_H
#define LIMELIGHTLOCK_H

#include "auto/AutoStep.h"
#include "SwerveTrain.h"
#include "LimelightLock.h"

class LimelightLock : public AutoStep {

    public:
        LimelightLock(SwerveTrain &refZion, Limelight &refLime) : AutoStep("LimelightLock") {

            m_zion = &refZion;
            m_limelight = &refLime;
        }

        void Init() {}

        bool Execute() {

            m_zion->Drive(0, 0, 0, false, false, true);
            return m_limelight->isWithinHorizontalTolerance();
        }

    private:
        SwerveTrain* m_zion;
        Limelight* m_limelight;
};

#endif