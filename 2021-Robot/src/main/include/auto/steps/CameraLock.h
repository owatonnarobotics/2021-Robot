#ifndef CAMERALOCK_H
#define CAMERALOCK_H

#include "Vision.h"
#include "RobotMap.h"
#include "auto/steps/AssumeRotationDegrees.h"
#include "auto/steps/WaitSeconds.h"
#include <opencv/cv.hpp>

Vision cameraView;

//Rotates Zion to attempt to find powercell.
//Will need more documentation in future changes.

class CameraLock : public AutoStep {

    public:

        CameraLock(cv::Mat frame, 
            SwerveTrain &refZion,
            Limelight &refLimelight,
            NavX &refNavX
        ) : AutoStep("CameraLock") {

            m_image = frame;
            m_zion = &refZion;
            m_limelight = &refLimelight;
            m_navX = &refNavX;
        }

        void Init(){

            m_image = cameraView.imageCapturer();
            degreesPerTurn = 30;
        }

        bool Execute() {

            cv::Vec3i largestVector = cameraView.largestVectorByImage(m_image);
            int turnsMade = 0;
            
            while(largestVector[0] < 5 || turnsMade >= (360 / degreesPerTurn)) {

                turnsMade++;
                AssumeRotationDegrees(*m_zion, *m_limelight, *m_navX, degreesPerTurn);
                WaitSeconds(0.75);

                m_image = cameraView.imageCapturer();
                largestVector = cameraView.largestVectorByImage(m_image);
            }
            
            m_image = cameraView.imageCapturer();
            double degreesToTurnRound = cameraView.degreesToTurn(m_image);

            AssumeRotationDegrees(*m_zion, *m_limelight, *m_navX, degreesToTurnRound);

            return false;
        }
    
    private:
        cv::Mat m_image;
        SwerveTrain* m_zion;
        Limelight* m_limelight;
        NavX* m_navX;
        double degreesPerTurn;
        
};

#endif