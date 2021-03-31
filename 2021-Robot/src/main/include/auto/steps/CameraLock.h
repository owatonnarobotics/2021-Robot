#ifndef CAMERALOCK_H
#define CAMERALOCK_H

#include "Vision.h"
#include "RobotMap.h"
#include "auto/AutoSequence.h"
#include "auto/steps/AssumeRotationDegrees.h"
#include "auto/steps/WaitSeconds.h"

Vision cameraView;

//Rotates Zion to attempt to find powercell.
//Will need more documentation in future revisions.

class CameraLock : public AutoStep {

    public:

        CameraLock(
            cv::Mat frame, 
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
            
            //Gets the first image and how many degrees per individual rotation.
            m_image = cameraView.imageCapturer();
            degreesPerTurn = 30;
        }

        bool Execute() {

            cv::Vec3i largestVector = cameraView.largestVectorByImage(m_image);
            int turnsMade = 0;
            
            AutoSequence loop(true);
            loop.AddStep(new AssumeRotationDegrees(*m_zion, *m_limelight, *m_navX, degreesPerTurn));
            loop.AddStep(new WaitSeconds(0.75));
            loop.Init();

            //Turns Zion in 30 degree increments (set above) until it sees a circle 
            //larger than 4 pixels in radius (limits false positives) or has reached a
            //full circle of rotation. Prevents non-stop turning, if no cells are seen.
            while(largestVector[2] < 4 && turnsMade < (360 / degreesPerTurn)) {

                loop.Execute();
                turnsMade++;

                m_image = cameraView.imageCapturer();
                largestVector = cameraView.largestVectorByImage(m_image);
            }
            
            //Once power cell is in view, final rotation to line up using x coordinate of it.
            m_image = cameraView.imageCapturer();
            double degreesToTurnRound = cameraView.degreesToTurn(m_image);

            AutoSequence lastStep(false);
            lastStep.AddStep(new AssumeRotationDegrees(*m_zion, *m_limelight, *m_navX, degreesToTurnRound));
            lastStep.Init();
            while (!lastStep.Execute());

            return true;
        }
    
    private:
        cv::Mat m_image;
        SwerveTrain* m_zion;
        Limelight* m_limelight;
        NavX* m_navX;
        double degreesPerTurn;   
};

#endif