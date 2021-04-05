#ifndef CAMERALOCK2_H
#define CAMERALOCK2_H

#include "Vision.h"
#include "RobotMap.h"
#include "auto/AutoSequence.h"
#include "auto/steps/AssumeRotationDegrees.h"
#include "auto/steps/WaitSeconds.h"

Vision cameraView2;

//Rotates Zion to attempt to find powercell.
//Will need more documentation in future revisions.

class CameraLock2 : public AutoStep {

    public:

        CameraLock2(
            cv::Mat frame, 
            SwerveTrain &refZion,
            Limelight &refLimelight,
            NavX &refNavX
        ) : AutoStep("CameraLock2") {

            m_image = frame;
            m_zion = &refZion;
            m_limelight = &refLimelight;
            m_navX = &refNavX;
        }

        void Init(){
            
            //Gets the first image and how many degrees per individual rotation.
            m_image = cameraView2.imageCapturer();
            //degreesPerTurn = 30;
        }

        bool Execute() {
            
            m_image = cameraView2.imageCapturer();

            m_zion->Drive(0, 0, cameraView2.cameraSpeedNeeded(m_image), false);
            
            return cameraView2.withinCameraTolerance(m_image);
        }
    
    private:
        cv::Mat m_image;
        SwerveTrain* m_zion;
        Limelight* m_limelight;
        NavX* m_navX;
        double degreesPerTurn;   
};

#endif