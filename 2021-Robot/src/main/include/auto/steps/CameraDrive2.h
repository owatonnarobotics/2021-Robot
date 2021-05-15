#ifndef CAMERADRIVE2_H
#define CAMERADRIVE2_H

#include "Vision.h"
#include "RobotMap.h"
#include "auto/AutoSequence.h"
#include "auto/steps/AssumeDistance.h"
#include "auto/steps/WaitSeconds.h"

Vision cameraDrive1;

//Drives Zion until the power cell is at least a certain number of pixels wide.
//Will drive final distance and run intake once proven to work.

class CameraDrive2 : public AutoStep {

    public:

        CameraDrive2(
            cv::Mat frame,
            SwerveTrain &refZion
        ) : AutoStep("CameraDrive2") {

            m_image = frame;
            m_zion = &refZion;
        }

        void Init(){

            //Gets first image.
            m_image = cameraDrive1.imageCapturer();
        }

        bool Execute(){

            m_image = cameraDrive1.imageCapturer();

            //Decides on drive speed based on distance. Currently only has two speeds,
            //but more can be introduced.
            m_zion->Drive(cameraDrive1.cameraDriveSpeedNeeded(m_image), 0, 0, false);

            if(cameraDrive1.withinCameraDriveTolerance(m_image)){
                m_zion->Drive(0, 0, 0, false);
            }

            return cameraDrive1.withinCameraDriveTolerance(m_image);
        }
        
    private:
        cv::Mat m_image;
        SwerveTrain* m_zion;
};

#endif