#ifndef CAMERADRIVE_H
#define CAMERADRIVE_H

#include "Vision.h"
#include "RobotMap.h"
#include "auto/AutoSequence.h"
#include "auto/steps/AssumeDistance.h"
#include "auto/steps/WaitSeconds.h"

Vision cameraView1;

//Drives Zion until the power cell is at least 50 pixels wide.
//Will drive final distance and run intake once proven to work.

class CameraDrive : public AutoStep {

    public:

        CameraDrive(
            cv::Mat frame,
            SwerveTrain &refZion
        ) : AutoStep("CameraDrive") {

            m_image = frame;
            m_zion = &refZion;
        }

        void Init(){

            //Gets first image and sets distance to drive.
            m_image = cameraView1.imageCapturer();
            distancePerDrive = 12;
        }

        bool Execute(){

            m_image = cameraView1.imageCapturer();
            cv::Vec3i largestVector = cameraView1.largestVectorByImage(m_image);

            AutoSequence loop(true);
            loop.AddStep(new AssumeDistance(*m_zion, distancePerDrive));
            loop.AddStep(new WaitSeconds(0.5));

            //Drives Zion forward until radius of largest powercell is greater than 50 pixels.
            while(largestVector[2] < 50){

                loop.Execute();
                largestVector = cameraView1.largestVectorByImage(m_image);
            }

            return true;
        }
        
    private:
        cv::Mat m_image;
        SwerveTrain* m_zion;
        int distancePerDrive;
};

#endif