/*
class Vision
Constructors
    
Public

    double estPixelsPerDegree
        Estimated number of pixels per degree in field of view. Used later on
        to determine how many degrees robot will need to rotate.
    cv::Vec3i largestVectorByRadius(std::vector<cv::Vec3i>, int)
        Outputs circle vector with largest radius from given vector. Used in
        largestVectorFromImage to simplify code.
    cv::Mat imageProcessor(cv::Mat)
        Takes in image and processes it to needed levels for Hough transform.
        Applies colorspace change, threshold and noise reduction to get a usable
        image for functions that need it.
    cv::Vec3i largestVectorFromImage(cv::Mat)
        Returns the circle vector with the largest radius from the given image.
        Uses imageProcessor on image before Hough Circle Transform.
    double degreesToTurn(cv::Mat)
        Estimates degrees robot will need to turn to line up with power cell.
        Used in auto to determine how far it has to turn to line up from just the image.
    cv::Mat imageCapturer()
        Takes in most recent image from USB camera and converts it to the Mat type, 
        needed for image processing using openCV.
    bool hasTarget(cv::Mat)
        Checks if the camera has imaged and is tracking a power cell currently.
    bool withinCameraRotationTolerance(cv::Mat)
        Checks if the power cell x value is within a tolerance range set.
    bool withinCameraDriveTolerance(cv::Mat)
        Tests if the powercell is close enough that Zion should stop and do nothing.
    double cameraRotationSpeedNeeded(cv::Mat)
        Determines the speed that the robot should be turning at, based on the position of the ball.
    double cameraDriveSpeedNeeded
    cv::Mat optionalVisionOutput(cv::Mat)
        Outlines circles, places radius and x value, then highlights text of largest radius.
        Currently unused, mainly was for testing phase.
*/

#ifndef VISION_H
#define VISION_H

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

class Vision {

    public:

        Vision(){
        };
        //Needs to be cleaned up and documented better in some spots.

        //Estimate for pixels per degree of view. Will need to adjust based on USB camera FOV.
        double estPixelsPerDegree = 640 / 55; // ~ 11.636363 pixels per degree


        //Returns circle vector with largest radius.
        cv::Vec3i largestVectorByRadius(std::vector<cv::Vec3f> input, double size){
            
            double largestValue = 0;
            cv::Vec3i largestVector;
            for(size_t i = 0; i < size; i++){

                cv::Vec3i c = input[i];
                if(c[2] > largestValue){

                    largestValue = c[2];
                    largestVector = c;
                }
            };
            return largestVector;
        }


        cv::Mat imageProcessor(cv::Mat inputFrame){

            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            int low_H = 20, low_S = 100, low_V = 100;       //Minimum color value
            int high_H = 40, high_S = 255, high_V = 255;    //Maximum color value

            //Changes image to the HSV colorspace then filters video using HSV values given above
            cv::cvtColor(inputFrame, inputFrame, cv::COLOR_BGR2HSV);

            //Thresholds the image, (optimally) showing pixels of only powercells.
            cv::inRange(inputFrame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V),inputFrame); 

            //Removes noise (small circles in the background)
            cv::morphologyEx(inputFrame, inputFrame, cv::MORPH_OPEN, element); 

            //Applies a blur to help reduce false circle detection
            cv::medianBlur(inputFrame, inputFrame, 5);

            return inputFrame;
        }


        cv::Vec3i largestVectorByImage(cv::Mat inputFrame){

            inputFrame = imageProcessor(inputFrame);

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(inputFrame, circles, cv::HOUGH_GRADIENT, 1,
                             inputFrame.rows/10, //This value determines how far apart circles have to be to get recognized.
                             35, 10, 1, 400      //change the last two parameters (which are min./max. radius respectively)
            );

            return largestVectorByRadius(circles, circles.size());
        }


        double degreesToTurn(cv::Mat inputFrame){

            cv::Vec3i largest = largestVectorByImage(inputFrame);

            int pixelsFromCenter = largest[0] - 320;

            return ((pixelsFromCenter / estPixelsPerDegree) * -1);
        }


        cv::Mat imageCapturer(){

            cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

            cv::Mat frame;           //Initializing the image being manipulated by OpenCV 
            cvSink.GrabFrame(frame); //Converts image from camera to Mat format (for manipulation)

            return frame;
        }


        bool hasTarget(cv::Mat frame){

            cv::Vec3i largestVector = largestVectorByImage(frame);
            
            //If the power cell is larger than our noise limit, output true.
            if (largestVector[2] > R_cameraLowerPowerCellNoiseLimit) {

                return true;
            }
            else {

                return false;
            }
        }

        

        bool withinCameraRotationTolerance(cv::Mat frame){

            cv::Vec3i largestVector = largestVectorByImage(frame);

            //If the power cell is within our tolerance and larger than the noise limit,
            //output that the ball is within tolerance.
            if (abs(largestVector[0]) < R_cameraRotationToleranceInPixels && hasTarget(frame)){
                
                return true;
            }
            else {

                return false;
            }
        }

        bool withinCameraDriveTolerance(cv::Mat frame) {

            cv::Vec3i largestVector = largestVectorByImage(frame);

            if (abs(largestVector[2]) > R_cameraDriveCloseEnoughLimit && hasTarget(frame)){

                return true;
            }
            else {

                return false;
            }
        }


        double cameraRotationSpeedNeeded(cv::Mat frame){

            cv::Vec3i largestVector = largestVectorByImage(frame);

            //First checks if the camera has a power cell in view...
            if (hasTarget(frame)) {

                //then determines which way and how fast, if so. If within tolerance, set to 0.
                //Quadratic hits the auto execution cap at 320 pixels.
                if (largestVector[0] > R_cameraRotationToleranceInPixels){

                    return -(pow(largestVector[0], 2) / 102400) * R_autoSearchTurningSpeedExecutionCap;
                }
                else if (largestVector[0] < -R_cameraRotationToleranceInPixels){

                    return (pow(largestVector[0], 2) / 102400) * R_autoSearchTurningSpeedExecutionCap;
                }
                else {

                    return 0.0;
                }
            }
            //If the camera does not have a target, presumes that it needs to turn to find it.
            else {

                return R_autoSearchTurningSpeedExecutionCap;
            }
        }


        double cameraDriveSpeedNeeded(cv::Mat frame){

            cv::Vec3i largestVector = largestVectorByImage(frame);

            if (hasTarget(frame)){

                if (largestVector[2] > R_cameraDriveCloseEnoughLimit) {

                    return R_autoSearchDriveSpeedExecutionCap;
                }
                else {

                    return 0;
                }
            }

            return 0;
        }


        //May be removed in future version if determined to not be needed.
        cv::Mat optionalVisualOutput(cv::Mat frame){
            
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            /*cv::VideoCapture cap;
            // open the default camera, use something different from 0 otherwise;
            // Check VideoCapture documentation.
            if(!cap.open(0)){
                return frame;
            }*/
        
            //Processes and thresholds image.
            frame = imageProcessor(frame);

            // Creates a vector named circles, then applies the transform that looks for circles in an image.
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 1,
                            frame.rows/10,     //This value determines how far apart circles have to be to get recognized.
                            35, 10, 1, 400     // change the last two parameters (which are min./max. radius respectively)
            );

            //Outputs the number of found circles to SmartDashboard.
            frc::SmartDashboard::PutNumber("Number of Circles: ", circles.size());

            // Draws circles and center points from vector(s).
            for( size_t i = 0; i < circles.size(); i++ ){

                cv::Vec3i c = circles[i];
                cv::Point center = cv::Point(c[0], c[1]); // circle center
                cv::circle( frame, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA );   // circle outline
                int radius = c[2];
                cv::circle( frame, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA );

                //Adds radius and position data to image to demonstrate.
                cv::String radiusOutput = std::to_string(radius) + " " + std::to_string(c[0] - 320);
                cv::putText(frame, radiusOutput, center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(50, 255, 0), 2, cv::LINE_AA);
            }

            cv::Vec3i largestVector = largestVectorByRadius(circles, circles.size());

            //Converts largest radius to string then adds to image.
            cv::String maxCircleString = std::to_string(largestVector[2]);
            cv::putText(frame, maxCircleString, cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX,
                        1, cv::Scalar(150, 255, 0), 2, cv::LINE_AA);
            
            //Changes doubles from vector to ints (for appearance)
            int largestX = largestVector[0];
            int largestY = largestVector[1];
            int largestRadiusFromVector = largestVector[2];

            //Converts relevant parts of vector with largest radius to string then overlays text lighter.
            cv::String largestCircle = std::to_string(largestRadiusFromVector) + " " + std::to_string(largestX - 320);
            cv::putText(frame, largestCircle, cv::Point(largestX, largestY), 
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(175, 255, 0), 2, cv::LINE_AA);

            return frame;
        };
};
#endif