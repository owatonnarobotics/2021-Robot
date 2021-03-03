/*
class Limelight

Constructors

    Limelight
        Creates a limelight on the default NetworkTable interface.

Public Methods

    double getHorizontalOffset()
        Returns the horizontal offset of the target (tx).
    double getVerticalOffset()
        Returns the vertical offset of the target (ty).
    double getTargetArea()
        Returns the area of the target in-sight.
    bool getTarget()
        Returns true if there is a target in-sight, false otherwise.
    All return 0 in event of a null target.
    void setProcessing(const bool& = true)
        Turns on or off the vision processing for using the Limelight
        as a camera. Defaults to on.
    void setLime(const bool& = true)
        Turns the Limelight LEDs on or off. Defaults to on.
*/

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Limelight {

    public:
        Limelight() {

            table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-lnchr");
        }

        double getHorizontalOffset() {

            return table->GetNumber("tx",0.0);
        }
        double getVerticalOffset() {

            return table->GetNumber("ty", 0);
        }
        double getTargetArea() {

            return table->GetNumber("ta", 0);
        }
        bool getTarget() {

            return table->GetNumber("tv", 0);
        }
        bool isWithinHorizontalTolerance() {

            return abs(getHorizontalOffset()) < R_zionAutoToleranceHorizontalOffset;
        }

        void setProcessing(const bool &toSet = true) {

            //According to doc, 1 is off, 0 is on
            table->PutNumber("camMode", toSet ? 0 : 1);
        }
        void setLime(const bool &toSet = true) {

            //According to doc, 3 is on, 1 is off, and 2 is blink.
            table->PutNumber("ledMode", toSet ? 3 : 1);
        }

        //Almost exactly the same function as
        //SwerveModule::calculateAssumePositionSpeed, except with constants for
        //limelight lock
        double CalculateLimelightLockSpeed() {

            //This if block is for driving in limelight lock mode.  This means that no
            //matter which way we are driving, we will always be pointed at the goal.
            //Turn on the limelight so that we can check if a target is found.
            setLime();
            setProcessing();

            //Check if we are looking at a valid target...
            if (getTarget()) {

                double howFarRemainingInTravelInDegrees = getHorizontalOffset();
                //Update our rotational speed so that we turn towards the goal.
                //Begin initally with a double calculated with the simplex function with a horizontal stretch of factor two...
                double toReturn = ((1) / (1 + exp((-1 * (0.5 * abs(0.5 * howFarRemainingInTravelInDegrees))) + 5)));
                //If we satisfy conditions for the first linear piecewise, take that speed instead...
                if (abs(howFarRemainingInTravelInDegrees) < R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorAt) {

                    toReturn = R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorSpeed;
                }
                //Do the same for the second...
                if (abs(howFarRemainingInTravelInDegrees) < R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorAt) {

                    toReturn = R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorSpeed;
                }
                //And if we needed to travel negatively to get where we need to be, make the final speed negative...
                if (abs(howFarRemainingInTravelInDegrees) < R_zionAutoToleranceHorizontalOffset) {

                    toReturn = 0;
                }
                if (howFarRemainingInTravelInDegrees < 0) {

                    toReturn = -toReturn;
                }
                return toReturn;
            }
            else {

                return 1.0;
            }
        }

    private:
        std::shared_ptr<NetworkTable> table;
};
