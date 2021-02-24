/*
class SwerveModule

Constructors

    SwerveModule(const int&, const int&)
        Creates a swerve module with Spark MAX motor controllers on the
        two supplied CAN IDs, the first controlling drive, the second
        controlling swerve.

Public Methods

    void setDriveSpeed(const double&)
        Sets the driving speed to a double. Defaults to zero.
    void setSwerveSpeed(const double&)
        Sets the swerve speed to a double. Defaults to zero.
        void setSwerveBrake(const bool &)
        If true, sets the swerve to brake mode, if false, to coast mode.
        This is used in SwerveTrain to allow "unlocking" the swerve wheels
        for zeroing by overriding the default brake initialization.
    void setDriveBrake(const bool &)
        If true, sets the drive to brake mode, if false, to coast mode.
        This is used in auto functions to allow stopping on a dime.
    void setSwerveBrake(const bool &)
        If true, sets the swerve to brake mode, if false, to coast mode.
        This is used in SwerveTrain to allow "unlocking" the swerve wheels
        for zeroing by overriding the default brake initialization.
    void setZeroPosition()
        Sets the zero position to the current position.
    double getDrivePosition()
        Returns the total REV revolutions of the drive encoder.
    double getSwervePosition()
        Returns the total REV revolutions of the swerve encoder.
    double getSwervePositionSingleRotation()
        Returns the REV revolution position of the swerve motor as an
        equivalent value inside of one rotation (only from 0 to Nic's
        Constant). For example, a position value equivalent to 1.5 Nic's
        Constants will return a half of Nic's Constant.
    double getSwerveZeroPosition()
        Returns the zero position of the swerve encoder (whatever the value of
        its variable is).
    double getSwerveNearestZeroPosition()
        Uses getSwervePositionSingleRotation() to determine if 0 or one
        Nic's Constant is the most efficient zero for pathfinding.
        See SwerveTrain.h for a more thorough explanation of why
        this works.
    double getDriveSpeed()
        Returns the speed of the drive encoder in RPM.
    double getSwerveSpeed()
        Returns the speed of the swerve encoder in RPM.
    Note that the values returned by the get functions persist across disables, but
        not across power cycles.
    double getStandardDegreeSwervePosition(VectorDouble&, const double&)
        D O C U M E N T  M E
    void assumeSwervePosition(const double& positionToAssume)
        Uses a mathematical function to assign a speed to the swerve motor to
        move quickly and accurately, within a tolerance, to any REV rotation
        value, clockwise or counterclockwise, with an optimal path. See the
        function itself for further detail.
    void assumeSwerveZeroPosition()
        Drives the swerve to the current value of the swerve's zero position
        variable (the last set zero position).
    void assumeSwerveNearestZeroPosition()
        Drives the swerve to its nearest zero position (the closest multiple
        of Nic's Constant to the zero value) either clockwise or
        counterclockwise.

Private Methods

    double calculateAssumePositionSpeed(const double&)
        Uses the following function
             {(1)/(1+e^((-1 * abs(z)) + 5)); z >= R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
        s(z)={R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
             {R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt
            for
                s = speed at which the motor rotates to assume a position
                z = remaining REV revolutions of the position assumption
        to assign a speed with which to proceed towards the final position. It
        was developed, regressed, and tuned to move to the final position as
        fast as possible initially, slowing down as it approaches and becoming
        linear as it settles into tolerance at a high accuracy.
*/

#pragma once

#include <math.h>

#include "rev/CANSparkMax.h"

#include "RobotMap.h"

class SwerveModule {

    public:
        SwerveModule(const int &canDriveID, const int &canSwerveID);

        void setDriveSpeed(const double &speedToSet = 0);
        void setSwerveSpeed(const double &speedToSet = 0);
        void setDriveBrake(const bool &brake);
        void setSwerveBrake(const bool &brake);
        void stop();
        void setZeroPosition();

        double getDrivePosition();
        double getSwervePosition();
        double getSwervePositionSingleRotation();
        double getSwerveZeroPosition();
        double getSwerveNearestZeroPosition();
        double getDriveSpeed();
        double getSwerveSpeed();
        double getStandardDegreeSwervePosition(VectorDouble &vector, const double &angle);

        bool assumeSwervePosition(const double &positionToAssume);
        void assumeSwerveZeroPosition();
        void assumeSwerveNearestZeroPosition();

        bool isAtPositionWithinTolerance(const double &position);

    private:
        double calculateAssumePositionSpeed(const double &howFarRemainingInTravel);

        rev::CANSparkMax *m_driveMotor;
        rev::CANEncoder *m_driveMotorEncoder;
        rev::CANSparkMax *m_swerveMotor;
        rev::CANEncoder *m_swerveMotorEncoder;

        double m_swerveZeroPosition;
};
