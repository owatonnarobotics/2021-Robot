/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.
*/

#pragma once

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include "NavX.h"
#include "SwerveModule.h"
#include "VectorDouble.h"
#include "auto/Recorder.h"
#include "Limelight.h"

class SwerveTrain {

    public:
        /**
         * Constructor for a SwerveTrain.
         * 
         * Creates a swerve train with the swerve modules on the front right,
         * front left, back left, and back right positions, and takes a NavX
         * for use in calculating rotational vectors.
         * 
         * @param frontRightModule A reference to the front right SwerveModule
         * @param frontLeftModule A reference to the front left SwerveModule
         * @param rearLeftModule A reference to the rear left SwerveModule
         * @param rearRightModule A reference to the rear right SwerveModule
         * @param navXToSet A reference to a NavX
         * @param recorderToSet A reference to a Recorder
         * @param refLime A reference to a Limelight
         */
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule, NavX &navXToSet, Recorder &recorderToSet, Limelight &refLime);

        /**
         * Sets a speed to the driving motors on the train. Defaults to zero.
         * 
         * @param driveSpeed The speed to set
         */
        void SetDriveSpeed(const double &driveSpeed = 0);

        /**
         * Sets a speed to all swerve motors on the train. Defaults to zero.
         * 
         * @param swerveSpeed The speed to set
         */
        void SetSwerveSpeed(const double &swerveSpeed = 0);

        /**
         * 
         * Sets the drive motor's brake mode.
         * 
         * If true, sets the drives to brake mode (as defaultly constructed),
         * if false, sets them to coast. Persists across calls. This is used in
         * the autonomous to allow the swervetrain to Stop very precisely.
         * 
         * @param brake Whether or not to set the brake
         */
        void SetDriveBrake(const bool &brake);

        /**
         * Sets the swerve motor's brake mode.
         * 
         * Same as above for swerve. This is used to
         * "unlock and lock" all of the swerve wheels for easy manual zeroing,
         * instead of fighting the wheel brake as defaultly constructed.
         * 
         * @param brake Whether or not to set the brake
         */
        void SetSwerveBrake(const bool &brake);

        /**
         * Stops all swerve at drive motors.
         * 
         * Sets all swerve motors and drive motors to 0.
         */
        void Stop();

        /**
         * Sets the zero position of all swerve modules.
         * 
         * Gets the current encoder values of the swerve motors and stores them
         * as privates of the class. These are the values the swerve motors return
         * to when invoking AssumeSwerveZeroPosition().
         * If the passed bool is true, publishes the stored data to the
         * SmartDashboard. This is currently used for returning to and maintaining
         * "straight".
         * 
         * @param verbose Whether or not to print the positions
         */
        void SetZeroPosition(const bool &verbose = false);

        /**
         * Drives the swerves to return to their zero position.
         * 
         * @return Whether or not the serves are at their zero positions
         */
        bool AssumeZeroPosition();

        /**
         * Drives the swerves to thier turn around center positions.
         * 
         * The swerve turn around center positions are at 45 degree angles in a
         * way that allows the robot to turn around the center as efficiently
         * as possible.
         * 
         * @return Whether or not the swerves are at their turn around center positions
         */
        bool AssumeTurnAroundCenterPositions();

        /**
         * Puts the current swerve encoder positions to the SmartDashboard.
         */
        void PrintSwervePositions();

        /**
         * Sets the swerve modules to a vector.
         * 
         * The vector is absolute, or in other words in relation to the field.
         *
         * @param vectorToSet The vector to set the swerve modules to
         * @return Whether or not the swerve modules are at the vector
         */
        bool SetZionMotorsToVector(VectorDouble &vectorToSet);

        /**
         * Drives the swere modules field oriented.
         * 
         * Fully drives the swerve train on the supplied x, y, and z values. If precise,
         * it scales all values according to a R_ constant and doesn't re-center
         * after maneuvering to allow for slow, incredibly precise positioning by
         * hand in the full range of the controller.
         * 
         * @param rawX The x value to drive the SwerveTrain with
         * @param rawY The y value to drive the SwerveTrain with
         * @param rawZ The z value to drive the SwerveTrain with
         * @param precise Whether or not to drive with increased precision
         * @param record Whether or not to record the drive values
         * @param limelightLock Whether or not to drive while locked on to the Limelight target
         */
        void Drive(const double rawX, const double rawY, const double rawZ, const bool precise, const bool record, const bool limelightLock);

        /**
         * Prints the positions of the drive motors to the SmartDashboard.
         */
        void PrintDrivePositions();

    private:
        double CalculateLimelightLockSpeed(const double &howFarRemainingInTravelInDegrees);

    //Allow the peices of the SwerveTrain to be public for convenient
    //low-level access when needed. SwerveTrain is a great container.
    //This is primarily used for Hal, the auto driver, so he can set low-level
    //module commands through one passed SwerveTrain.
    public:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;
        NavX *navX;
        Recorder* m_recorder;
        Limelight* m_limelight;

        enum ZionDirections {

            kForward, kRight, kBackward, kLeft
        };
};
