/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.

Constructors

    SwerveTrain(SwerveModule&, SwerveModule&, SwerveModule&, SwerveModule&, NavX&)
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions, and takes a NavX
        for use in calculating rotational vectors.

Public Methods

    void setDriveSpeed(const double&)
        Sets a speed to the driving motors on the train. Defaults to zero.
    void setSwerveSpeed(const double&)
        Sets a speed to all swerve motors on the train. Defaults to zero.
    void setDriveBrake(const double&)
        If true, sets the drives to brake mode (as defaultly constructed),
        if false, sets them to coast. Persists across calls. This is used in
        the autonomous to allow the swervetrain to stop very precisely.
    void setSwerveBrake(const bool&)
        Same as above for swerve. This is used to
        "unlock and lock" all of the swerve wheels for easy manual zeroing,
        instead of fighting the wheel brake as defaultly constructed.
    void setZeroPosition(const bool& = false)
        Gets the current encoder values of the swerve motors and stores them
        as privates of the class. These are the values the swerve motors return
        to when invoking assumeSwerveZeroPosition().
        If the passed bool is true, publishes the stored data to the
        SmartDashboard. This is currently used for returning to and maintaining
        "straight".
    void assumeZeroPosition()
        Drives the swerves to return to their zero position.
    void publishSwervePositions()
        Puts the current swerve encoder positions to the SmartDashboard.
    void drive(const double x, const double y, const double z, const bool precision, const bool record)
        Fully drives the swerve train on the supplied x, y, and z values. If precise,
        it scales all values according to a R_ constant and doesn't re-center
        after maneuvering to allow for slow, incredibly precise positioning by
        hand in the full range of the controller.

Private Methods

    void forceControllerXYZToZeroInDeadzone(const int&, const int&, const int&)
        If any of the passed X, Y, or Z values fall outside of their global
        deadzone, they will be set to 0. Otherwise, they are untouched.
    void optimizeControllerXYToZ(const double&, const double&, double &)
        Scales the value of Z with a propotion constant to the magnitude of
        X and Y. Makes rotation harder to incude as speed increases, which
        makes strafing with a controller much more reliable.
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
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule, NavX &navXToSet, Recorder &recorderToSet, Limelight &refLime);

        void setDriveSpeed(const double &driveSpeed = 0);
        void setSwerveSpeed(const double &swerveSpeed = 0);
        void setDriveBrake(const bool &);
        void setSwerveBrake(const bool &);
        void stop();

        void setZeroPosition(const bool &verbose = false);
        bool assumeZeroPosition();
        bool assumeTurnAroundCenterPositions();
        void publishSwervePositions();

        bool setZionMotorsToVector(VectorDouble &);

        void drive(const double, const double, const double, const bool, const bool, const bool);
        void PrintDrivePositions();

    private:
        double calculateLimelightLockSpeed(const double &howFarRemainingInTravelInDegrees);

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
