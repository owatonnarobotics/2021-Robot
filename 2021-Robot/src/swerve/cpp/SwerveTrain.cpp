#include <math.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"
#include "Launcher.h"
#include "Limelight.h"
#include "Controller.h"

SwerveTrain::SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule, NavX &navXToSet, Recorder &recorderToSet, Limelight &refLime) {

    m_frontRight = &frontRightModule;
    m_frontLeft = &frontLeftModule;
    m_rearLeft = &rearLeftModule;
    m_rearRight = &rearRightModule;
    navX = &navXToSet;
    m_recorder = &recorderToSet;
    m_limelight = &refLime;
}

void SwerveTrain::SetDriveSpeed(const double &driveSpeed) {

    m_frontRight->SetDriveSpeed(driveSpeed);
    m_frontLeft->SetDriveSpeed(driveSpeed);
    m_rearLeft->SetDriveSpeed(driveSpeed);
    m_rearRight->SetDriveSpeed(driveSpeed);
}

void SwerveTrain::SetSwerveSpeed(const double &swerveSpeed) {

    m_frontRight->SetSwerveSpeed(swerveSpeed);
    m_frontLeft->SetSwerveSpeed(swerveSpeed);
    m_rearLeft->SetSwerveSpeed(swerveSpeed);
    m_rearRight->SetSwerveSpeed(swerveSpeed);
}

void SwerveTrain::SetDriveBrake(const bool &brake) {

    m_frontRight->SetDriveBrake(brake);
    m_frontLeft->SetDriveBrake(brake);
    m_rearLeft->SetDriveBrake(brake);
    m_rearRight->SetDriveBrake(brake);
}

void SwerveTrain::SetSwerveBrake(const bool &brake) {

    m_frontRight->SetSwerveBrake(brake);
    m_frontLeft->SetSwerveBrake(brake);
    m_rearLeft->SetSwerveBrake(brake);
    m_rearRight->SetSwerveBrake(brake);
}

void SwerveTrain::Stop() {

    SetDriveSpeed();
    SetSwerveSpeed();
}

void SwerveTrain::SetZeroPosition(const bool &verbose) {

    m_frontRight->SetZeroPosition();
    m_frontLeft->SetZeroPosition();
    m_rearLeft->SetZeroPosition();
    m_rearRight->SetZeroPosition();

    if (verbose) {

        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFR", m_frontRight->GetSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFL", m_frontLeft->GetSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRL", m_rearLeft->GetSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRR", m_rearRight->GetSwerveZeroPosition());
    }
}

void SwerveTrain::PrintDrivePositions() {

    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFR", m_frontRight->GetDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFL", m_frontLeft->GetDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRL", m_rearLeft->GetDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRR", m_rearRight->GetDrivePosition());
}

bool SwerveTrain::AssumeZeroPosition() {

    return m_frontRight->AssumeSwerveZeroPosition();
            m_frontLeft->AssumeSwerveZeroPosition();
            m_rearLeft->AssumeSwerveZeroPosition();
            m_rearRight->AssumeSwerveZeroPosition();
}

bool SwerveTrain::AssumeTurnAroundCenterPositions() {

    return  m_frontRight->AssumeSwervePosition((1.0 / 8.0) * R_nicsConstant) &&
            m_frontLeft->AssumeSwervePosition((3.0 / 8.0) * R_nicsConstant) &&
            m_rearLeft->AssumeSwervePosition((5.0 / 8.0) * R_nicsConstant) &&
            m_rearRight->AssumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
}

bool SwerveTrain::SetZionMotorsToVector(VectorDouble &vectorToSet) {

    double angle = navX->getYawFull();
    return m_frontRight->AssumeSwervePosition(m_frontRight->AbsoluteVectorToNics(vectorToSet, angle)) &&
           m_frontLeft->AssumeSwervePosition(m_frontLeft->AbsoluteVectorToNics(vectorToSet, angle)) &&
           m_rearLeft->AssumeSwervePosition(m_rearLeft->AbsoluteVectorToNics(vectorToSet, angle)) &&
           m_rearRight->AssumeSwervePosition(m_rearRight->AbsoluteVectorToNics(vectorToSet, angle));
}

void SwerveTrain::PrintSwervePositions() {

    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFR", m_frontRight->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFL", m_frontLeft->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRL", m_rearLeft->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRR", m_rearRight->GetSwervePosition());
}

void SwerveTrain::Drive(const double rawX, const double rawY, const double rawZ, const bool precision, const bool record, const bool limelightLock) {

    double x = -rawX;
    double y = -rawY;
    double z = rawZ;

    frc::SmartDashboard::PutNumber("X", x);
    frc::SmartDashboard::PutNumber("Y", y);
    frc::SmartDashboard::PutNumber("Z", z);

    if (record) {

        m_recorder->Record(rawX, rawY, rawZ, precision, limelightLock);
    }
    else {

        m_recorder->Publish();
    }

    //To prevent controller drift, if the values of X, Y, and Z are inside of
    //deadzone, set them to 0.
    Controller::forceControllerXYZToZeroInDeadzone(x, y, z);

    //To prevent accidental turning, optimize Z to X and Y's magnitude.
    Controller::optimizeControllerXYToZ(x, y, z);

    //If the controller is in the total deadzone (entirely still)...
    if (Controller::getControllerInDeadzone(x, y, z)) {

        /*
        Go to the nearest zero position, take it as the new zero, and
        reset the angle coming off of the gyroscope to zero to allow
        setting of the next translation vector, as the translation
        vector is always refereced from "center" (the beginning of the
        turn) and updated based on how much that gyro angle changes.
        Due to this, it must be reset when not in movement to allow
        this behavior to occur...
        */
        Stop();
    }
    //Otherwise, go to the result vectors and use the magnitude to set the
    //speed of driving, and set each wheel's swerve position based on its
    //respective resulting vector.
    else {

        //This if block is for driving in limelight lock mode.  This means that no
        //matter which way we are driving, we will always be pointed at the goal.
        if (limelightLock) {

            //Turn on the limelight so that we can check if a target is found.
            m_limelight->setLime();
            m_limelight->setProcessing();

            //Check if we are looking at a valid target...
            if (m_limelight->getTarget()) {

                //Update our rotational speed so that we turn towards the goal.
                z = CalculateLimelightLockSpeed(m_limelight->getHorizontalOffset());
            }
        }
        else {

            //If we want to drive normally, turn off the limelight
            //(because it is blinding).
            m_limelight->setLime(false);
            m_limelight->setProcessing(false);
        }
        
        /*
        The translation vector is the "standard" vector - that is, if no
        rotation were applied, the robot would simply travel in the direction
        of this vector. In order to obtain this, we need the X and Y from the
        controller in addition to input from a gyroscope. This is due to the
        fact that pushing straight on the controller should always make it
        drive directly away from the operator, and simply driving "straight" at
        a 45* angle would make it drive away from the operator at 45*. This is
        true of any angle, so the gyro is needed to offset the vector described
        by X and Y. VectorDouble translationVector(0, 0);
        */
        //TODO: why inverted?
        VectorDouble translationVector(-x, y);

        //Get the navX yaw angle for computing the field-oriented angle
        double angle = navX->getYawFull();

        /*
        The rotational vectors are found by multiplying the controller's
        rotational axis [-1, 1] by the cosine of the wheel's RELATIVE yaw (the
        position we put the wheels in so that it can turn, with zero at the
        top) minus the number of degrees we are offset from 0.  Then, for the j
        value we do the same, except we use sine, for Y.  All angles passed as
        paramaters to cos() and sin() are converted to radians first.
        */
        VectorDouble frontRightRotationVector (

            z * cos((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180))
        );

        VectorDouble frontLeftRotationVector (

            z * cos((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearLeftRotationVector (

            z * cos((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearRightRotationVector (

            z * cos((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180))
        );

        /*
        And the vector we actually want to apply to the swerves is the sum of
        the two vectors - the vector that forms "straight" (translationVector)
        and the vector that forms strictly the rotation (rotationVector).
        */
        VectorDouble frontRightResultVector = translationVector + frontRightRotationVector;
        VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
        VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
        VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

        /*
        Here, all of the resulting vectors are converted into Nics so that they
        can be written to the swerve modules using AssumeSwervePosition(). They
        were in degrees before to enable the use of common trigonomoetry. We
        get Nics from degrees by calling getSwerveRotatingPosition().
        */
        m_frontRight->AssumeSwervePosition(m_frontRight->AbsoluteVectorToNics(frontRightResultVector, angle));
        m_frontLeft->AssumeSwervePosition(m_frontLeft->AbsoluteVectorToNics(frontLeftResultVector, angle));
        m_rearLeft->AssumeSwervePosition(m_rearLeft->AbsoluteVectorToNics(rearLeftResultVector, angle));
        m_rearRight->AssumeSwervePosition(m_rearRight->AbsoluteVectorToNics(rearRightResultVector, angle));

        const double executionCap = precision ? R_executionCapZionPrecision : R_executionCapZion;
        m_frontRight->SetDriveSpeed(frontRightResultVector.magnitude() * executionCap);
        m_frontLeft->SetDriveSpeed(frontLeftResultVector.magnitude() * executionCap);
        m_rearLeft->SetDriveSpeed(rearLeftResultVector.magnitude() * executionCap);
        m_rearRight->SetDriveSpeed(rearRightResultVector.magnitude() * executionCap);
    }
}

//Almost exactly the same function as
//SwerveModule::calculateAssumePositionSpeed, except with constants for
//limelight lock
double SwerveTrain::CalculateLimelightLockSpeed(const double &howFarRemainingInTravelInDegrees) {

    //Begin initally with a double calculated with the simplex function with a horizontal stretch of factor two...
    double toReturn = ((1) / (1 + exp((-1 * (0.5 * abs(howFarRemainingInTravelInDegrees))) + 5)));
    //If we satisfy conditions for the first linear piecewise, take that speed instead...
    if (abs(howFarRemainingInTravelInDegrees) < R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorAt) {

        toReturn = R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorSpeed;
    }
    //Do the same for the second...
    if (abs(howFarRemainingInTravelInDegrees) < R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorAt) {

        toReturn = R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorSpeed;
    }
    //And if we needed to travel negatively to get where we need to be, make the final speed negative...
    if (howFarRemainingInTravelInDegrees < 0) {

        toReturn = -toReturn;
    }
    return toReturn;
}