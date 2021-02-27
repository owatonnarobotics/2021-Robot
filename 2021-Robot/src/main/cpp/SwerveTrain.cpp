#include <math.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"
#include "Launcher.h"
#include "Limelight.h"

SwerveTrain::SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule, NavX &navXToSet, Recorder &recorderToSet, Limelight &refLime) {

    m_frontRight = &frontRightModule;
    m_frontLeft = &frontLeftModule;
    m_rearLeft = &rearLeftModule;
    m_rearRight = &rearRightModule;
    navX = &navXToSet;
    m_recorder = &recorderToSet;
    m_limelight = &refLime;
}

void SwerveTrain::setDriveSpeed(const double &driveSpeed) {

    m_frontRight->setDriveSpeed(driveSpeed);
    m_frontLeft->setDriveSpeed(driveSpeed);
    m_rearLeft->setDriveSpeed(driveSpeed);
    m_rearRight->setDriveSpeed(driveSpeed);
}

void SwerveTrain::setSwerveSpeed(const double &swerveSpeed) {

    m_frontRight->setSwerveSpeed(swerveSpeed);
    m_frontLeft->setSwerveSpeed(swerveSpeed);
    m_rearLeft->setSwerveSpeed(swerveSpeed);
    m_rearRight->setSwerveSpeed(swerveSpeed);
}

void SwerveTrain::setDriveBrake(const bool &brake) {

    m_frontRight->setDriveBrake(brake);
    m_frontLeft->setDriveBrake(brake);
    m_rearLeft->setDriveBrake(brake);
    m_rearRight->setDriveBrake(brake);
}

void SwerveTrain::setSwerveBrake(const bool &brake) {

    m_frontRight->setSwerveBrake(brake);
    m_frontLeft->setSwerveBrake(brake);
    m_rearLeft->setSwerveBrake(brake);
    m_rearRight->setSwerveBrake(brake);
}

void SwerveTrain::stop() {

    setDriveSpeed();
    setSwerveSpeed();
}

void SwerveTrain::setZeroPosition(const bool &verbose) {

    m_frontRight->setZeroPosition();
    m_frontLeft->setZeroPosition();
    m_rearLeft->setZeroPosition();
    m_rearRight->setZeroPosition();

    if (verbose) {

        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFR", m_frontRight->getSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFL", m_frontLeft->getSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRL", m_rearLeft->getSwerveZeroPosition());
        frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRR", m_rearRight->getSwerveZeroPosition());
    }
}

void SwerveTrain::PrintDrivePositions() {

    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFR", m_frontRight->getDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFL", m_frontLeft->getDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRL", m_rearLeft->getDrivePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRR", m_rearRight->getDrivePosition());
}

bool SwerveTrain::assumeZeroPosition() {

    return m_frontRight->assumeSwerveZeroPosition();
            m_frontLeft->assumeSwerveZeroPosition();
            m_rearLeft->assumeSwerveZeroPosition();
            m_rearRight->assumeSwerveZeroPosition();
}

bool SwerveTrain::assumeNearestZeroPosition() {

    return m_frontRight->assumeSwerveNearestZeroPosition() &&
           m_frontLeft->assumeSwerveNearestZeroPosition() &&
           m_rearLeft->assumeSwerveNearestZeroPosition() &&
           m_rearRight->assumeSwerveNearestZeroPosition();
}

bool SwerveTrain::assumeTurnAroundCenterPositions() {

    return  m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant) &&
            m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant) &&
            m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant) &&
            m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
}

bool SwerveTrain::setZionMotorsToVector(VectorDouble &vectorToSet) {

    double angle = navX->getYawFull();
    return m_frontRight->assumeSwervePosition(m_frontRight->getStandardDegreeSwervePosition(vectorToSet, angle)) &&
           m_frontLeft->assumeSwervePosition(m_frontLeft->getStandardDegreeSwervePosition(vectorToSet, angle)) &&
           m_rearLeft->assumeSwervePosition(m_rearLeft->getStandardDegreeSwervePosition(vectorToSet, angle)) &&
           m_rearRight->assumeSwervePosition(m_rearRight->getStandardDegreeSwervePosition(vectorToSet, angle));
}

void SwerveTrain::publishSwervePositions() {

    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFR", m_frontRight->getSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFL", m_frontLeft->getSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRL", m_rearLeft->getSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRR", m_rearRight->getSwervePosition());
}

void SwerveTrain::drive(const double rawX, const double rawY, const double rawZ, const bool precision, const bool record, const bool limelightLock) {

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
    forceControllerXYZToZeroInDeadzone(x, y, z);

    //To prevent accidental turning, optimize Z to X and Y's magnitude.
    optimizeControllerXYToZ(x, y, z);

    //If the controller is in the total deadzone (entirely still)...
    if (getControllerInDeadzone(x, y, z)) {

        /*
        Go to the nearest zero position, take it as the new zero, and
        reset the angle coming off of the gyroscope to zero to allow
        setting of the next translation vector, as the translation
        vector is always refereced from "center" (the beginning of the
        turn) and updated based on how much that gyro angle changes.
        Due to this, it must be reset when not in movement to allow
        this behavior to occur...
        */
        stop();
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
                z = calculateLimelightLockSpeed(m_limelight->getHorizontalOffset());
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
        can be written to the swerve modules using assumeSwervePosition(). They
        were in degrees before to enable the use of common trigonomoetry. We
        get Nics from degrees by calling getSwerveRotatingPosition().
        */
        m_frontRight->assumeSwervePosition(m_frontRight->getStandardDegreeSwervePosition(frontRightResultVector, angle));
        m_frontLeft->assumeSwervePosition(m_frontLeft->getStandardDegreeSwervePosition(frontLeftResultVector, angle));
        m_rearLeft->assumeSwervePosition(m_rearLeft->getStandardDegreeSwervePosition(rearLeftResultVector, angle));
        m_rearRight->assumeSwervePosition(m_rearRight->getStandardDegreeSwervePosition(rearRightResultVector, angle));

        const double executionCap = precision ? R_executionCapZionPrecision : R_executionCapZion;
        m_frontRight->setDriveSpeed(frontRightResultVector.magnitude() * executionCap);
        m_frontLeft->setDriveSpeed(frontLeftResultVector.magnitude() * executionCap);
        m_rearLeft->setDriveSpeed(rearLeftResultVector.magnitude() * executionCap);
        m_rearRight->setDriveSpeed(rearRightResultVector.magnitude() * executionCap);
    }
}

//TODO: Inline function documentation
double SwerveTrain::getStandardDegreeAngleFromCenter(const double &x, const double &y) {

    VectorDouble center(0, 1);
    VectorDouble current(x, y);

    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (magnitudeProduct == 0) {

        angleRad = 0;
    }

    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    angleRad *= (180. / M_PI);
    return angleRad;
}

bool SwerveTrain::getControllerInDeadzone(const double x, const double y, const double z) {

    return abs(x) < R_deadzoneController && abs(y) < R_deadzoneController && abs(z) < R_deadzoneController;
}

void SwerveTrain::forceControllerXYZToZeroInDeadzone(double &x, double &y, double &z) {

    if (abs(x) < R_deadzoneController) {x = 0;}
    if (abs(y) < R_deadzoneController) {y = 0;}
    if (abs(z) < R_deadzoneControllerZ) {z = 0;}
}

void SwerveTrain::optimizeControllerXYToZ(const double &x, const double &y, double &z) {

    double magnitudeXY = sqrt(x * x + y * y);
    double absZ = abs(z);
    double deadzoneAdjustmentZ = R_deadzoneControllerZ + .3 * magnitudeXY * R_deadzoneControllerZ;

    if (z > deadzoneAdjustmentZ) {

        z -= (deadzoneAdjustmentZ - R_deadzoneController);
    }
    else if (z < -deadzoneAdjustmentZ) {

        z += (deadzoneAdjustmentZ - R_deadzoneController);
    }
    if (absZ < deadzoneAdjustmentZ) {

        z = 0;
    }
}

//Almost exactly the same function as
//SwerveModule::calculateAssumePositionSpeed, except with constants for
//limelight lock
double SwerveTrain::calculateLimelightLockSpeed(const double &howFarRemainingInTravelInDegrees) {

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