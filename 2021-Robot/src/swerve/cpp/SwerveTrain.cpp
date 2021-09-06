#include <math.h>

#include <frc/DriverStation.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"

SwerveTrain::SwerveTrain(
        const int frontRightCANDriveID,
        const int frontRightCANSwerveID,
        const int frontLeftCANDriveID,
        const int frontLeftCANSwerveID,
        const int rearLeftCANDriveID,
        const int rearLeftCANSwerveID,
        const int rearRightCANDriveID,
        const int rearRightCANSwerveID,
        NavX &navXToSet
    ) {

    m_frontRight = new SwerveModule(frontRightCANDriveID, frontRightCANSwerveID);
    m_frontLeft = new SwerveModule(frontLeftCANDriveID, frontLeftCANSwerveID);
    m_rearLeft = new SwerveModule(rearLeftCANDriveID, rearLeftCANSwerveID);
    m_rearRight = new SwerveModule(rearRightCANDriveID, rearRightCANSwerveID);
    navX = &navXToSet;
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

    bool fr = m_frontRight->AssumeSwerveZeroPosition();
    bool fl = m_frontLeft->AssumeSwerveZeroPosition();
    bool rl = m_rearLeft->AssumeSwerveZeroPosition();
    bool rr = m_rearRight->AssumeSwerveZeroPosition();
    return fr && fl && rl && rr;
}

bool SwerveTrain::AssumeTurnAroundCenterPositions() {

    bool fr = m_frontRight->AssumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
    bool fl = m_frontLeft->AssumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
    bool rl = m_rearLeft->AssumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
    bool rr = m_rearRight->AssumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
    return  fr && fl && rl && rr;
}

bool SwerveTrain::SetZionMotorsToVector(VectorDouble &vectorToSet) {

    double angle = navX->getYawFull();
    bool fr = m_frontRight->AssumeSwervePosition(m_frontRight->AbsoluteVectorToNics(vectorToSet, angle));
    bool fl = m_frontLeft->AssumeSwervePosition(m_frontLeft->AbsoluteVectorToNics(vectorToSet, angle));
    bool rl = m_rearLeft->AssumeSwervePosition(m_rearLeft->AbsoluteVectorToNics(vectorToSet, angle));
    bool rr = m_rearRight->AssumeSwervePosition(m_rearRight->AbsoluteVectorToNics(vectorToSet, angle));
    frc::SmartDashboard::PutBoolean("fr at vector", fr);
    frc::SmartDashboard::PutBoolean("fl at vector", fl);
    frc::SmartDashboard::PutBoolean("rl at vector", rl);
    frc::SmartDashboard::PutBoolean("rr at vector", rr);
    return  fr && fl && rl && rr;
}

void SwerveTrain::PrintSwervePositions() {

    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFR", m_frontRight->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosFL", m_frontLeft->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRL", m_rearLeft->GetSwervePosition());
    frc::SmartDashboard::PutNumber("Zion::Swerve::PosRR", m_rearRight->GetSwervePosition());
}

void SwerveTrain::Drive(const double &x, const double &y, const double rawZ, const bool &precision, const bool &relative, const bool &hold, const double throttle) {

    static double holdAngle = 0.0;
    static bool wasHolding = false;

    frc::SmartDashboard::PutNumber("Throttle", throttle);

    if (!hold && x == 0 && y == 0 && rawZ == 0) {

        Stop();
    }
    else {
        
        double z = rawZ;

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

        double angle;
        //Get the navX yaw angle for computing the field-oriented
        //angle only if field orienteds
        if (relative) {

            angle = 0;
        }
        else if (hold) {

            /*if (!wasHolding) {

                holdAngle = navX->getYawFull();
            }*/
            angle = navX->getYaw() - holdAngle;
            //wasHolding = true;
        }
        else {

            angle = navX->getYawFull();
            //wasHolding = false;
        }

        frc::SmartDashboard::PutNumber("Angle", angle);

        if (hold) {

            //Update our rotational speed so that we turn towards the goal.
            //Begin initally with a double calculated with the simplex function with a horizontal stretch of factor two...
            z = ((1) / (1 + exp((-1 * abs(1.0 / 4.0 * angle)) + 5)));
            //If we satisfy conditions for the first linear piecewise, take that speed instead...
            if (abs(angle) < R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorAt) {

                z = R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorSpeed;
            }
            //Do the same for the second...
            /*if (abs(angle) < R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorAt) {

                z = R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorSpeed;
            }*/
            //And if we needed to travel negatively to get where we need to be, make the final speed negative...
            if (abs(angle) < R_swerveTrainHoldAngleTolerance) {

                z = 0;
            }
            if (angle < 0) {

                z = -z;
            }
            z *= -1;
            angle = navX->getYawFull();
            frc::SmartDashboard::PutNumber("HoldAngleSpeedCalculaton", z);
        }

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

        const double executionCap = throttle;//precision ? R_executionCapZionPrecision : R_executionCapZion;
        double fr = frontRightResultVector.magnitude() * executionCap;
        double fl = frontLeftResultVector.magnitude() * executionCap;
        double rl = rearLeftResultVector.magnitude() * executionCap;
        double rr = rearRightResultVector.magnitude() * executionCap;
        frc::SmartDashboard::PutNumber("fr", fr);
        frc::SmartDashboard::PutNumber("fl", fl);
        frc::SmartDashboard::PutNumber("rl", rl);
        frc::SmartDashboard::PutNumber("rr", rr);
        m_frontRight->SetDriveSpeed(fr);
        m_frontLeft->SetDriveSpeed(fl);
        m_rearLeft->SetDriveSpeed(rl);
        m_rearRight->SetDriveSpeed(rr);
    }
}