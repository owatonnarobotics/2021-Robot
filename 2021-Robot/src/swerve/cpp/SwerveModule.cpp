#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.h"

SwerveModule::SwerveModule(const int &canDriveID, const int &canSwerveID) {

    m_driveMotor = new rev::CANSparkMax(canDriveID, rev::CANSparkMax::MotorType::kBrushless);
    m_driveMotorEncoder = new rev::CANEncoder(m_driveMotor->GetEncoder());
    m_swerveMotor = new rev::CANSparkMax(canSwerveID, rev::CANSparkMax::MotorType::kBrushless);
    m_swerveMotorEncoder = new rev::CANEncoder(m_swerveMotor->GetEncoder());

    //Default the swerve's zero position to its power-on position.
    m_swerveZeroPosition = m_swerveMotorEncoder->GetPosition();

    //Allow the drive motor to coast, but brake the swerve motor for accuracy.
    //These must be set as they become overwritten from code.
    m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void SwerveModule::SetDriveSpeed(const double &speedToSet) {

    m_driveMotor->Set(speedToSet);
}

void SwerveModule::SetSwerveSpeed(const double &speedToSet) {

    m_swerveMotor->Set(speedToSet);
}

void SwerveModule::SetDriveBrake(const bool &brake) {

    if (brake) {

        m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else {

        m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

void SwerveModule::SetSwerveBrake(const bool &brake) {

    if (brake) {

        m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else {

        m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

void SwerveModule::Stop() {

    SetDriveSpeed();
    SetSwerveSpeed();
}

void SwerveModule::SetZeroPosition() {

    m_swerveZeroPosition = m_swerveMotorEncoder->GetPosition();
}

double SwerveModule::GetDrivePosition() {

    return m_driveMotorEncoder->GetPosition();
}

double SwerveModule::GetSwervePosition() {

    return m_swerveMotorEncoder->GetPosition();
}

double SwerveModule::GetSwervePositionSingleRotation() {

    double clockwiseNicsFromZero = m_swerveMotorEncoder->GetPosition() - m_swerveZeroPosition;
    //If more than a full rotation from zero...
    if (clockwiseNicsFromZero >= R_nicsConstant) {

        //Return the most local equivalent position...
        return fmod(clockwiseNicsFromZero, R_nicsConstant);
    }
    else {

        //Otherwise, return only the position.
        return clockwiseNicsFromZero;
    }
}

double SwerveModule::GetSwerveZeroPosition() {

    return m_swerveZeroPosition;
}

double SwerveModule::GetDriveSpeed() {

    return m_driveMotorEncoder->GetVelocity();
}

double SwerveModule::GetSwerveSpeed() {

    return m_swerveMotorEncoder->GetVelocity();
}

double SwerveModule::AbsoluteVectorToNics(VectorDouble &vector, const double &angle) {

    return R_nicsConstant * (vector.unitCircleAngleDeg() + angle - 90.) / 360.;
}

bool SwerveModule::AssumeSwervePosition(const double &positionToAssumeRaw) {

    double currentPosition = GetSwervePositionSingleRotation();
    double positionToAssume = fmod(positionToAssumeRaw, R_nicsConstant);

    

    frc::SmartDashboard::PutNumber("ASP", abs(abs(fmod(abs(positionToAssume - GetSwervePositionSingleRotation()), R_nicsConstant) - R_nicsConstant / 2) - R_nicsConstant / 2));

    //If the current position is close enough to where we want to go (within one tolerance value)...
    if (IsAtPositionWithinTolerance(positionToAssume)) {

        //Stop rotating the swerve motor and skip checking anything else...
        m_swerveMotor->Set(0);
        return true;
    }
    //If the position to assume is greater than half a revolution...
    else if (abs(positionToAssume - currentPosition) > R_nicsConstant / 2) {

        //If such a rotation needs to be clockwise...
        if (positionToAssume < currentPosition) {

            //Set the speed of the motor using the Nic's Constant distance between the two points...
            m_swerveMotor->Set(calculateAssumePositionSpeed(R_nicsConstant - (currentPosition - positionToAssume)));
        }
        //If such a rotation needs to be counterclockwise...
        else if (positionToAssume > currentPosition) {

            //Set the speed similarly, but negatively...
            m_swerveMotor->Set(calculateAssumePositionSpeed(-R_nicsConstant + (positionToAssume - currentPosition)));
        }
    }
    else {

        //Otherwise, perform a normal between two points rotation with a Nic's Constant value.
        m_swerveMotor->Set(calculateAssumePositionSpeed(positionToAssume - currentPosition));
    }
    return false;
}

bool SwerveModule::AssumeSwerveZeroPosition() {

    return AssumeSwervePosition(getSwerveNearestZeroPosition());
}

bool SwerveModule::IsAtPositionWithinTolerance(const double &position) {

    //If the current position is close enough to where we want to go (within one tolerance value)...
    return abs(abs(fmod(abs(position - GetSwervePositionSingleRotation()), R_nicsConstant) - R_nicsConstant / 2) - R_nicsConstant / 2) < R_swerveTrainAssumePositionTolerance;
}

double SwerveModule::calculateAssumePositionSpeed(const double &howFarRemainingInTravel) {

    //Begin initally with a double calculated with the simplex function...
    double toReturn = ((1) / (1 + exp((-1 * abs(howFarRemainingInTravel)) + 5)));
    //If we satisfy conditions for the first linear piecewiseposition, take that speed instead...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed;
    }
    //Do the same for the second...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed;
    }
    //And if we needed to travel negatively to get where we need to be, make the final speed negative...
    if (howFarRemainingInTravel < 0) {

        toReturn = -toReturn;
    }
    return toReturn;
}

double SwerveModule::getSwerveNearestZeroPosition() {	

    //If a full rotation minus the curent position is less than half of Nic's Constant,	
    //the position is within the second or third quadrant, so a rotation to	
    //Nic's Constant is the fastest path...	
    if (R_nicsConstant - GetSwervePositionSingleRotation() < (R_nicsConstant / 2)) {	

        return R_nicsConstant;	
    }	
    //Otherwise, going to 0 from the first or second quadrant is going to be faster.	
    else {	

        return 0;	
    }	
}
