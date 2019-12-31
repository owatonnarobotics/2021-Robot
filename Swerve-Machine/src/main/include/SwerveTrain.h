/*
class SwerveModule

Constructors

    SwerveTrain(SwerveModule*, SwerveModule*, SwerveModule*, SwerveModule*):
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions.

Public Methods

    void setSwerveZeroPosition(const bool&): Gets the current encoder values of the swerve
        motors and stores them as privates of the class. These are the values
        the swerve motors return to when invoking assumeSwerveZeroPosition().
        If the passed bool is true, publishes the stored data to the SmartDashboard.
        This is currently used for returning to and maintaining "straight".
    void assumeSwerveZeroPosition(): Drives the swerves to return to their
        zero position.
    void publishSwervePositions(): Puts the current swerve encoder positions
        to the SmartDashboard.

*/

#pragma once

#include <SmartDashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include "SwerveModule.h"

class SwerveTrain {

    public:
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule) {

            m_frontRight = &frontRightModule;
            m_frontLeft = &frontLeftModule;
            m_rearLeft = &rearLeftModule;
            m_rearRight = &rearRightModule;

            m_frontRightSwerveZeroPosition = 0;
            m_frontLeftSwerveZeroPosition = 0;
            m_rearLeftSwerveZeroPosition = 0;
            m_rearRightSwerveZeroPosition = 0;
        }

        void setSwerveZeroPosition(const bool &verbose = false) {

            m_frontRightSwerveZeroPosition = m_frontRight->getSwervePosition();
            m_frontLeftSwerveZeroPosition = m_frontLeft->getSwervePosition();
            m_rearLeftSwerveZeroPosition = m_rearLeft->getSwervePosition();
            m_rearRightSwerveZeroPosition = m_rearRight->getSwervePosition();

            if (verbose) {

                frc::SmartDashboard::PutNumber("FR Swrv Pos0", m_frontRightSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("FL Swrv Pos0", m_frontLeftSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("RL Swrv Pos0", m_rearLeftSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("RR Swrv Pos0", m_rearRightSwerveZeroPosition);
            }
        }
        void assumeSwerveZeroPosition() {

            m_frontRight->assumeSwervePosition(m_frontRightSwerveZeroPosition);
            m_frontLeft->assumeSwervePosition(m_frontLeftSwerveZeroPosition);
            m_rearLeft->assumeSwervePosition(m_rearLeftSwerveZeroPosition);
            m_rearRight->assumeSwervePosition(m_rearRightSwerveZeroPosition);
        }
        void publishSwervePositions() {

            frc::SmartDashboard::PutNumber("FR Swrv Pos", m_frontRight->getSwervePosition());
            frc::SmartDashboard::PutNumber("FL Swrv Pos", m_frontLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RL Swrv Pos", m_rearLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RR Swrv Pos", m_rearRight->getSwervePosition());
        }

    private:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;

        double m_frontRightSwerveZeroPosition;
        double m_frontLeftSwerveZeroPosition;
        double m_rearLeftSwerveZeroPosition;
        double m_rearRightSwerveZeroPosition;
};
