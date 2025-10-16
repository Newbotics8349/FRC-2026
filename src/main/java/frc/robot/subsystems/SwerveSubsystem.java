// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro;

    private final SwerveDriveKinematics m_kinematics;

    private final SwerveModule[] swerveModules = {
        new SwerveModule(Constants.ModuleConstants.FrontLeft.kDriveMotorId, Constants.ModuleConstants.FrontLeft.kTurnMotorId, Constants.ModuleConstants.FrontLeft.kTurnEncoderId),
        new SwerveModule(Constants.ModuleConstants.FrontRight.kDriveMotorId, Constants.ModuleConstants.FrontRight.kTurnMotorId, Constants.ModuleConstants.FrontRight.kTurnEncoderId), 
        new SwerveModule(Constants.ModuleConstants.BackRight.kDriveMotorId, Constants.ModuleConstants.BackRight.kTurnMotorId, Constants.ModuleConstants.BackRight.kTurnEncoderId), 
        new SwerveModule(Constants.ModuleConstants.BackLeft.kDriveMotorId, Constants.ModuleConstants.BackLeft.kTurnMotorId, Constants.ModuleConstants.BackLeft.kTurnEncoderId), 
    };

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem() {
        gyro = new AHRS(NavXComType.kMXP_SPI, 50);

        Translation2d m_frontLeftLocation = Constants.DriveConstants.kFrontLeftLocation;
        Translation2d m_frontRightLocation = Constants.DriveConstants.kFrontRightLocation;
        Translation2d m_backRightLocation = Constants.DriveConstants.kBackRightLocation;
        Translation2d m_backLeftLocation = Constants.DriveConstants.kBackLeftLocation;

        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backRightLocation, m_backLeftLocation);
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setModuleState(moduleStates[i]);
        }
    }

    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void zeroHeading() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
