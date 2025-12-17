// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;

    private final SwerveDriveKinematics m_kinematics;

    private final SwerveModule[] swerveModules = {
        new SwerveModule(Constants.ModuleConstants.FrontLeft.kDriveMotorId, Constants.ModuleConstants.FrontLeft.kTurnMotorId, Constants.ModuleConstants.FrontLeft.kTurnEncoderId, Constants.ModuleConstants.FrontLeft.driveInverted),
        new SwerveModule(Constants.ModuleConstants.FrontRight.kDriveMotorId, Constants.ModuleConstants.FrontRight.kTurnMotorId, Constants.ModuleConstants.FrontRight.kTurnEncoderId, Constants.ModuleConstants.FrontRight.driveInverted), 
        new SwerveModule(Constants.ModuleConstants.BackRight.kDriveMotorId, Constants.ModuleConstants.BackRight.kTurnMotorId, Constants.ModuleConstants.BackRight.kTurnEncoderId, Constants.ModuleConstants.BackRight.driveInverted), 
        new SwerveModule(Constants.ModuleConstants.BackLeft.kDriveMotorId, Constants.ModuleConstants.BackLeft.kTurnMotorId, Constants.ModuleConstants.BackLeft.kTurnEncoderId, Constants.ModuleConstants.BackLeft.driveInverted), 
    };

    private final StructArrayPublisher<SwerveModuleState> publisher;
    private final StructArrayPublisher<ChassisSpeeds> publisher2;

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.DriveConstants.kGyroId, "rio");

        Translation2d m_frontLeftLocation = Constants.DriveConstants.kFrontLeftLocation;
        Translation2d m_frontRightLocation = Constants.DriveConstants.kFrontRightLocation;
        Translation2d m_backRightLocation = Constants.DriveConstants.kBackRightLocation;
        Translation2d m_backLeftLocation = Constants.DriveConstants.kBackLeftLocation;

        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backRightLocation, m_backLeftLocation);

        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
        publisher2 = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveChassis", ChassisSpeeds.struct).publish();
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setModuleState(moduleStates[i]);
        }      
        
        publisher2.set(new ChassisSpeeds[] {
            chassisSpeeds
        });

        publisher.set(new SwerveModuleState[] {
            moduleStates[0],
            moduleStates[1],
            moduleStates[2],
            moduleStates[3]
        });
    }

    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
