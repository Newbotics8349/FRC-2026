// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    SparkMax driveMotor;
    SparkMax turnMotor;

    CANcoder turnEncoder;
    PIDController turnPID;
    /** Creates a new SwerveModule. */
    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        turnEncoder = new CANcoder(turnEncoderId);
        turnPID = new PIDController(Constants.ModuleConstants.kP, Constants.ModuleConstants.kI, Constants.ModuleConstants.kD);
        SmartDashboard.putData(turnPID);
    }

    public void setModuleState(SwerveModuleState speeds) {
        Rotation2d curAngle = getAngle();
        speeds.optimize(curAngle);
        speeds.speedMetersPerSecond *= speeds.angle.minus(curAngle).getCos();
        SmartDashboard.putNumber("speedMetersPerSecond", speeds.speedMetersPerSecond);
        this.driveMotor.set(speeds.speedMetersPerSecond);
        this.turnMotor.set(turnPID.calculate(getAngle().getDegrees(), speeds.angle.getDegrees()));
        SmartDashboard.putNumber("encoder", getAngle().getDegrees());
        SmartDashboard.putNumber("turnPID" + turnEncoder, turnPID.calculate(getAngle().getDegrees(), speeds.angle.getDegrees()));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
