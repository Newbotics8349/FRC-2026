// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    SparkMax driveMotor;
    SparkMax turnMotor;

    DutyCycleEncoder turnEncoder;
    PIDController turnPID;
    /** Creates a new SwerveModule. */
    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        turnEncoder = new DutyCycleEncoder(turnEncoderId, 360, 0);
        turnPID = new PIDController(Constants.ModuleConstants.kP, Constants.ModuleConstants.kI, Constants.ModuleConstants.kD);
    }

    public void setModuleState(SwerveModuleState speeds) {
        Rotation2d curAngle = Rotation2d.fromDegrees(turnEncoder.get());
        speeds.optimize(curAngle);
        speeds.speedMetersPerSecond *= speeds.angle.minus(curAngle).getCos();
        this.driveMotor.set(speeds.speedMetersPerSecond);
        this.turnMotor.set(turnPID.calculate(turnEncoder.get(), speeds.angle.getDegrees()));
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
