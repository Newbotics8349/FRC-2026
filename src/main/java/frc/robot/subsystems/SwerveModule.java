// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId, boolean driveInverted) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig();
        config.inverted(driveInverted);
        driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = new CANcoder(turnEncoderId);
        turnPID = new PIDController(Constants.ModuleConstants.kP, Constants.ModuleConstants.kI, Constants.ModuleConstants.kD);
    }

    private double closestAngle(double a, double b) {
        double dir = (b % (Math.PI * 2)) - (a % (Math.PI * 2));
        if (Math.abs(dir) > Math.PI) {
            dir = -(Math.signum(dir) * 2 * Math.PI) + dir;
        }

        return dir;
    }

    public void setModuleState(SwerveModuleState speeds) {
        Rotation2d curAngle = getAngle();
        this.driveMotor.set(speeds.speedMetersPerSecond);
        SmartDashboard.putNumber("setpoint-" + turnMotor.getDeviceId(), closestAngle(curAngle.getRadians(), speeds.angle.getRadians()));
        SmartDashboard.putData(turnPID);
        double setpointAngle = closestAngle(curAngle.getRadians(), speeds.angle.getRadians());
        double setpointAngleFlipped = closestAngle(curAngle.getRadians(), speeds.angle.getRadians() + Math.PI);
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            this.driveMotor.set(Math.abs(this.driveMotor.get()));
            this.turnMotor.set(turnPID.calculate(getAngle().getRadians(), curAngle.getRadians() + setpointAngle));
        } else {
            this.driveMotor.set(Math.abs(this.driveMotor.get()) * -1);
            this.turnMotor.set(turnPID.calculate(getAngle().getRadians(), curAngle.getRadians() + setpointAngleFlipped));
        }
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
