// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
	/** Creates a new TurretSubsystem. */
	private final PIDController pid;
	private final SparkMax motor;
	private final DutyCycleEncoder encoder;

	public HoodSubsystem() {
		pid = new PIDController(Constants.ShooterConstants.hoodPID[0], Constants.ShooterConstants.hoodPID[1], Constants.ShooterConstants.hoodPID[2]);
		motor = new SparkMax(Constants.ShooterConstants.hoodMotorId, null);
		encoder = new DutyCycleEncoder(Constants.ShooterConstants.hoodEncoderPort, Math.PI * 2, 0);
	}

	public Rotation2d getPos() {
		return Rotation2d.fromRadians(encoder.get());
	}

	public void setPos(Rotation2d newPos) {
		pid.calculate(getPos().getRadians(), newPos.getRadians());
		motor.set(pid.calculate(getPos().getRadians(), newPos.getRadians()));
	}
	
	public void setPos(double newPos) {
		motor.set(pid.calculate(getPos().getRadians(), newPos));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
