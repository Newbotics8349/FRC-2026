// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
	/** Creates a new TurretSubsystem. */
	private final PIDController pid;
	private final SparkMax motor;
	private final DutyCycleEncoder encoder;

	public TurretSubsystem() {
		pid = new PIDController(0, 0, 0);
		// TODO constants
		motor = new SparkMax(0, null);
		encoder = new DutyCycleEncoder(0, Math.PI * 2, 0);
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
