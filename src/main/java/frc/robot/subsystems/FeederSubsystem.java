// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
	/** Creates a new FeederSubsystem. */
	private SparkMax motor;

	public FeederSubsystem() {
		motor = new SparkMax(Constants.IntakeConstants.feederMotorId, null);
	}

	public void start() {
		motor.set(Constants.IntakeConstants.feederMotorSpeed);
	}

	public void stop() {
		motor.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
