// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	/** Creates a new ShooterSubsystem. */
	private final SparkMax[] motors;
	public ShooterSubsystem() {
		motors = new SparkMax[] {
			// TODO constants
			new SparkMax(0, null),
			new SparkMax(0, null),
		};
	}

	public void start() {
		// TODO constants
		motors[0].set(0);
		// TODO negative?
		motors[1].set(0);
	}

	public void stop() {
		for (SparkMax sparkMax : motors) {
			sparkMax.set(0);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
