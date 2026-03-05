// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
	/** Creates a new ExtensionSubsystem. */
	private SparkMax motor;
	// TODO absolute encoder (check type)
	// TODO constants
	private PIDController pid;
	

	public ExtensionSubsystem() {
		// TODO constants
		motor = new SparkMax(0, null);
		pid = new PIDController(0, 0, 0);
	}

	// REVIEW do based off of angle
	public void in() {

	}

	public void out() {

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
