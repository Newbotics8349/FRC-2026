// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtensionSubsystem extends SubsystemBase {
	/** Creates a new ExtensionSubsystem. */
	private SparkMax motor;
	private PIDController pid;
	private DutyCycleEncoder encoder;
	

	public ExtensionSubsystem() {
		motor = new SparkMax(Constants.IntakeConstants.extensionMotorId, null);
		encoder = new DutyCycleEncoder(Constants.IntakeConstants.extensionEncoderPort, 2 * Math.PI, 0);
		pid = new PIDController(Constants.IntakeConstants.extensionPID[0], Constants.IntakeConstants.extensionPID[1], Constants.IntakeConstants.extensionPID[2]);
	}

	public void in() {
		motor.set(pid.calculate(encoder.get(), Constants.IntakeConstants.extensionIn));
	}

	public void out() {
		motor.set(pid.calculate(encoder.get(), Constants.IntakeConstants.extensionOut));

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
