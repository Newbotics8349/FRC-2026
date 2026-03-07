// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
	/** Creates a new ConveyorSubsystem. */
	private SparkMax motor;

	public ConveyorSubsystem() {
		motor = new SparkMax(Constants.IntakeConstants.conveyorMotorId, null);
	}

	public void start() {
		motor.set(Constants.IntakeConstants.conveyorMotorSpeed);
	}

	public void stop() {
		motor.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
