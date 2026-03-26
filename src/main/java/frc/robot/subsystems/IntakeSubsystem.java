// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
	/** Creates a new IntakeSubsystem. */
	private SparkMax motor;

	public IntakeSubsystem() {
		motor = new SparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushed);
		SmartDashboard.putNumber("intake speed", -0.34);
	}

	public void start() {
		System.out.println("here");
		motor.set(SmartDashboard.getNumber("intake speed", 0));
	}

	public void stop() {
		motor.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
