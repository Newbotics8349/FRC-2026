// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OtherConstants;

public class IntakeSubsystem extends SubsystemBase {
	/** Creates a new ConveyorSubsystem. */
	public SparkMax motor1 = new SparkMax(OtherConstants.kIntakeMotor1Id, MotorType.kBrushless);
	public SparkMax motor2 = new SparkMax(OtherConstants.kIntakeMotor2Id, MotorType.kBrushless);
	public IntakeSubsystem() {}

	public Command start() {
		return this.runOnce(() -> {
			motor1.set(OtherConstants.kIntakeMotorSpeed);
			motor2.set(-1 * OtherConstants.kIntakeMotorSpeed);
		});
	}

	public Command stop() {
		return this.runOnce(() -> {
			motor1.set(0);
			motor2.set(0);
		});
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
