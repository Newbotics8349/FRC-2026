// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OtherConstants;

public class DrawerSubsystem extends SubsystemBase {
	/** Creates a new ConveyorSubsystem. */
	public SparkMax motor = new SparkMax(OtherConstants.kDrawerMotorId, MotorType.kBrushless);
	public DrawerSubsystem() {}

	public Command start() {
		return this.runOnce(() -> motor.set(OtherConstants.kDrawerMotorSpeed));
	}

	public Command stop() {
		return this.runOnce(() -> motor.set(0));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
