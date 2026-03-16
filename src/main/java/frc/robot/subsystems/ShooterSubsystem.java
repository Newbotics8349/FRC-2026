// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ShooterSubsystem extends SubsystemBase {
// 	/** Creates a new ShooterSubsystem. */
// 	private final SparkMax[] motors;
// 	public ShooterSubsystem() {
// 		motors = new SparkMax[] {
// 			// TODO motortype
// 			new SparkMax(Constants.ShooterConstants.shooterMotorIds[0], MotorType.kBrushed),
// 			new SparkMax(Constants.ShooterConstants.shooterMotorIds[1], MotorType.kBrushed),
// 		};
// 	}

// 	public void start() {
// 		motors[0].set(Constants.ShooterConstants.shooterMotorSpeed);
// 		// REVIEW negative?
// 		motors[1].set(Constants.ShooterConstants.shooterMotorSpeed);
// 	}

// 	public void stop() {
// 		for (SparkMax sparkMax : motors) {
// 			sparkMax.set(0);
// 		}
// 	}

// 	@Override
// 	public void periodic() {
// 		// This method will be called once per scheduler run
// 	}
// }
