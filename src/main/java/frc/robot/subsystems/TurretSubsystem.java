// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class TurretSubsystem extends SubsystemBase {
// 	/** Creates a new TurretSubsystem. */
// 	private final PIDController pid;
// 	private final SparkMax motor;
// 	private final DutyCycleEncoder encoder;

// 	public TurretSubsystem() {
// 		pid = new PIDController(Constants.ShooterConstants.turretPID[0], Constants.ShooterConstants.turretPID[1], Constants.ShooterConstants.turretPID[2]);
// 			// TODO motortype
// 		motor = new SparkMax(Constants.ShooterConstants.turretMotorId, MotorType.kBrushed);
// 		encoder = new DutyCycleEncoder(Constants.ShooterConstants.turretEncoderPort, Math.PI * 2, 0);
// 	}

// 	public Rotation2d getPos() {
// 		return Rotation2d.fromRadians(encoder.get());
// 	}

// 	public void setPos(Rotation2d newPos) {
// 		if (newPos.getRadians() < Math.PI && newPos.getRadians() > -1 * Math.PI) {
// 			pid.calculate(getPos().getRadians(), newPos.getRadians());
// 			motor.set(pid.calculate(getPos().getRadians(), newPos.getRadians()));
// 		}
// 	}
	
// 	public void setPos(double newPos) {
// 		motor.set(pid.calculate(getPos().getRadians(), newPos));
// 	}

// 	@Override
// 	public void periodic() {
// 		// This method will be called once per scheduler run
// 	}
// }
