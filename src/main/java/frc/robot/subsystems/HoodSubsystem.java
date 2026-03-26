// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
	/** Creates a new TurretSubsystem. */
	private final PIDController pid;
	private final SparkMax motor;

	public HoodSubsystem() {
		pid = new PIDController(Constants.ShooterConstants.hoodPID[0], Constants.ShooterConstants.hoodPID[1], Constants.ShooterConstants.hoodPID[2]);
		motor = new SparkMax(Constants.ShooterConstants.hoodMotorId, MotorType.kBrushless);
	}

	public Rotation2d getPos() {
		return Rotation2d.fromRadians(Math.PI / 4 - motor.getEncoder().getPosition() * 2 * Math.PI * Units.degreesToRadians(22) / 4.6);
	}

	public void setPos(Rotation2d newPos) {
		pid.calculate(getPos().getRadians(), newPos.getRadians());
		motor.set(pid.calculate(getPos().getRadians(), newPos.getRadians()));
	}
	
	public void setPos(double newPos) {
		if (newPos < Units.degreesToRadians(45) && newPos > Units.degreesToRadians(23)) {
			motor.set(-pid.calculate(getPos().getRadians(), newPos));
		}
	}

    public void stop() {
        motor.set(0);
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        SmartDashboard.putNumber("Hood encoder", getPos().getRadians());
	}
}
