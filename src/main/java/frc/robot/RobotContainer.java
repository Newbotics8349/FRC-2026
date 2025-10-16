// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	public Joystick driverController = new Joystick(Constants.ControllerConstants.kDriverController);

	public RobotContainer() {
		configureBindings();

		swerveSubsystem.setDefaultCommand(new DriveJoystickCommand(
			swerveSubsystem,
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverYAxis),
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverXAxis),
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverRotAxis),
			() -> driverController.getRawButton(Constants.ControllerConstants.kDriverFieldOrientedButtonId)));
	}

	private void configureBindings() {}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
