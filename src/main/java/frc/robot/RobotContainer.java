// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	// public XboxController driverController = new XboxController(Constants.ControllerConstants.kDriverController);
	public CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.kDriverController);

	public RobotContainer() {
		configureBindings();

		swerveSubsystem.setDefaultCommand(new DriveJoystickCommand(
			swerveSubsystem,
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverYAxis),
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverXAxis),
			() -> driverController.getRawAxis(Constants.ControllerConstants.kDriverRotAxis),
			() -> driverController.a().getAsBoolean()));
	}

	private void configureBindings() {
		driverController.x().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
		driverController.button(2).onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
