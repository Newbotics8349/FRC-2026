// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private SwerveSubsystem m_SwerveSubsystem;
	private LEDSubsystem m_LedSubsystem;
	private VisionSubsystem m_VisionSubsystem;

	private CommandXboxController driveController = new CommandXboxController(Constants.ControllerConstants.kDriverController);

	public RobotContainer() {
		m_SwerveSubsystem = new SwerveSubsystem();
		m_LedSubsystem = new LEDSubsystem();
		m_VisionSubsystem = new VisionSubsystem(m_LedSubsystem);

		configureBindings();
	}

	private void configureBindings() {
		m_SwerveSubsystem.setDefaultCommand(new DriveJoystickCommand(
			m_SwerveSubsystem, 
			() -> driveController.getRawAxis(Constants.ControllerConstants.kDriverYAxis), 
			() -> driveController.getRawAxis(Constants.ControllerConstants.kDriverXAxis), 
			() -> driveController.getRawAxis(Constants.ControllerConstants.kDriverRotAxis), 
			() -> driveController.button(Constants.ControllerConstants.kDriverFieldOrientedButtonId).getAsBoolean(), 
			() -> driveController.getRawAxis(Constants.ControllerConstants.kDriverSlowAxis)));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
