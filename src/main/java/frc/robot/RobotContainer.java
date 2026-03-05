// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.commands.DriveToCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
	private final SendableChooser<Constants.AprilTag> m_AprilTagChooser = new SendableChooser<>();

	private final SwerveSubsystem m_SwerveSubsystem;
	private final LEDSubsystem m_LedSubsystem;
	private final VisionSubsystem m_VisionSubsystem;

	private CommandXboxController driveController = new CommandXboxController(Constants.ControllerConstants.kDriverController);

	public RobotContainer() {
		m_SwerveSubsystem = new SwerveSubsystem();
		m_LedSubsystem = new LEDSubsystem();
		m_VisionSubsystem = new VisionSubsystem(m_LedSubsystem);

		m_AutoChooser.setDefaultOption("Test Auto", Commands.startEnd(() -> System.out.println("Test auto"), () -> System.out.println("end"), m_VisionSubsystem));
		m_AutoChooser.addOption("Test Auto 2", Commands.startEnd(() -> System.out.println("Second auto"), () -> System.out.println("end"), m_VisionSubsystem));
		SmartDashboard.putData("Auto Chooser", m_AutoChooser);

		m_AprilTagChooser.setDefaultOption("20", Constants.AprilTag.Test2);
		m_AprilTagChooser.addOption("21", Constants.AprilTag.Test);
		SmartDashboard.putData("April Tag Chooser", m_AprilTagChooser);

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
		
		driveController.y().whileTrue(new DriveToCommand(m_SwerveSubsystem, m_VisionSubsystem, () -> m_AprilTagChooser.getSelected()));
		driveController.a().onTrue(m_SwerveSubsystem.zeroHeading());
	}

	public Command getAutonomousCommand() {
		return m_AutoChooser.getSelected();
	}
}
