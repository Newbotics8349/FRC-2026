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
import frc.robot.commands.DriveToPosCommand;
import frc.robot.commands.FeedAllianceCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.TurretPosCommand;
import frc.robot.subsystems.HoodSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OmniSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

	private final SwerveSubsystem m_SwerveSubsystem;
	// private final LEDSubsystem m_LedSubsystem;
	private final VisionSubsystem m_VisionSubsystem;
	private final FeederSubsystem m_FeederSubsystem;
	private final ShooterSubsystem m_ShooterSubsystem;
	private final OmniSubsystem m_OmniSubsystem;
	private final IntakeSubsystem m_IntakeSubsystem;
	private final ExtensionSubsystem m_ExtensionSubsystem;
	private final TurretSubsystem m_TurretSubsystem;
	private final HoodSubsystem m_HoodSubsystem;

	private CommandXboxController driveController = new CommandXboxController(Constants.ControllerConstants.kDriverController);

	// public RobotContainer(ConveyorSubsystem conveyorSubsystem, LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem) {
	// 	m_SwerveSubsystem = new SwerveSubsystem();
	// 	m_LedSubsystem = ledSubsystem;
	// 	m_VisionSubsystem = new VisionSubsystem(m_LedSubsystem);
	// 	m_FeederSubsystem = new FeederSubsystem();
	// 	m_ShooterSubsystem = shooterSubsystem;
	// 	m_OmniSubsystem = new OmniSubsystem();
	// 	m_IntakeSubsystem = new IntakeSubsystem();
	// 	m_ConveyorSubsystem = conveyorSubsystem;
	// 	m_ExtensionSubsystem = new ExtensionSubsystem();
	// 	m_TurretSubsystem = new TurretSubsystem();
	// 	m_HoodSubsystem = new HoodSubsystem();

	// 	m_AutoChooser.setDefaultOption("Test Auto", Commands.startEnd(() -> System.out.println("Test auto"), () -> System.out.println("end"), m_VisionSubsystem));
	// 	m_AutoChooser.addOption("Test Auto 2", Commands.startEnd(() -> System.out.println("Second auto"), () -> System.out.println("end"), m_VisionSubsystem));
	// 	SmartDashboard.putData("Auto Chooser", m_AutoChooser);

	// 	m_AprilTagChooser.setDefaultOption("20", Constants.AprilTag.Test2);
	// 	m_AprilTagChooser.addOption("21", Constants.AprilTag.Test);
	// 	SmartDashboard.putData("April Tag Chooser", m_AprilTagChooser);

	// 	configureBindings();
	// }

	public RobotContainer(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem) {
		m_SwerveSubsystem = new SwerveSubsystem();
		// m_LedSubsystem = ledSubsystem;
		// m_VisionSubsystem = new VisionSubsystem(m_LedSubsystem);
		m_VisionSubsystem = new VisionSubsystem();
		m_FeederSubsystem = new FeederSubsystem();
		m_ShooterSubsystem = shooterSubsystem;
		m_OmniSubsystem = new OmniSubsystem();
		m_IntakeSubsystem = new IntakeSubsystem();
		m_ExtensionSubsystem = new ExtensionSubsystem();
		m_TurretSubsystem = new TurretSubsystem();
		m_HoodSubsystem = new HoodSubsystem();

		// REVIEW auto chooser
		m_AutoChooser.setDefaultOption("Auto center", Commands.sequence(
			new DriveToPosCommand(m_SwerveSubsystem, -1, 0, 0),
			Commands.parallel(
				new TurretPosCommand(m_VisionSubsystem, m_TurretSubsystem, m_HoodSubsystem, 
									() -> m_SwerveSubsystem.getChassisSpeeds(),
									() -> m_VisionSubsystem.getTargets()),
				Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
				Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
				Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem)
			)

		));
		m_AutoChooser.addOption("Auto left", Commands.sequence(
			new DriveToPosCommand(m_SwerveSubsystem, -1, 0, Math.PI / 4),
			Commands.parallel(
				new TurretPosCommand(m_VisionSubsystem, m_TurretSubsystem, m_HoodSubsystem, 
									() -> m_SwerveSubsystem.getChassisSpeeds(),
									() -> m_VisionSubsystem.getTargets()),
				Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
				Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
				Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem)
			)

		));
		m_AutoChooser.addOption("Auto right", Commands.sequence(
			new DriveToPosCommand(m_SwerveSubsystem, -1, 0, Math.PI / -4),
			Commands.parallel(
				new TurretPosCommand(m_VisionSubsystem, m_TurretSubsystem, m_HoodSubsystem, 
									() -> m_SwerveSubsystem.getChassisSpeeds(),
									() -> m_VisionSubsystem.getTargets()),
				Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
				Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
				Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem)
			)

		));
		SmartDashboard.putData("Auto Chooser", m_AutoChooser);

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
		
		// driveController.y().whileTrue(new DriveToCommand(m_SwerveSubsystem, m_VisionSubsystem, () -> m_AprilTagChooser.getSelected()));
		driveController.a().onTrue(m_SwerveSubsystem.zeroHeading());
		driveController.x().whileTrue(Commands.parallel(
			new TurretPosCommand(m_VisionSubsystem, m_TurretSubsystem, m_HoodSubsystem, 
								() -> m_SwerveSubsystem.getChassisSpeeds(),
								() -> m_VisionSubsystem.getTargets()),
			Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
			Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
			Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem)
		)).onFalse(Commands.parallel(
			Commands.runOnce(() -> m_ShooterSubsystem.start(false), m_ShooterSubsystem),
			Commands.runOnce(() -> m_FeederSubsystem.stop(), m_FeederSubsystem),
			Commands.runOnce(() -> m_OmniSubsystem.stop(), m_OmniSubsystem)
		));

		driveController.b().whileTrue(
			Commands.run(() -> m_IntakeSubsystem.start(), m_IntakeSubsystem)
		).onFalse(
			Commands.runOnce(() -> m_IntakeSubsystem.stop(), m_IntakeSubsystem)
		);

		driveController.povLeft().whileTrue(
			Commands.parallel(new FeedAllianceCommand(m_TurretSubsystem, m_HoodSubsystem, true),
							Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
							Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
							Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem))
		).onFalse(
			Commands.parallel(
				Commands.runOnce(() -> m_ShooterSubsystem.start(false), m_ShooterSubsystem),
				Commands.runOnce(() -> m_FeederSubsystem.stop(), m_FeederSubsystem),
				Commands.runOnce(() -> m_OmniSubsystem.stop(), m_OmniSubsystem)
			)
		);

		driveController.povRight().whileTrue(
			Commands.parallel(new FeedAllianceCommand(m_TurretSubsystem, m_HoodSubsystem, false),
							Commands.runOnce(() -> m_ShooterSubsystem.start(), m_ShooterSubsystem),
							Commands.runOnce(() -> m_FeederSubsystem.start(), m_FeederSubsystem),
							Commands.runOnce(() -> m_OmniSubsystem.start(), m_OmniSubsystem))
		).onFalse(
			Commands.parallel(
				Commands.runOnce(() -> m_ShooterSubsystem.start(false), m_ShooterSubsystem),
				Commands.runOnce(() -> m_FeederSubsystem.stop(), m_FeederSubsystem),
				Commands.runOnce(() -> m_OmniSubsystem.stop(), m_OmniSubsystem)
			)
		);
			
		driveController.povUp().whileTrue(
			Commands.run(() -> m_ExtensionSubsystem.in(), m_ExtensionSubsystem)
		).onFalse(
			Commands.runOnce(() -> m_ExtensionSubsystem.stop(), m_ExtensionSubsystem)
		);

		driveController.povDown().whileTrue(
			Commands.run(() -> m_ExtensionSubsystem.out(), m_ExtensionSubsystem)
		).onFalse(
			Commands.runOnce(() -> m_ExtensionSubsystem.stop(), m_ExtensionSubsystem)
		);
	}

	public Command getAutonomousCommand() {
		return m_AutoChooser.getSelected();
	}
}
