// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedAllianceCommand extends Command {
	/** Creates a new FeedAllianceCommand. */
	private final TurretSubsystem m_TurretSubsystem;
	private final HoodSubsystem m_HoodSubsystem;
	private final boolean left;
	
	public FeedAllianceCommand(TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, boolean left) {
		m_TurretSubsystem = turretSubsystem;
		m_HoodSubsystem = hoodSubsystem;
		this.left = left;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_TurretSubsystem, m_HoodSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_HoodSubsystem.setPos(Units.degreesToRadians(23));
		m_TurretSubsystem.setPos(Math.PI / 2 * (left ? -1 : 1));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
