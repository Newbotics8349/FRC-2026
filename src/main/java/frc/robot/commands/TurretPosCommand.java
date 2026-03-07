// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import util.Vector3;
import util.ShootAndMoveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretPosCommand extends Command {
	/** Creates a new TurretPosCommand. */
	private final VisionSubsystem m_VisionSubsystem;
	private final TurretSubsystem m_TurretSubsystem;
	private final HoodSubsystem m_HoodSubsystem;
	private Supplier<ChassisSpeeds> robotVelocity;

	public TurretPosCommand(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem,
							Supplier<ChassisSpeeds> robotVelocity
	) {
		m_VisionSubsystem = visionSubsystem;
		m_TurretSubsystem = turretSubsystem;
		m_HoodSubsystem = hoodSubsystem;
		this.robotVelocity = robotVelocity;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_VisionSubsystem, m_TurretSubsystem, m_HoodSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// TODO figure out targets
		Transform3d pos = m_VisionSubsystem.getTransformToTarget(0);
		ChassisSpeeds rv = robotVelocity.get();
		ShootAndMoveMath m = new ShootAndMoveMath(
			new Vector3(Units.metersToInches(pos.getX()), Units.metersToInches(pos.getY()), 0), 
			new Vector3(Units.metersToInches(rv.vxMetersPerSecond), Units.metersToInches(rv.vyMetersPerSecond), 0)
		);

		m_TurretSubsystem.setPos(m.rotToTarget());
		m_HoodSubsystem.setPos(m.heightToTarget());
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
