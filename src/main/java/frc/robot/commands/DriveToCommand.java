// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCommand extends Command {
	/** Creates a new DriveToCommand. */
	private final SwerveSubsystem m_SwerveSubsystem;
	private final VisionSubsystem m_VisionSubsystem;
	private Transform3d target;
	private Supplier<Constants.AprilTag> tag;
	private PIDController pidMove = new PIDController(0.3, 0, 0);
	private PIDController pidRot = new PIDController(0.3, 0, 0);
	private Pose2d targetPose = null;


	public DriveToCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, Supplier<Constants.AprilTag> tag) {
		m_SwerveSubsystem = swerveSubsystem;
		m_VisionSubsystem = visionSubsystem;
		this.tag = tag;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_SwerveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Transform3d aprilTag = m_VisionSubsystem.getTransformToTarget(tag.get().id);

		if (aprilTag != null) {
			target = aprilTag.plus(tag.get().offset);

			Transform2d t2D = new Transform2d(target.getTranslation().toTranslation2d(), target.getRotation().toRotation2d());
			targetPose = m_SwerveSubsystem.getOdometry().plus(t2D.times(-1));

			double pidx = pidMove.calculate(target.getX());
			double pidy = pidMove.calculate(target.getY());
			double pidrot = pidRot.calculate(target.getRotation().getZ());

			m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(
				Math.abs(pidx) < 0.01 ? 0 : pidx, 
				Math.abs(pidy) < 0.01 ? 0 : pidy, 
				Math.abs(pidrot) < 0.003 ? 0 : pidrot));
		} else if (targetPose != null) {
			Transform2d target = targetPose.minus(m_SwerveSubsystem.getOdometry()).times(-1);

			double pidx = pidMove.calculate(target.getX());
			double pidy = pidMove.calculate(target.getY());
			double pidrot = pidRot.calculate(target.getRotation().getRadians() * -1);

			m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(
				Math.abs(pidx) < 0.01 ? 0 : pidx, 
				Math.abs(pidy) < 0.01 ? 0 : pidy, 
				Math.abs(pidrot) < 0.003 ? 0 : pidrot));
		} else {
			m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(0, 0, 0));
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(0, 0, 0));
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
