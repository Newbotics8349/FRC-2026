// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosCommand extends Command {
	/** Creates a new DriveToPosCommand. */
	private final SwerveSubsystem m_SwerveSubsystem;
	private PIDController pidMove = new PIDController(0.3, 0, 0);
	private PIDController pidRot = new PIDController(0.3, 0, 0);
	private Pose2d targetPose;
	private Supplier<Double> x;
	private Supplier<Double> y;
	private Supplier<Double> rot;

	public DriveToPosCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
		m_SwerveSubsystem = swerveSubsystem;

		this.x = x;
		this.y = y;
		this.rot = rot;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_SwerveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		targetPose = m_SwerveSubsystem.getOdometry().plus(new Transform2d(x.get() * -1, y.get() * -1, new Rotation2d(rot.get())));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Transform2d target = targetPose.minus(m_SwerveSubsystem.getOdometry()).times(-1);

		SmartDashboard.putString("Target", targetPose.toString());
		SmartDashboard.putString("Odo", m_SwerveSubsystem.getOdometry().toString());
		SmartDashboard.putString("Transform", target.toString());

		double pidx = pidMove.calculate(target.getX());
		double pidy = pidMove.calculate(target.getY());
		double pidrot = pidRot.calculate(target.getRotation().getRadians() * -1);

		m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(
			Math.abs(pidx) < 0.01 ? 0 : pidx,
			Math.abs(pidy) < 0.01 ? 0 : pidy,
			Math.abs(pidrot) < 0.003 ? 0 : pidrot
		));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_SwerveSubsystem.setModuleStates(new ChassisSpeeds(0, 0, 0));
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return targetPose.getX() < 0.1 && targetPose.getY() < 0.1 && targetPose.getRotation().getRadians() < 0.003;
	}
}
