// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveJoystickCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, rotSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
    /** Creates a new DriveJoystickCommand. */
    public DriveJoystickCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> rotSpeedFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.rotSpeedFunction = rotSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleOpMaxAcceleration);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleOpMaxAcceleration);
        this.rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleOpMaxAngularAcceleration);
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Joystick data
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double rotSpeed = rotSpeedFunction.get();

        // Deadband
        xSpeed = Math.abs(xSpeed) > Constants.ControllerConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.ControllerConstants.kDeadband ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > Constants.ControllerConstants.kDeadband ? rotSpeed : 0.0;

        // Driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleOpMaxMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleOpMaxMetersPerSecond;    
        rotSpeed = rotLimiter.calculate(rotSpeed) * Constants.DriveConstants.kTeleOpMaxAngularMetersPerSecond;

        
        // Make chassis speeds
        ChassisSpeeds chassisSpeeds;
        // if (!fieldOrientedFunction.get()) {
        //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());
        // } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        // }

        SmartDashboard.putNumber("x-chassis", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("y-chassis", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("rot-chassis", chassisSpeeds.omegaRadiansPerSecond);
        

        // Send to subsystem
        swerveSubsystem.setModuleStates(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
