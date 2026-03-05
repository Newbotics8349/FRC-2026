// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.Elastic;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private String gamedata = "";
	private String alliance;
	private double time;

	private final RobotContainer m_robotContainer;

	public Robot() {
		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
		Elastic.selectTab("Autonomous");
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
	}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		alliance = DriverStation.getAlliance().get().toString().substring(0, 1);

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		Elastic.selectTab("Teleoperated");
	}

	@Override
	public void teleopPeriodic() {
		time = DriverStation.getMatchTime();
		SmartDashboard.putNumber("Match Time", time);
		
		if (gamedata == "") {
			gamedata = DriverStation.getGameSpecificMessage();
		} else {
			SmartDashboard.putBoolean(
				"Can Score", 
				time > 130
				|| time <= 30
				|| (130 >= time && time > 105 && !alliance.equals(gamedata))
				|| (105 >= time && time > 80 && alliance.equals(gamedata))
				|| (80 >= time && time > 55 && !alliance.equals(gamedata))
				|| (55 >= time && time > 30 && alliance.equals(gamedata))
			);
		}
	}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
