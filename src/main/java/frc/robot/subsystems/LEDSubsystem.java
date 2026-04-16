// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  	/** Creates a new LEDSubsystem. */
	private final AddressableLED m_led;

	public LEDSubsystem() {
		m_led = new AddressableLED(Constants.PWMConstants.kLedPort);

		m_led.setLength(Constants.numLeds);
		m_led.start();
	}

	public Command setColour(int red, int green, int blue) {
		return runOnce(() -> {
			AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.numLeds);
			LEDPattern pattern = LEDPattern.solid(new Color(red, green, blue));

			pattern.applyTo(buffer);

			m_led.setData(buffer);
		});
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
