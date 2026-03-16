// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
	/** Creates a new VisionSubsystem. */
	private final PhotonCamera camera = new PhotonCamera("Main Camera");
	private PhotonPipelineResult result;
	private LEDSubsystem m_LedSubsystem;

	public VisionSubsystem(LEDSubsystem ledSubsystem) {
		m_LedSubsystem = ledSubsystem;
	}

	public boolean hasTargets() {
		if (result == null) return false;
		return result.hasTargets();
	}

	public List<PhotonTrackedTarget> getTargets() {
		if (result == null) return null;
		if (result.hasTargets()) {
			return result.getTargets();
		}
		return new ArrayList<>();
	}

	public PhotonTrackedTarget getSpecificTarget(int id) {
		if (getTargets() == null) return null;
		for (PhotonTrackedTarget target : getTargets()) {
			if (target.getFiducialId() == id) return target;
		}
		return null;
	}

	public Transform3d getTransformToTarget(int id) {
		PhotonTrackedTarget target = getSpecificTarget(id);
		if (target != null) {
			Transform3d distance = target.bestCameraToTarget;
			return distance;
		}
		return null;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
		if (unread.size() > 0) result = unread.get(unread.size() - 1);
		// CommandScheduler.getInstance().schedule(m_LedSubsystem.setColour(hasTargets() ? 0 : 255, hasTargets() ? 255 : 0, 0));
	}
}
