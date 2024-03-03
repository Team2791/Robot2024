// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DistanceCommand extends Command {
	private final PhotonCamera camera;

	public DistanceCommand(PhotonCamera camera) {
		this.camera = camera;
	}

	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();

			double distance = PhotonUtils.calculateDistanceToTargetMeters(
				Units.inchesToMeters(7.2),
				Units.inchesToMeters(25),
				Math.toRadians(14),
				Math.toRadians(target.getPitch())
			);

			SmartDashboard.putNumber("(Photon) Distance to AprilTag (feet)", Units.metersToFeet(distance));
		}
	}
}
