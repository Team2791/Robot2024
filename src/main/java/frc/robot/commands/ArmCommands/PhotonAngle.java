// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAngle extends Command {
	private final PhotonCamera camera;
  double distance;

	public PhotonAngle(PhotonCamera camera) {
		this.camera = camera;
	}

	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
      var targetID = target.getFiducialId();

      if(targetID==4 || targetID == 7){

      
			distance = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(9.451),Units.inchesToMeters(57.13),Constants.VisionConstants.kCameraPitch,Units.degreesToRadians(result.getBestTarget().getPitch())); // based on camera pitch

      		distance = PhotonUtils.getDistanceToPose(Robot.photonestimator.getCurrentPose(), AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().toPose2d()); //based on poseestimation


			Robot.arm.setAngle(53-Units.radiansToDegrees(Math.atan(Units.inchesToMeters(Constants.VisionConstants.kspeakerHeight)/distance)));
      }

		}
	}
}
