// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DistanceCommand extends Command {
	PIDController pid = new PIDController(.01, 0, 0);
	

	public DistanceCommand() {
	}

	public void execute() {
		PhotonPipelineResult result = Robot.camera1.getLatestResult();

		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();

			double distance = PhotonUtils.calculateDistanceToTargetMeters(
				Units.inchesToMeters(9.451),
				Units.inchesToMeters(56.5),
				Constants.VisionConstants.kCameraPitch,
				Units.degreesToRadians(result.getBestTarget().getPitch())
			);

			//Robot.arm.setAngle(Math.atan(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(target.getFiducialId()).get().getZ())/distance);

			SmartDashboard.putNumber("(Photon) Distance to AprilTag (feet)", Units.metersToFeet(distance));
			SmartDashboard.putNumber("arm angle", 53-Math.atan(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(target.getFiducialId()).get().getZ())/distance);	
			//Robot.arm.armLeft.set(pid.calculate(Robot.arm.getArmPot(),53-Math.atan(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(target.getFiducialId()).get().getZ())/distance));
		}
	}
}
