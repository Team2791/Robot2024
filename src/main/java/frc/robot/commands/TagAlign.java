// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TagAlign extends Command {
	PhotonCamera camera;
	DriveSubsystem drivetrain;
	PIDController pid = new PIDController(
	    VisionConstants.kP, VisionConstants.kI, VisionConstants.kD
	);
	AHRS gyro;

	public TagAlign(PhotonCamera camera, DriveSubsystem drivetrain) {
		this.gyro = DriveSubsystem.m_gyro;
		this.camera = camera;
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		this.pid.setSetpoint(VisionConstants.RES_CENTER);
		this.pid.setTolerance(VisionConstants.kTolerance);
	}

	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets())
		    return;

		PhotonTrackedTarget target = result.getBestTarget();

		// range 0 - 640
		double average_x = target.getDetectedCorners()
		    .stream()
		    .mapToDouble((a) -> a.x)
		    .sum() / 4;

		double power = this.pid.calculate(average_x);
		drivetrain.drive(0, 0, power, false, false);

		SmartDashboard.putNumber("April Tag Position", average_x);
		SmartDashboard.putNumber("Power", power);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stopModules();
	}
}