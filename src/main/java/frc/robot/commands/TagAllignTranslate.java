// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TagAllignTranslate extends Command {

	PhotonCamera camera;
	PIDController pid = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
  double setpoint = 320;

	AHRS gyro;

	public TagAllignTranslate(PhotonCamera camera) {
		this.gyro = DriveSubsystem.m_gyro;
		this.camera = camera;
	}

	@Override
	public void initialize() {
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

		double power = this.pid.calculate(average_x, setpoint);
		Robot.m_robotDrive.drive(power, 0, 0, false, false);

		SmartDashboard.putNumber("April Tag Position", average_x);
		SmartDashboard.putNumber("Power", power);
	}


	@Override
	public void end(boolean interrupted) {
		Robot.m_robotDrive.stopModules();
	}
}
