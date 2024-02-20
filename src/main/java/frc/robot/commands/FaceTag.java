// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class FaceTag extends Command {
	private final PhotonCamera camera;
	private final Drivetrain drivetrain;
	private final boolean forever;

	double x;
	boolean done = false;

	private final PIDController rotctl;
	private final double kP = 0.5;
	private final double kI = 0;
	private final double kD = 0;

	/**
	 * Creates a new TagAllign.
	 */
	public FaceTag(PhotonCamera camera, Drivetrain drivetrain, boolean forever) {
		this.camera = camera;
		this.drivetrain = drivetrain;
		this.rotctl = new PIDController(kP, kI, kD);
		this.forever = forever;

		this.rotctl.setSetpoint(320);
		this.rotctl.setTolerance(10);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) return;

		PhotonTrackedTarget target = result.getBestTarget();
		List<TargetCorner> corners = target.getDetectedCorners();

		double targetX = corners.parallelStream().mapToDouble(c -> c.x).sum() / 4;
		double rPower = rotctl.calculate(targetX);

		drivetrain.drive(0, 0, rPower, false, false);
		done = rPower < 0.1;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.done && !this.forever;
	}
}
