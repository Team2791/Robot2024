// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;

public class NoteAlign extends Command {
	private final PIDController alignctl = new PIDController(.01, 0, 0);
	private final int tolerance = 3;

	/** Creates a new NoteAlign. */
	public NoteAlign() {
		addRequirements(Robot.drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		alignctl.setTolerance(1);
		alignctl.setSetpoint(300);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		PhotonPipelineResult result = Robot.camera1.getLatestResult();		
		PhotonTrackedTarget target = result.getBestTarget();

		if (result == null || target == null || !result.hasTargets()) return;

		if (target.getYaw() < tolerance && target.getYaw() > -tolerance) {
			Robot.drivetrain.stopModules();
			return;
		}

		double output = alignctl.calculate(target.getYaw());
		Robot.drivetrain.drive(0, -output, 0, false, false);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.stopModules();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return alignctl.atSetpoint();
	}
}
