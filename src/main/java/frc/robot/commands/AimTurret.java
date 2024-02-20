// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class AimTurret extends Command {
	private final PhotonCamera camera;
	private final Turret turret;

	/** Creates a new AprilTagDistance. */
	public AimTurret(PhotonCamera camera, Turret turret) {
		this.camera = camera;
		this.turret = turret;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		double range = PhotonUtils.calculateDistanceToTargetMeters(
		    Constants.Vision.CameraHeight,
		    Constants.Vision.TargetHeight,
		    Constants.Vision.CameraPitch,
		    Units.degreesToRadians(result.getBestTarget().getPitch())
		);

		double angle = Math.atan(Constants.Game.SpeakerHeight - Constants.Subsystem.ShooterHeight / range);
		this.turret.setAngle(angle);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
