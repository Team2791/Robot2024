// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class AprilTagDistance extends Command {

  PhotonCamera camera;
  /** Creates a new AprilTagDistance. */
  public AprilTagDistance(PhotonCamera camera) {
    this.camera=camera;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.GameConstants.CAMERA_HEIGHT_METERS, Constants.GameConstants.TARGET_HEIGHT_METERS,Constants.GameConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    double angle = Math.atan(Constants.GameConstants.SpeakerHeight-Constants.GameConstants.ShooterHeight/range);
    Robot.turret.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
