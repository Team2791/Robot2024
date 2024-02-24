// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretAngle extends Command {

  PhotonCamera camera;
  double range;
  double angle;
  public static int targetID;
  boolean angleSet = false;
  /** Creates a new AprilTagDistance. */
  public TurretAngle(PhotonCamera camera) {
    this.camera=camera;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.led.setColor(0,0,255);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    if (!result.hasTargets())Robot.led.setColor(255,0,0);
    
    

    PhotonTrackedTarget target = result.getBestTarget();
    targetID = target.getFiducialId();


    switch(targetID){
      case 4:
        range = PhotonUtils.getDistanceToPose(Robot.poseEstimator.getCurrentPose(), AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().toPose2d());

        range = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.CAMERA_HEIGHT_METERS,AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().getZ(),Constants.RobotConstants.CAMERA_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()));
        angle = Math.atan(Constants.RobotConstants.SpeakerHeight-Constants.RobotConstants.ShooterHeight/range);
        Robot.turret.setAngle(angle);
        Robot.led.setColor(0,255,0);
        angleSet = true;
      break;
      case 5:
        range = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.CAMERA_HEIGHT_METERS,AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().getZ(),Constants.RobotConstants.CAMERA_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()));
        angle = Math.atan(Constants.RobotConstants.SpeakerHeight-Constants.RobotConstants.ShooterHeight/range);

        Robot.turret.setAngle(angle);
        Robot.led.setColor(0,255,0);
        angleSet = true;
      case 6:
        range = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.CAMERA_HEIGHT_METERS,AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().getZ(),Constants.RobotConstants.CAMERA_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()));
        angle = Math.atan(Constants.RobotConstants.SpeakerHeight-Constants.RobotConstants.ShooterHeight/range);

        Robot.turret.setAngle(angle);
        Robot.led.setColor(0,255,0);
        angleSet = true;
      case 7:
        range = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.CAMERA_HEIGHT_METERS,AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(targetID).get().getZ(),Constants.RobotConstants.CAMERA_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()));
        angle = Math.atan(Constants.RobotConstants.SpeakerHeight-Constants.RobotConstants.ShooterHeight/range);

        Robot.turret.setAngle(angle);
        Robot.led.setColor(0,255,0);
        angleSet = true;
      default :
        Robot.led.setColor(255,255,0);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleSet;
  }
}
