// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class PhotonEstimator extends SubsystemBase {

  PhotonCamera camera;
  DriveSubsystem drivetrain;


  private static final Vector<N3> stateStdDevs = VecBuilder.fill(1, 1, 1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.1, .1, .1);
  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

  private final Field2d field2d = new Field2d();
  private boolean sawTag = false;
  
  private double previousTimeStamp = 0;
  
  /** Creates a new PoseEstimator. */
  public PhotonEstimator(PhotonCamera camera, DriveSubsystem drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;
    
    //poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }


  public void setAlliance(Alliance alliance) { 
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
        originPosition = OriginPosition.kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getCurrentPose());
      Robot.m_drivetrain.m_odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), newPose);
    }
  }
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
  }


  @Override
  public void periodic() {

    var pipelineResult = camera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();


    if (resultTimestamp == previousTimeStamp && pipelineResult.hasTargets()) {
      previousTimeStamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialld = target.getFiducialId();  


      if (target.getPoseAmbiguity() <= .2 && fiducialld >= 0){
        var targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(fiducialld).get();

        // SmartDashboard.putNumber("April Tag X PoseEstimator", targetPose.getX());
        // SmartDashboard.putNumber("April Tag Y PoseEstimator", targetPose.getY());
        // SmartDashboard.putNumber("April Tag Z PoseEstimator", targetPose.getZ());
        // SmartDashboard.putNumber("April Tag ID PoseEstimator", fiducialld);

        // SmartDashboard.putString("April Tag Rotation PoseEstimator", targetPose.getRotation().toString());
        // SmartDashboard.putString("Translation PoseEstimator", targetPose.getTranslation().toString());
        // SmartDashboard.putString("Photon Pose estimator ", getCurrentPose().getTranslation().toString());

        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT);
        Robot.m_drivetrain.m_odometry.addVisionMeasurement(visionMeasurement.toPose2d(), previousTimeStamp,visionMeasurementStdDevs);

      }

    // This method will be called once per scheduler run
  }

  Robot.m_drivetrain.m_odometry.updateWithTime(resultTimestamp, drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());
  field2d.setRobotPose(getCurrentPose());
  SmartDashboard.putData(field2d);

  
}

  public Pose2d getCurrentPose(){
    return Robot.m_drivetrain.m_odometry.getEstimatedPosition();
  }

  


}