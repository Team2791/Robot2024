// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {

  PhotonCamera camera;
  DriveSubsystem drivetrain;


  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Field2d field2d = new Field2d();
  
  private double previousTimeStamp = 0;

  SwerveDrivePoseEstimator poseEstimator;
  
  /** Creates a new PoseEstimator. */
  public PoseEstimator(PhotonCamera camera, DriveSubsystem drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;
    
    poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {

    var pipelineResult = camera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp == previousTimeStamp && pipelineResult.hasTargets()) {
      previousTimeStamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialld = target.getFiducialId();  


      if (target.getPoseAmbiguity() <= 2 && fiducialld >= 0){
        var targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(fiducialld).get();

        SmartDashboard.putNumber("April Tag X", targetPose.getX());
        SmartDashboard.putNumber("April Tag Y", targetPose.getY());
        SmartDashboard.putNumber("April Tag Z", targetPose.getZ());

        SmartDashboard.putString("April Tag Rotation", targetPose.getRotation().toString());
        
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), previousTimeStamp);

      }

    // This method will be called once per scheduler run
  }

  poseEstimator.update(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());
  field2d.setRobotPose(getCurrentPose());

  SmartDashboard.putString("Position", getCurrentPose().toString());
  
}

  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }


}
