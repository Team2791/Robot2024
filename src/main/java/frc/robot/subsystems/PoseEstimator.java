package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
	private final PhotonCamera camera;
	private final Drivetrain drivetrain;
	private final AHRS gyro;

	private final Vector<N3> statevec = VecBuilder.fill(1, 1, 1);
	private final Vector<N3> measurevec = VecBuilder.fill(.1, .1, .1);
	private OriginPosition origin = OriginPosition.kBlueAllianceWallRightSide;

	private final Field2d field2d = new Field2d();
	private boolean sawTag = false;

	private double previousTimeStamp = 0;

	SwerveDrivePoseEstimator poseEstimator;

	/** Creates a new PoseEstimator. */
	public PoseEstimator(PhotonCamera camera, Drivetrain drivetrain, AHRS gyro) {
		this.camera = camera;
		this.drivetrain = drivetrain;
		this.gyro = gyro;

		poseEstimator = new SwerveDrivePoseEstimator(
		    Constants.Drive.Kinematics,
		    gyro.getRotation2d(),
		    drivetrain.getModulePositions(),
		    new Pose2d(),
		    statevec,
		    measurevec
		);
	}

	public void setAlliance(Alliance alliance) {
		boolean allianceChanged = false;

		switch (alliance) {
			case Blue:
				allianceChanged = (origin == OriginPosition.kRedAllianceWallRightSide);
				origin = OriginPosition.kBlueAllianceWallRightSide;
				break;
			case Red:
				allianceChanged = (origin == OriginPosition.kBlueAllianceWallRightSide);
				origin = OriginPosition.kRedAllianceWallRightSide;
				break;
			default:
				// No valid alliance data. Nothing we can do about it
		}

		if (allianceChanged && sawTag) {
			// The alliance changed, which changes the coordinate system.
			// Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
			// needs to be transformed to the new coordinate system.
			var newPose = flipAlliance(getCurrentPose());
			poseEstimator.resetPosition(gyro.getRotation2d(), drivetrain.getModulePositions(), newPose);
		}
	}

	private Pose2d flipAlliance(Pose2d poseToFlip) {
		return poseToFlip.relativeTo(Constants.Vision.FlippingPose);
	}


	@Override
	public void periodic() {

		var pipelineResult = camera.getLatestResult();
		var resultTimestamp = pipelineResult.getTimestampSeconds();


		if (resultTimestamp == previousTimeStamp && pipelineResult.hasTargets()) {
			previousTimeStamp = resultTimestamp;
			var target = pipelineResult.getBestTarget();
			var fiducialld = target.getFiducialId();


			if (target.getPoseAmbiguity() <= 2 && fiducialld >= 0) {
				var targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(fiducialld).get();

				SmartDashboard.putNumber("April Tag X", targetPose.getX());
				SmartDashboard.putNumber("April Tag Y", targetPose.getY());
				SmartDashboard.putNumber("April Tag Z", targetPose.getZ());

				SmartDashboard.putString("April Tag Rotation", targetPose.getRotation().toString());

				Transform3d camToTarget = target.getBestCameraToTarget();
				Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

				Pose3d visionMeasurement = camPose.transformBy(Constants.Vision.CameraToRobot);
				poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), previousTimeStamp);
			}
		}

		poseEstimator.update(gyro.getRotation2d(), drivetrain.getModulePositions());
		field2d.setRobotPose(getCurrentPose());
	}

	public Pose2d getCurrentPose() {
		return poseEstimator.getEstimatedPosition();
	}
}
