// Should put rough distance on smartdashboard when Y is pressed.
// Smart Dashboard label : "Distance"

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** Gets data from Photon and puts it onto SmartDashboard */
public class AprilTelemetry extends Command {
	private final PhotonCamera camera;

	public AprilTelemetry(PhotonCamera camera) {
		this.camera = camera;
	}

	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) return;
		PhotonTrackedTarget target = result.getBestTarget();

		double x = target.getDetectedCorners().stream().mapToDouble(a -> a.x).sum() / 4;
		double distance = PhotonUtils.calculateDistanceToTargetMeters(
		    Constants.Vision.CameraHeight,
		    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(target.getFiducialId()).get().getZ(),
		    Constants.Vision.CameraPitch,
		    Units.degreesToRadians(result.getBestTarget().getPitch())
		);

		SmartDashboard.putNumber("(Photon) Latest Target Distance", distance);
		SmartDashboard.putNumber("(Photon) Latest Target Yaw", target.getYaw());
		SmartDashboard.putNumber("(Photon) Latest Target X-Coordinate", x);
	}

	public boolean isFinished() {
		return false;
	}
}
