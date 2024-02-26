package frc.robot.commands.Arm;

import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Led.LedMode;

public class TurretAngle extends Command {
	private final Arm arm;
	private final Led led;
	private final PhotonCamera camera;

	boolean done = false;

	public TurretAngle(PhotonCamera camera, Arm arm, Led led) {
		this.camera = camera;
		this.arm = arm;
		this.led = led;
	}

	public void initialize() {
		this.led.set(LedMode.init().base(Color.kGreen));
	}

	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (!result.hasTargets()) this.led.set(LedMode.init().base(Color.kRed));

		PhotonTrackedTarget target = result.getBestTarget();
		int id = target.getFiducialId();

		if (!Set.of(4, 5, 6, 7).contains(id)) {
			this.led.set(LedMode.init().base(Color.kOrange));
			return;
		}

		double distance = PhotonUtils.calculateDistanceToTargetMeters(
		    Constants.Vision.CameraHeight,
		    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(id).get().getZ(),
		    Constants.Vision.CameraPitch,
		    Units.degreesToRadians(result.getBestTarget().getPitch())
		);

		double angle = Math.atan(Constants.Arm.SpeakerHeight - Constants.Arm.ShooterHeight / distance);

		this.arm.setAngle(angle);
		this.led.set(LedMode.init().base(Color.kRed));

		done = true;
	}

	public boolean isFinished() {
		return done;
	}
}
