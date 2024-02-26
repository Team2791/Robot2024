// Rotates until April tag is centered, and then stops the command
// Button : A

package frc.robot.commands.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AprilRotate extends Command {
	private final PhotonCamera camera;
	private final PIDController controller;
	private final Drivetrain drivetrain;

	private boolean done = false;

	/** Creates a new TagAllign. */
	public AprilRotate(PhotonCamera camera, Drivetrain drivetrain) {
		this.controller = new PIDController(
		    Constants.PID.Vision.Rotation.P,
		    Constants.PID.Vision.Rotation.I,
		    Constants.PID.Vision.Rotation.D
		);

		this.controller.setTolerance(30);
		this.controller.setSetpoint(320);

		this.camera = camera;
		this.drivetrain = drivetrain;
	}

	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) return;

		PhotonTrackedTarget target = result.getBestTarget();
		List<TargetCorner> corners = target.getDetectedCorners();

		double x = corners.parallelStream().mapToDouble(c -> c.x).sum() / 4;
		double power = controller.calculate(x);

		this.drivetrain.drive(0, 0, power, false, false);

		if (power == 0 || this.controller.atSetpoint()) {
			done = true;
		}
	}

	public void end(boolean interrupted) {
		this.drivetrain.stop();
	}

	public boolean isFinished() {
		return this.done;
	}
}
