//Should put rough distance on smartdashboard when Y is pressed.
//Smart Dashboard label : "Distance"

package frc.robot.commands.AprilTagCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AprilTagDistance extends Command {

  private PhotonCamera camera;
  private double range;
  /** Creates a new AprilTagDistance. */
  public AprilTagDistance(PhotonCamera camera) {
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) return;

		PhotonTrackedTarget target = result.getBestTarget();
    range = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.CAMERA_HEIGHT_METERS,1.32,Constants.RobotConstants.CAMERA_PITCH_RADIANS,Units.degreesToRadians(result.getBestTarget().getPitch()));
    SmartDashboard.putNumber("Distance", range);

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
