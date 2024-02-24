//Should theoretically translate left/right to center the april tag on camera
//Will need to tune PID.
//Button X



package frc.robot.commands.AprilTagCommands;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagTranslate extends Command {

  PhotonCamera camera;
  DriveSubsystem drivetrain;
  boolean done = false;
  double setPoint = 320;

  private PIDController rotctl;
  /** Creates a new TagAllign. */
  public AprilTagTranslate(PhotonCamera camera) {
    this.rotctl = new PIDController(Constants.RobotConstants.kATtranslateP, Constants.RobotConstants.kATtranslateI, Constants.RobotConstants.kATtranslateD);
    this.rotctl.setSetpoint(320);
		this.rotctl.setTolerance(10);
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
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
		List<TargetCorner> corners = target.getDetectedCorners();

    //Pose2d pose = new Pose2d(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY(), target.getBestCameraToTarget().getRotation().toRotation2d());

		double targetX = corners.parallelStream().mapToDouble(c -> c.x).sum() / 4;
		double rPower = rotctl.calculate(targetX, setPoint);
    SmartDashboard.putData("Translation PID controller", rotctl);

		Robot.drivetrain.drive(rPower, 0, 0, false, false);
    

    if(rotctl.atSetpoint()){
      done = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.done;
  }
}
