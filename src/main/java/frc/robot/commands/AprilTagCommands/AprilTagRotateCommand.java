//Rotates until April tag is centered, and then stops the command
//Button : A

package frc.robot.commands.AprilTagCommands;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RGBLED;

public class AprilTagRotateCommand extends Command {

  PhotonCamera camera;
  boolean done = false;
  double setPoint=320;

  private PIDController rotctl;
  /** Creates a new TagAllign. */
  public AprilTagRotateCommand(PhotonCamera camera) {
    this.rotctl = new PIDController(Constants.AprilTagCommandsConstants.kATrotateP, Constants.AprilTagCommandsConstants.kATrotateI, Constants.AprilTagCommandsConstants.kATrotateD);
		this.rotctl.setTolerance(10);
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Robot.led.setColor(255,0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) return;

		PhotonTrackedTarget target = result.getBestTarget();
		List<TargetCorner> corners = target.getDetectedCorners();

		double targetX = corners.parallelStream().mapToDouble(c -> c.x).sum() / 4;
		double rPower = rotctl.calculate(targetX, setPoint);

		Robot.m_robotDrive.drive(0, 0, rPower, false, false, false);

    if(rotctl.atSetpoint()){
      done=true;
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Robot.led.setColor(0,255,0);
    Robot.m_robotDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.done;
  }
}
