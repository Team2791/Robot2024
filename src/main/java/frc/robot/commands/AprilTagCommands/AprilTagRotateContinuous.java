// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RGBLED;

public class AprilTagRotateContinuous extends Command {

  PhotonCamera camera;
  boolean done = false;

  private PIDController rotctl;
  /** Creates a new TagAllign. */
  public AprilTagRotateContinuous(PhotonCamera camera) {
    this.rotctl = new PIDController(Constants.RobotConstants.kATrotateP, Constants.RobotConstants.kATrotateI, Constants.RobotConstants.kATrotateD);
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

		double targetX = corners.parallelStream().mapToDouble(c -> c.x).sum() / 4;
		double rPower = rotctl.calculate(targetX);
    SmartDashboard.putData("Rotation PID controller", rotctl);

		Robot.drivetrain.drive(0, 0, rPower, false, false);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.done;
  }
}
