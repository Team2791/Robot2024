// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class AprilTagTranslate extends Command {
  PIDController pid = new PIDController(.01, .01, .01);
  double setPoint;

  PhotonCamera camera;
  PhotonTrackedTarget target;
  /** Creates a new AprilTagAngleDiff. */
  public AprilTagTranslate(PhotonCamera camera) {
    this.camera = camera;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = 640/2;
    var result = camera.getLatestResult();
    target = result.getBestTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    target = result.getBestTarget();




    double average_x = target.getDetectedCorners()
    .stream()
    .mapToDouble((a) -> a.x)
    .sum() / 4;

    double power = this.pid.calculate(average_x);
		Robot.drivetrain.drive(power, 0, 0, false, false);

		SmartDashboard.putNumber("April Tag Position", average_x);
		SmartDashboard.putNumber("Power", power);



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
