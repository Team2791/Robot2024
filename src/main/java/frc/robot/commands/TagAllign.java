// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TagAllign extends Command {

  PhotonCamera camera;
  DriveSubsystem drivetrain;
  double yaw;
  double pitch;
  double area;
  double skew;
  /** Creates a new TagAllign. */
  public TagAllign(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.camera = camera;
    this.drivetrain = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var result = camera.getLatestResult();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if(hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
    }

    drivetrain.drive(.1,.1,yaw,false,false);

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
