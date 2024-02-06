// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.DriveSubsystem;

public class TagAllign extends Command {

  PhotonCamera camera;
  DriveSubsystem drivetrain;
  TargetCorner yaw;
  double pitch;
  double area;
  double skew;
  boolean done = false;
  AHRS gyro;
  /** Creates a new TagAllign. */
  public TagAllign(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.gyro = DriveSubsystem.m_gyro;
    this.camera = camera;
    this.drivetrain = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var result = camera.getLatestResult();


    boolean hasTargets = result.hasTargets();

    if(hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getDetectedCorners().get(0);
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var res = camera.getLatestResult();

    if (res.hasTargets()) {
      var target = res.getBestTarget();
      double c_y = target.getDetectedCorners().get(0).y;

      if (c_y < 0.2) {
        drivetrain.drive(
          0.5,0.5,.5,true,false);
      } else if (c_y > 0.2) {
        drivetrain.drive(-0.5,-0.5,-.5,true,false);
      } else {
        drivetrain.stopModules();
        done = true;
      }
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
