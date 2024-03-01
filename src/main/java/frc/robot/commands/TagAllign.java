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
  double x;
  double setPoint = 400;
  public static double  power;
  double p,i,d,lastError = 0;
  /** Creates a new TagAllign. */
  public TagAllign(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.camera = camera;
    this.drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var result = camera.getLatestResult();


    boolean hasTargets = result.hasTargets();

    if(hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var res = camera.getLatestResult();

    if (res.hasTargets()) {
      var target = res.getBestTarget();
    
      // average as stream
      // 0 - 640
      x = target.getDetectedCorners().stream().mapToDouble((a) -> a.x).sum() / 4;
      SmartDashboard.putNumber("avg april tag position", x);
      SmartDashboard.putNumber("April tag yaw", target.getYaw());

      p = setPoint-x;
      i += p;
      d=p-lastError;
      lastError = p;
      power = .001*p + i*0.00001 + d*.0002;

      drivetrain.drive(0,0,power,false,false);
      if(power<.1 && Math.abs(p)<20){
        done = true;
      }

    }
    else drivetrain.drive(0,0,0,false,false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
