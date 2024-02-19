// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagRotateContinuous extends Command {

  PhotonCamera camera;
  DriveSubsystem drivetrain;
  double pitch;
  double area;
  double skew;
  AHRS gyro;
  double x;
  boolean done = false;
  double setPoint;
  double p;
  double i;
  double d;
  /** Creates a new TagAllign. */
  public AprilTagRotateContinuous(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.gyro = DriveSubsystem.m_gyro;
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
      TargetCorner yaw1 = target.getDetectedCorners().get(0);
      TargetCorner yaw2 = target.getDetectedCorners().get(1);
      x = yaw1.x + yaw2.x;
    }

    //setpoint = resolution/2
    
    setPoint = 640/2;
    p=x-setPoint;
    drivetrain.drive(0,0,p*.5,true,true);
    

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
