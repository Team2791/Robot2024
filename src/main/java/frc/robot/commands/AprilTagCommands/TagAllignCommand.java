// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.w3c.dom.xpath.XPathNSResolver;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TagAllignCommand extends Command {

  PhotonCamera camera;
  DriveSubsystem drivetrain;

  double x;
  double setPoint = 400;
  XboxController controller;
  double  power;
  PIDController pid;

  /** Creates a new TagAllign. */
  public TagAllignCommand(PhotonCamera camera, DriveSubsystem driveSubsystem, XboxController drivController) {
    this.controller = drivController;
    this.camera = camera;
    this.drivetrain = driveSubsystem;
    this.pid = new PIDController(.001, 0.00001, .0002);
    addRequirements(driveSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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


      drivetrain.drive(0,0, pid.calculate(x, 320), false, false);

    }
    else drivetrain.drive(0,0,0, false, false);

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
