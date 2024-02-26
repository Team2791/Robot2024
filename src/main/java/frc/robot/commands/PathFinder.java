// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;


public class PathFinder extends Command {
  Pose2d p;
  Command pathFindingCommand;
  /** Creates a new PathFinder. */
  public PathFinder(Pose2d pose) {
    this.p = pose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d targetPose = new Pose2d(p.getX(), p.getY(), p.getRotation());

    PathConstraints constraints = new PathConstraints(3.0, 1.5,Units.degreesToRadians(360), Units.degreesToRadians(180));
    pathFindingCommand = AutoBuilder.pathfindToPose(targetPose,constraints,0.0, 1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathFindingCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathFindingCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}