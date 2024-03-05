// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;

public class NoteAlign extends Command {
  PhotonCamera camera;
  boolean atSetpoint;
  PIDController pid;
  double output;
  int nonPIDTolerance = 3;
  /** Creates a new NoteAlign. */
  public NoteAlign(PhotonCamera camera) {
    addRequirements(Robot.m_drivetrain);
    this.camera = camera;
    pid = new PIDController(1, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(1);
		pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
		if (result != null && result.hasTargets()) {
			output = pid.calculate(result.getBestTarget().getYaw()); // -8
      
			Robot.m_drivetrain.drive(0, 0, output, false, false);
	    if(result.getBestTarget().getYaw() < nonPIDTolerance && result.getBestTarget().getYaw() > -nonPIDTolerance)
      	{
			Robot.m_drivetrain.stopModules();
      	}
	}

		if (result == null) {
			output = 0;
		}
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
