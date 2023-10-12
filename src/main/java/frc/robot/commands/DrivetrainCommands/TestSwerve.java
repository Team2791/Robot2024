// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TestSwerve extends CommandBase {
	public TestSwerve() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		addRequirements(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		Robot.drivetrain.setMotors(.1);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		if (Robot.drivetrain.getAveragePosition() > 0) {
			System.out.println("works");
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.setMotors(0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run

}
