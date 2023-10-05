// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TimedDrive extends CommandBase {
	/** Creates a new TimedDrive. */
	Timer timer = new Timer();
	private double time;

	public TimedDrive(double t) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);
		time = t;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.start();
		Robot.drivetrain.arcadeDrive(-0.53, 0);// 0.58

		// Driving toward battery side
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Robot.drivetrain.setMotors(0.29, 0.29);
		Robot.drivetrain.arcadeDrive(-0.53, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.arcadeDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > time;
	}
}
