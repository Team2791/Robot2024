// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PitstickCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LeftClimbUp extends Command {
	/** Creates a new LeftClimbUp. */
	public LeftClimbUp() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Robot.climber.setLeft(.5);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.climber.setLeft(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
