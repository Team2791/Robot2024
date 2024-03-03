// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shoot extends Command {
	Timer timer = new Timer();

	/** Creates a new Shoot. */
	public Shoot() {
		timer.reset();
		addRequirements(Robot.shintake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.start();

		Robot.shintake.shoot(.8, .8);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {


		if (timer.get() > 3)
			Robot.shintake.feedToShooter();



	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		while (Robot.shintake.broken()) {
			Robot.shintake.feedToShooter();

		}
		Robot.shintake.shoot(0, 0);
		Robot.shintake.stopIntake();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Robot.shintake.broken();
	}
}
