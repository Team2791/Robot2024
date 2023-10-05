// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ManualPivotDown extends CommandBase {
	/** Creates a new ManualPivotBack. */
	private Timer timer;

	public ManualPivotDown() {
		// Use addRequirements() here to declare subsystem dependencies.
		timer = new Timer();
		addRequirements(Robot.arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.start();
		Robot.arm.setPivotbrake(false);
		Constants.ManualPivot = true;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (timer.get() > 0.2)
			Robot.arm.pivotArm(-Constants.ManualPivotSpeed * Constants.Jonah);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.arm.pivotArm(0);
		Robot.arm.setPivotbrake(true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
