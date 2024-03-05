// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Fullextension extends Command {
	Timer timer = new Timer();

	/** Creates a new Fullextension. */
	public Fullextension() {

		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		timer.start();



	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		while (Robot.arm.getArmPot() < 95) {
			Robot.arm.manualExtend();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.arm.stopExtension();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Robot.arm.getArmPot() > 85 || timer.get() > 20;
	}
}
