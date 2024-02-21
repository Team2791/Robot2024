// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TakeIn extends Command {
	private final Intake intake;

	public TakeIn(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void initialize() {
		this.intake.takeIn();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (this.intake.amps() > 5) {
		}

		// TODO: figure out how to end correctly
	}


	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.intake.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
