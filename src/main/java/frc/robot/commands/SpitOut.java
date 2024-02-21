// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SpitOut extends Command {
	private final Intake intake;

	public SpitOut(Intake intake) {
		this.intake = intake;
	}

	public void execute() {
		this.intake.spitOut();
	}

	public void end(boolean interrupted) {
		this.intake.stop();
	}

	public boolean isFinished() {
		return true;
	}
}
