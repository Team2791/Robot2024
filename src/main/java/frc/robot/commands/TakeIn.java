// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TakeIn extends Command {
	private final Shooter shooter;
	private boolean done;

	public TakeIn(Shooter shooter) {
		this.shooter = shooter;
		this.done = false;

		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		this.shooter.setIntake(0.5);
	}

	@Override
	public void execute() {
		if (this.shooter.broken()) {
			this.done = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.shooter.setIntake(0);
	}

	@Override
	public boolean isFinished() {
		return this.done;
	}
}
