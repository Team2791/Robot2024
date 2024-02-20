// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
	private final double speed;
	private final Shooter shooter;

	/** Creates a new Shoot. */
	public Shoot(Shooter shooter, double speed) {
		this.shooter = shooter;
		this.speed = speed;
	}

	public Shoot(Shooter shooter) {
		this(shooter, 1);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.shooter.setShooter(speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.shooter.setShooter(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
