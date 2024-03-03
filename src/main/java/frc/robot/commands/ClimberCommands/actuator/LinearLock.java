// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LinearLock extends Command {
	public void initialize() {
		Robot.climber.lock();
	}

	public boolean isFinished() {
		return true;
	}
}
