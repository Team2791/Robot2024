// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeExtensionPosition extends Command {
	public IntakeExtensionPosition() {}

	public void execute() {
		Robot.arm.extend(true);
	}

	public boolean isFinished() {
		return false;
	}
}
