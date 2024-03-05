// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

public class FullExtension extends Command {
	public FullExtension() {}

	public void execute() {
		Robot.arm.extend(false);
	}

	public void end(boolean interrupted) {}

	public boolean isFinished() {
		return Robot.arm.getPivotPoint() >= ArmConstants.kMaxExt;
	}
}
