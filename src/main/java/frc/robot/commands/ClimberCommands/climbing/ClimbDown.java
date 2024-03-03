// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class ClimbDown extends Command {
	public void initialize() {
		Robot.climber.setAll(-ClimberConstants.ClimbSpeedLow);
	}

	public void end(boolean interrupted) {
		Robot.climber.setAll(0.0);
	}

	public boolean isFinished() {
		return Robot.climber.leftPos() >= ClimberConstants.RelativeEncoderWhileUp;
	}
}
