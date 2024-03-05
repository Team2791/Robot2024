package frc.robot.commands.ArmCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ManualAngleUp extends Command {
	public ManualAngleUp() {}

	public void execute() {
		Robot.arm.pivot(true);
	}

	public boolean isFinished() {
		return false;
	}
}
