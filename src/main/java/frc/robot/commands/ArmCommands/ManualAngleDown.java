package frc.robot.commands.ArmCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ManualAngleDown extends Command {
	public ManualAngleDown() {}

	public void execute() {
		Robot.arm.pivot(false);
	}

	public boolean isFinished() {
		return false;
	}
}
