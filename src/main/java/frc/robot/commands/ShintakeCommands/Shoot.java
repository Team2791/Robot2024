// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.UtilCommands.Wait;

public class Shoot extends SequentialCommandGroup {
	Timer timer = new Timer();

	public Shoot() {
		addRequirements(Robot.shintake);

		addCommands(new Command() {
			public void initialize() {
				Robot.shintake.shoot(.8, .8);
			}

			public boolean isFinished() {
				return true;
			}
		});

		addCommands(new Wait(3));

		addCommands(new Command() {
			public void initialize() {
				Robot.shintake.feed();
			}

			public void end(boolean interrupted) {
				Robot.shintake.stopIntake();
				Robot.shintake.shoot(0, 0);
			}

			public boolean isFinished() {
				return !Robot.shintake.broken();
			}
		});
	}
}
