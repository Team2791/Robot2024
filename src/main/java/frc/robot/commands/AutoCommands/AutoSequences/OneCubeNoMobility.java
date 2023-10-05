// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import frc.robot.commands.AutoCommands.AutoStallCube;
import frc.robot.commands.IntakeCommands.RotateLeft;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeOutConeIn;
import frc.robot.commands.AutoCommands.Wait;

public class OneCubeNoMobility extends SequentialCommandGroup {
	/** Creates a new OneCubeNoMobility. */
	public OneCubeNoMobility() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCube(),
				new RotateLeft(),
				new PivotArm(-48),
				new ExtendArm(19),
				new AutoCubeOutConeIn(0),
				new Wait(0.4),
				new PivotArm(0)

		);
	}
}
