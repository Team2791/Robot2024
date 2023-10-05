// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.commands.ArmCommands.PivotArm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.IntakeCommands.RotateLeft;
import frc.robot.commands.AutoCommands.AutoSpitCone;

public class OneConeNoMobility extends SequentialCommandGroup {
	/** Creates a new OneConeNoMobility. */
	public OneConeNoMobility() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCone(),
				new RotateLeft(),
				new PivotArm(-46),
				new AutoExtend(28),
				new AutoSpitCone(),
				new Wait(0.4),
				new ExtendArm(5)

		);
	}
}
