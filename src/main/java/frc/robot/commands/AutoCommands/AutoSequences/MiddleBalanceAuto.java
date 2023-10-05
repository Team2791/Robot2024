// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.ChargeStationDrive;
import frc.robot.commands.AutoCommands.Level;
import frc.robot.commands.AutoCommands.StopIntake;
import frc.robot.commands.IntakeCommands.RotateLeft;

public class MiddleBalanceAuto extends SequentialCommandGroup {
	/** Creates a new BalanceAuto. */
	public MiddleBalanceAuto() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCone(),
				new RotateLeft(),
				new PivotArm(-46),
				new AutoExtend(28),
				new AutoCubeinConeOut(0.4),
				new StopIntake(),
				new ExtendArm(3),

				new ChargeStationDrive(),
				new Level());

	}

}
