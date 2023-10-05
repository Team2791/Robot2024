// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.ChargeStationDriveBack;
import frc.robot.commands.AutoCommands.Level;
import frc.robot.commands.AutoCommands.SlowAutoDrive;
import frc.robot.commands.AutoCommands.StopIntake;
import frc.robot.commands.AutoCommands.TimedDrive;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.commands.IntakeCommands.RotateLeft;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleMobile extends SequentialCommandGroup {
	/** Creates a new MiddleMobile. */
	public MiddleMobile() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCone(),
				new RotateLeft(),
				new PivotArm(-46),
				new AutoExtend(28),
				new AutoCubeinConeOut(0.25),
				new AutoExtend(3),
				new StopIntake(),

				new SlowAutoDrive(-5.9),
				new PivotArm(45),
				new Wait(0.4),

				new SlowAutoDrive(3.6),
				new Level()
		);

	}
}
// new AutoStallCone(),
// new RotateLeft(),
// new PivotArm(-46),
// new AutoExtend(28),
// new AutoCubeinConeOut(0.35),

// new ParallelCommandGroup(new TimedDrive(3.6), // 3.3
// new SequentialCommandGroup(new ExtendArm(3), new Wait(1.9), new
// PivotArm(45))),

// new Wait(0.6),
// new ChargeStationDriveBack(),
// new Level()