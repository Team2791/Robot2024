// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeOutConeIn;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoPhotonAimCube;
import frc.robot.commands.AutoCommands.AutoPhotonAimTag;
import frc.robot.commands.AutoCommands.AutoSpitCone;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.AutoStallCube;
import frc.robot.commands.AutoCommands.AutoTurn;
import frc.robot.commands.AutoCommands.DriveIntakeCone;
import frc.robot.commands.AutoCommands.DriveIntakeCube;
import frc.robot.commands.AutoCommands.FastAutoDrive;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.commands.IntakeCommands.RotateLeft;
import frc.robot.commands.AutoCommands.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeCubeWire extends SequentialCommandGroup {
	/** Creates a new ConeCube. */
	public ConeCubeWire() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCone(),
				new RotateLeft(),
				new PivotArm(-46),
				new AutoExtend(28),
				new AutoSpitCone(),
				new Wait(0.25),
				new StopIntake(),
				new AutoExtend(2),
				new PivotArm(-60),
				new AutoDrive(-4.2),

				new AutoPhotonAimCube(RobotContainer.camera, RobotContainer.camera, -5, false),
				new ParallelCommandGroup(new SequentialCommandGroup(new Wait(0.66), new DriveIntakeCube(-2.5)), // -2.2
						new PivotArm(84)),
				new AutoTurn(0),

				new ParallelCommandGroup(new PivotArm(-51), new FastAutoDrive(2.9)),
				new AutoDrive(2.7),
				new Wait(0.15),
				new AutoPhotonAimTag(RobotContainer.camera2, RobotContainer.camera2, 14, true),
				new ParallelCommandGroup(new AutoDrive(1.8),
						new ExtendArm(24)),

				new AutoCubeOutConeIn(0),
				new Wait(0.25),
				new ParallelCommandGroup(new ExtendArm(5), new PivotArm(0)),
				new FastAutoDrive(-5)

		);
	}
}
