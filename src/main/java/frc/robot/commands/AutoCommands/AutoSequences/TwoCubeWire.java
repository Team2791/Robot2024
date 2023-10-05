// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeOutConeIn;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoPhotonAimCube;
import frc.robot.commands.AutoCommands.AutoPhotonAimTag;
import frc.robot.commands.AutoCommands.AutoStallCube;
import frc.robot.commands.AutoCommands.AutoTurn;
import frc.robot.commands.AutoCommands.DriveIntakeCube;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.commands.IntakeCommands.RotateLeft;
import frc.robot.commands.AutoCommands.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeWire extends SequentialCommandGroup {
	/** Creates a new TwoCubeWire. */
	public TwoCubeWire() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCube(),
				new RotateLeft(),
				new PivotArm(-48),
				new ExtendArm(19),
				new AutoCubeOutConeIn(0),
				new Wait(0.4),
				new StopIntake(),

				new ParallelCommandGroup(new SequentialCommandGroup(new ExtendArm(6), new PivotArm(30)),
						new AutoDrive(-4.6)),
				new AutoPhotonAimCube(RobotContainer.camera, RobotContainer.camera, -5, false),
				new PivotArm(82),
				new DriveIntakeCube(-1.6),
				new AutoTurn(0),

				new ParallelCommandGroup(new PivotArm(-55), new AutoDrive(2.6)), // 4.6
				new AutoDrive(2.7),
				new AutoTurn(0),
				new AutoPhotonAimTag(RobotContainer.camera2, RobotContainer.camera2, -13, true),
				new AutoDrive(1.7),
				new Wait(0.2),
				new AutoCubeOutConeIn(0.3),
				new StopIntake()

		);
	}
}
