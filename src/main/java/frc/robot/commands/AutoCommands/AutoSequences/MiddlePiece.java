// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoPhotonAimCube;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.AutoTurn;
import frc.robot.commands.AutoCommands.DriveIntakeCube;
import frc.robot.commands.AutoCommands.Level;
import frc.robot.commands.AutoCommands.SlowAutoDrive;
import frc.robot.commands.AutoCommands.StopIntake;
import frc.robot.commands.AutoCommands.VisionDrive;
import frc.robot.commands.AutoCommands.Wait;
import frc.robot.commands.IntakeCommands.RotateLeft;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddlePiece extends SequentialCommandGroup {
	/** Creates a new MiddlePiece. */
	public MiddlePiece() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new AutoStallCone(),
				new RotateLeft(),
				new PivotArm(-46),
				new AutoExtend(28),
				new AutoCubeinConeOut(0.35),
				new StopIntake(),
				new ExtendArm(6),
				new PivotArm(-42),

				new SlowAutoDrive(-5.5),
				new AutoTurn(0),

				new ParallelCommandGroup(new SequentialCommandGroup(new Wait(0.6),
						new ParallelCommandGroup(
								new VisionDrive(RobotContainer.camera, RobotContainer.camera, -5, false, -1.6),
								new AutoCubeinConeOut(2))),
						new PivotArm(82)),

				new AutoTurn(0),
				new ParallelCommandGroup(new PivotArm(55), new SlowAutoDrive(3.6)),
				new Level()

		);

	}
}
