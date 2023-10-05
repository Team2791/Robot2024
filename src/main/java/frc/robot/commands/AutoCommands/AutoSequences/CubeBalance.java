// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.PivotArm;
import frc.robot.commands.AutoCommands.AutoCubeOutConeIn;
import frc.robot.commands.AutoCommands.AutoCubeinConeOut;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoExtend;
import frc.robot.commands.AutoCommands.AutoStallCone;
import frc.robot.commands.AutoCommands.AutoStallCube;
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
public class CubeBalance extends SequentialCommandGroup {
  /** Creates a new CubeBalance. */
  public CubeBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
		new AutoStallCube(),
		new RotateLeft(),
		new PivotArm(-51),
		new AutoExtend(22),

		new AutoCubeOutConeIn(0.25),
		new StopIntake(),

		new AutoExtend(3),

		new SlowAutoDrive(-5.9),
		new PivotArm(46),
		new Wait(0.2),

		new SlowAutoDrive(3.4),
		new Level()


	);
  }
}
