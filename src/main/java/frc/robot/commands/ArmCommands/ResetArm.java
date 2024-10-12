// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.subsystems.Arm;

public class ResetArm extends SequentialCommandGroup {
	public ResetArm(Arm arm) {
		addCommands(new Command() {
			boolean is = true;

			public void initialize() {
				if (arm.getAngle() > 0)
					is = false;
				else
					arm.setAngleTarget(20);
			}

			public boolean isFinished() {
				return !is || arm.atPivTarget();
			}
		}, new ExtendArm(arm, 0), new AngleArm(arm, 0));
	}
}
