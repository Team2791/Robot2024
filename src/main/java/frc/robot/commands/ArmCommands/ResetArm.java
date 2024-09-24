// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.subsystems.Arm;

public class ResetArm extends SequentialCommandGroup {
    public ResetArm(Arm arm) {
        if (arm.getAngle() < -0.5) addCommands(new AngleArm(arm, 5));
        addCommands(new ParallelCommandGroup(new AngleArm(arm, 0), new ExtendArm(arm, 0)));
    }
}
