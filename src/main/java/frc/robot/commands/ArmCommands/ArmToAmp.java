// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.constants.ArmConstants;
import frc.robotkt.subsystems.Arm;

public class ArmToAmp extends SequentialCommandGroup {
    public ArmToAmp(Arm arm) {
        addCommands(new AngleArm(arm, ArmConstants.kAmpAngle), new ExtendArm(arm, 100));
    }
}
