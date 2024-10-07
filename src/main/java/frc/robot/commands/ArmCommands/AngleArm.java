// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robotkt.subsystems.Arm;

public class AngleArm extends Command {
    final Arm arm;
    final double angle;

    public AngleArm(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;

        addRequirements(arm);
    }

    public void initialize() {
        arm.setAngleTarget(angle);
    }

    public boolean isFinished() {
        return arm.atPivTarget();
    }
}
