// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.subsystems.Arm;

class BringUp extends Command {
    final Arm arm;
    boolean set = true;

    public BringUp(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (arm.getAngle() < 0) set = false;
        else arm.setAngleTarget(20);
    }

    @Override
    public boolean isFinished() {
        return !set || arm.atPivTarget();
    }
}

public class ResetArm extends SequentialCommandGroup {
    public ResetArm(Arm arm) {
        addCommands(new BringUp(arm), new ExtendArm(arm, 0), new AngleArm(arm, 0));
    }
}
