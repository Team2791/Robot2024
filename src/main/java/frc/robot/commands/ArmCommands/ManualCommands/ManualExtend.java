package frc.robot.commands.ArmCommands.ManualCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robotkt.subsystems.Arm;

import java.util.function.Consumer;

public class ManualExtend extends Command {
    private static final Consumer<Arm> Extend = Arm::extend;
    private static final Consumer<Arm> Retract = Arm::retract;

    final Arm arm;
    final Consumer<Arm> extender;

    /**
     * Creates a new ManualExtend command.
     *
     * @param arm    The arm subsystem to use
     * @param extend Whether to extend or retract the arm
     */
    public ManualExtend(Arm arm, boolean extend) {
        this.arm = arm;
        this.extender = extend ? Extend : Retract;

        addRequirements(arm);
    }

    public void initialize() {
        extender.accept(arm);
    }

    public void end(boolean interrupted) {
        arm.holdExtension();
    }
}