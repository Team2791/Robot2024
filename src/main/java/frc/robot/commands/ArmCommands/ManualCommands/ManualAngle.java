package frc.robot.commands.ArmCommands.ManualCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robotkt.subsystems.Arm;

import java.util.function.Consumer;

public class ManualAngle extends Command {
    private static final Consumer<Arm> AngleDown = Arm::angleDown;
    private static final Consumer<Arm> AngleUp = Arm::angleUp;

    final Arm arm;
    final Consumer<Arm> angler;

    public ManualAngle(Arm arm, boolean up) {
        this.arm = arm;
        this.angler = up ? AngleUp : AngleDown;

        addRequirements(arm);
    }

    public void initialize() {
        angler.accept(arm);
    }

    public void end(boolean interrupted) {
        arm.hold();
    }
}