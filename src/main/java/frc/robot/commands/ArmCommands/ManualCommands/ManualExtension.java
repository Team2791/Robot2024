package frc.robot.commands.ArmCommands.ManualCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ManualExtension extends Command {
    boolean inout = false;

    /**
     * Creates a new Extension.
     */
    public ManualExtension() {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.arm.manualExtend();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.arm.stopExtension();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.arm.GetExtension() > 95;
    }
}