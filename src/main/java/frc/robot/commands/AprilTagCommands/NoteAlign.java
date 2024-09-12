// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class NoteAlign extends Command {
    boolean atSetpoint;
    PIDController pid;
    double output;
    int nonPIDTolerance = 3;

    /**
     * Creates a new NoteAlign.
     */
    public NoteAlign() {
        addRequirements(Robot.m_drivetrain);
        pid = new PIDController(.01, 0, 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.setTolerance(1);
        pid.setSetpoint(300);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var result = Robot.camera1.getLatestResult();
        if (result != null && result.hasTargets()) {
            output = pid.calculate(result.getBestTarget().getYaw()); // -8

            Robot.m_drivetrain.drive(0, -output, 0, false, false);
            if (result.getBestTarget().getYaw() < nonPIDTolerance && result.getBestTarget().getYaw() > -nonPIDTolerance) {
                Robot.m_drivetrain.stop();
            }
        }

        if (result == null) {
            output = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
