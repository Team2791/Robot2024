// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FunctionalCommands.ExecuteCommand;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

import java.util.concurrent.atomic.AtomicBoolean;

public class SetShooter extends Command {
    final Shintake shintake;
    final AtomicBoolean signal;
    final double speed;

    public SetShooter(Shintake shintake, AtomicBoolean signal, double speed) {
        this.shintake = shintake;
        this.signal = signal;
        this.speed = speed;

        addRequirements(shintake);
    }

    public SetShooter(Shintake shintake, AtomicBoolean signal) {
        this(shintake, signal, ShintakeConstants.ShooterSpeeds.kShoot);
    }

    @Override
    public void initialize() {
        shintake.setShooter(speed);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !signal.get()) shintake.setShooter(0);
        signal.set(false);
        shintake.setIntake(ShintakeConstants.IntakeSpeeds.kIntake);

        // Stop the shooter after 1s without nonsense threading
        new SequentialCommandGroup(new WaitCommand(1.0), new ExecuteCommand(() -> {
            shintake.stopShooter();
            shintake.stopIntake();
        })).schedule();
    }

    @Override
    public boolean isFinished() {
        return signal.get();
    }
}
