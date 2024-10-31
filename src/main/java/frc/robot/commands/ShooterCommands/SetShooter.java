// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FunctionalCommands.ExecuteCommand;
import frc.robot.subsystems.Shooter;

import java.util.concurrent.atomic.AtomicBoolean;

public class SetShooter extends Command {
    final Shooter shooter;
    final AtomicBoolean signal;

    public SetShooter(Shooter shooter, AtomicBoolean signal) {
        this.shooter = shooter;
        this.signal = signal;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        signal.set(false);
        shooter.setShooter();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !signal.get()) shooter.stop();

        signal.set(false);
        shooter.kick();

        // Stop the shooter after 1s without nonsense threading
        new SequentialCommandGroup(new WaitCommand(1.0), new ExecuteCommand(shooter::stop, shooter)).schedule();
    }

    @Override
    public boolean isFinished() {
        return signal.get();
    }
}
