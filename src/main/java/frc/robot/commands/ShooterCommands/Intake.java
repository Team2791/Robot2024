// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robotkt.subsystems.Notifier;


public class Intake extends Command {
    final Shooter shooter;
    final Notifier notifier;
    final Timer timer = new Timer();

    public Intake(Shooter shooter, Notifier notifier) {
        this.shooter = shooter;
        this.notifier = notifier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setIntake();
    }

    @Override
    public void execute() {
        if (shooter.getKicker() <= ShooterConstants.kIntakeThreshold)
            timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        notifier.vibrate();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(ShooterConstants.kIntakeDelay);
    }
}
