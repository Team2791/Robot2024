// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

public class SetShooter extends Command {
    final Shintake shintake;
    final CommandXboxController driverctl;
    final double speed;

    public SetShooter(Shintake shintake, CommandXboxController driverctl, double speed) {
        this.shintake = shintake;
        this.driverctl = driverctl;
        this.speed = speed;

        addRequirements(shintake);
    }

    public SetShooter(Shintake shintake, CommandXboxController driverctl) {
        this(shintake, driverctl, ShintakeConstants.ShooterSpeeds.kShoot);
    }

    @Override
    public void initialize() {
        shintake.setShooter(speed);
    }

    @Override
    public void end(boolean interrupted) {
        driverctl.getHID().setRumble(RumbleType.kBothRumble, 0.3);

        new Thread(() -> {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new Error(e);
            }

            driverctl.getHID().setRumble(RumbleType.kBothRumble, 0);
        }).start();
    }

    @Override
    public boolean isFinished() {
        return shintake.atShooterTarget();
    }
}
