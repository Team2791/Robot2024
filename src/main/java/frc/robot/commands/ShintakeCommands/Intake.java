// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shintake;

class TakeIn extends Command {
    final Shintake shintake;

    public TakeIn(Shintake shintake) {
        this.shintake = shintake;

        addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setIntake(ShintakeConstants.IntakeSpeeds.kIntake);
    }

    @Override
    public void end(boolean interrupted) {
        shintake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return shintake.isLoaded();
    }
}

class Load extends Command {
    final Shintake shintake;

    public Load(Shintake shintake) {
        this.shintake = shintake;
        addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setIntake(ShintakeConstants.IntakeSpeeds.kLoad);
        shintake.setShooter(ShintakeConstants.ShooterSpeeds.kLoad);
    }

    @Override
    public void end(boolean interrupted) {
        shintake.stopIntake();
        shintake.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return !shintake.isLoaded();
    }
}

public class Intake extends SequentialCommandGroup {
    public Intake(Shintake shintake, Led led) {
        addCommands(
                new RunCommand(() -> led.setRGB(255, 0, 0)),
                new TakeIn(shintake),
                new RunCommand(() -> led.setRGB(0, 255, 0)),
                new Load(shintake),
                new RunCommand(() -> led.setRGB(0, 0, 255))
        );
    }
}