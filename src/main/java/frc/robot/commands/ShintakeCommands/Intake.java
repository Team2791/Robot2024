// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FunctionalCommands.ExecuteCommand;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;
import frc.robotkt.subsystems.Notifier;

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
    public Intake(Shintake shintake, Notifier notifier) {
        addCommands(new TakeIn(shintake), new Load(shintake), new ExecuteCommand(notifier::notify, notifier));
    }
}
