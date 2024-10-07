// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

public class Shoot extends Command {
    final Shintake shintake;
    int status = 0;

    public Shoot(Shintake shintake) {
        this.shintake = shintake;
        addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setIntake(ShintakeConstants.IntakeSpeeds.kIntake);
    }

    @Override
    public void execute() {
        // status will be even iff the shintake was not loaded
        // by taking (!isLoaded) as int, we know when to increment status
        if (status % 2 == (shintake.isLoaded() ? 0 : 1)) status += 1;
    }

    @Override
    public void end(boolean interrupted) {
        shintake.stopIntake();
        if (!interrupted) shintake.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return status >= 2;
    }
}
