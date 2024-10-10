// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

public class Shoot extends Command {
    final Shintake shintake;

    public Shoot(Shintake shintake) {
        this.shintake = shintake;
        //addRequirements(shintake);
    }

    @Override
    public void initialize() {
        shintake.setIntake(ShintakeConstants.IntakeSpeeds.kIntake);
    }

    @Override
    public void end(boolean interrupted) {
        shintake.stopIntake();
        shintake.stopShooter();
    }
}
