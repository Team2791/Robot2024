// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpitOut extends Command {
  /** Creates a new SpitOut. */
  public SpitOut() {
    addRequirements(Robot.shintake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Robot.shintake.spitOut();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Robot.shintake.stopIntake();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !Robot.shintake.isin();

  }
}
