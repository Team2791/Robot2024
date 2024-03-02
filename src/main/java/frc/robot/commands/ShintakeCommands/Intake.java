// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Intake extends Command {
  boolean isIn = false;
  Timer timer = new Timer();
  /** Creates a new Intake. */
  public Intake() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot.shintake.takeIn();
    
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble,.5);
    isIn = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shintake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isIn;
  }
}
