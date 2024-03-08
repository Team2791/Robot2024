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
  boolean aPressedOnInit = false;
  
  /** Creates a new Intake. */
  public Intake() {
    

    addRequirements(Robot.shintake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shintake.takeIn();
    aPressedOnInit = RobotContainer.m_operatorController.a().getAsBoolean();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, .3);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, .5);
    while(Robot.shintake.isin()){
      Robot.shintake.slowOut();
    }
    Robot.shintake.stopIntake();
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.shintake.isin() || RobotContainer.m_operatorController.getHID().getAButtonReleased();
  }
}
