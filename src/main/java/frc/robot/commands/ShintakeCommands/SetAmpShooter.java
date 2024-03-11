// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SetAmpShooter extends Command {
  Timer timer = new Timer();
  
  /** Creates a new Shoot. */
  public SetAmpShooter() {
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shintake.setShooter(.1,.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Robot.shintake.getRPM()>-Constants.ShintakeConstants.kRPM){
      RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, .3);
      RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, .5);
    }

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Robot.shintake.setShooter(0, 0);
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);


  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return RobotContainer.m_driverController.getRightTriggerAxis()>.5 || timer.get()>10;

  }
}