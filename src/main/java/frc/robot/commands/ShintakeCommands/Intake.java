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
  Timer timer = new Timer();
  boolean cancel = false;
  
  /** Creates a new Intake. */
  public Intake() {
    

    addRequirements(Robot.shintake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Robot.led.setColor(255,99,71);
    Robot.shintake.takeIn();
    Robot.shintake.setShooter(-.1, -.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.shintake.getIntakeCurrent()>50){
      Robot.arm.moveDown(.1);
    }
    else Robot.arm.hold();
  
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, .3);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, .5);
    
    Robot.led.setColor(0, 255, 0);
    while(Robot.shintake.isin()){
      Robot.shintake.slowOut();
      Robot.shintake.setShooter(-.1, -.1);
    }
    
    Robot.shintake.stopIntake();
    Robot.shintake.setShooter(0, 0);
    Robot.arm.hold();
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Robot.shintake.isin() || timer.get()>5);
  }
}
