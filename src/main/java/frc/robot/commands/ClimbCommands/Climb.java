// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;



public class Climb extends Command {

  private boolean isDone = false;
  
  double robotRoll;

  
  /** Creates a new Climb. */
  public Climb() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    if(!ClimberActivate.isActive && !Robot.climber.lockedIN())new ClimberActivate();
    
    Robot.climber.climb(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotRoll = Robot.gyro.getRoll();


    Robot.climber.climb(-.1);

    while(Robot.climber.getLeftMotorCurrent() > Constants.ClimberConstants.climbVoltage && Robot.climber.getRightMotorCurrent() > Constants.ClimberConstants.climbVoltage){
      Robot.led.setColor(0,0,255);
      robotRoll = -Robot.gyro.getRoll();

      if(robotRoll>0)Robot.climber.climb(robotRoll*.01, robotRoll*.05);
      else if(robotRoll<0) Robot.climber.climb(robotRoll*.05, robotRoll*.01);
      else Robot.climber.climb(.1);



    }
    

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.setBrakeMode();
    isDone = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
