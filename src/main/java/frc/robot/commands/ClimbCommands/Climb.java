// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;


public class Climb extends Command {

  private boolean isDone = false;
  
  double robotRoll;

  
  /** Creates a new Climb. */
  public Climb() {
    Timer timer = new Timer();
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
    robotRoll = Robot.climber.drivetrain.m_gyro.getRoll();


    Robot.climber.climb(-.1);

    while(Robot.climber.getLeftMotorCurrent() > Constants.RobotConstants.climbVoltage && Robot.climber.getRightMotorCurrent() > Constants.RobotConstants.climbVoltage){
      Robot.led.setColor(0,0,255);
      robotRoll = -Robot.climber.drivetrain.m_gyro.getRoll();

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
