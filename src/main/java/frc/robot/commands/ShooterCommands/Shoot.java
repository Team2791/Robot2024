// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.TurretCommands.TurretAngle;

public class Shoot extends Command {

  
  boolean isFinished = false;
  /** Creates a new Shoot. */
  public Shoot() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.setShooter(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {


    switch(TurretAngle.targetID){
      case 4: // speaker tag

        Robot.shooter.setShooter(1);
        if(Robot.shitake.speedController.atSetpoint()){
          break;
        }
        
      case 5: //amp tag
        Robot.shooter.setShooter(.4);
        if(Robot.shitake.speedController.atSetpoint()){
          break;
        }
      case 6: //amp tag
        Robot.shooter.setShooter(.4);
        if(Robot.shitake.speedController.atSetpoint()){
          break;
        }
      case 7: //speaker tag

        Robot.shooter.setShooter(1);
        if(Robot.shitake.speedController.atSetpoint()){
          break;
        }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setShooter(0);
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
