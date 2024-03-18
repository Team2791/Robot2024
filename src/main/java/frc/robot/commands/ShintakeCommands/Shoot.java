// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Shoot extends Command {
  Timer timer = new Timer();
  /** Creates a new Shoot. */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Robot.shintake.takeIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.start();

    while(timer.get()<.5){
      
    }
    Robot.shintake.setShooter(0);
    Robot.shintake.stopIntake();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>.5 && !Robot.shintake.isin());
  }
}
