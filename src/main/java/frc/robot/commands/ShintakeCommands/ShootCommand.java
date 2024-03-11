// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShootCommand extends Command {
  Timer timer = new Timer();
  /** Creates a new ShootCommand. */
  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Robot.shintake.setShooter(1,1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>2){
      Robot.shintake.takeIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      Robot.shintake.setShooter(0, 0);
      Robot.shintake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>3; 
  }
}
