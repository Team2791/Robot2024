// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class spitOut extends Command {
  Timer timer;
  boolean isout = false;
  /** Creates a new spitOut. */
  public spitOut() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    while(timer.get()<3){
    Robot.shintake.spitOut();
  }
    isout = true;
    Robot.led.setColor(0,0,255);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shintake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isout;
  }
}
