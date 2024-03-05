package frc.robot.commands.ArmCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ManualExtension extends Command {
  boolean inout = false;
  /** Creates a new Extension. */
  public ManualExtension() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(Robot.arm.getExtensionPot()>95 && Robot.arm.extensionMotor.getEncoder().getVelocity()>0){
      Robot.arm.extensionMotor.set(0);
    }
    else {Robot.arm.manualExtend();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.arm.stopExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}