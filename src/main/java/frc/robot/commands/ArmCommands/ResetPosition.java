// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ResetPosition extends Command {
  /** Creates a new ResetPosition. */
  public ResetPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.arm.moveUp(.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.arm.hold();
    Robot.shintake.stopIntake();
    Robot.shintake.setShooter(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.arm.armLeft.getEncoder().getPosition()>15;
  }
}
