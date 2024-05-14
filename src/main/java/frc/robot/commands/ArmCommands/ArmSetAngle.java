// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmSetAngle extends Command {
  double angle;

  /** Creates a new ArmSetAngle. */
  public ArmSetAngle(double a) {
    angle = Math.min(Math.max(0, a), 53);
    addRequirements(Robot.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	Robot.arm.SetAngleTarget(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.arm.AtAngleTarget();
  }
}
