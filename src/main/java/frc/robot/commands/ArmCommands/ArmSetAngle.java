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
    angle = a;
    addRequirements(Robot.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(angle<0) angle=0;
			if(angle>53)angle = 53;

			Robot.arm.setAngle(angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, .5);
    //RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, .5);

    Robot.arm.hold();
    Robot.shintake.setShooter(0,0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.arm.leftPID.atSetpoint();
  }
}
