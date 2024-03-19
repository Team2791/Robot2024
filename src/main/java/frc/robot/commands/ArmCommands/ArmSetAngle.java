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
  PIDController armpid;
  double angle;

  /** Creates a new ArmSetAngle. */
  public ArmSetAngle(double a) {
    this.angle = a;
    armpid = new PIDController(.0168,0,0);
    // Use addRequirements() here to declare subsystem dependencies.
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

			Robot.arm.armLeft.set(armpid.calculate(Robot.arm.getArmPot(),angle));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, .5);
    RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, .5);

    Robot.arm.hold();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armpid.atSetpoint();
  }
}
