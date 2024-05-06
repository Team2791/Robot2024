package frc.robot.commands.ArmCommands.ManualCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

public class ManualAngleUp extends Command {
  Timer timer = new Timer();
  /** Creates a new ManualAngle. */
  public ManualAngleUp() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Robot.arm.moveUp(Constants.ArmConstants.kArmSpeedUp);  
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
    Robot.arm.hold();
  }  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.arm.GetAngle()<-10;
  }
}