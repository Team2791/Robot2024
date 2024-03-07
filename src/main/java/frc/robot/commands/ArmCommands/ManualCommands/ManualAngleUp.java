package frc.robot.commands.ArmCommands.ManualCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
    timer.start();

    while(timer.get()<ArmConstants.kArmSpeedUp/ArmConstants.kAccelerationTime){
      Robot.arm.moveUp((ArmConstants.kArmSpeedUp/ArmConstants.kAccelerationTime)*timer.get());}

    Robot.arm.moveUp(ArmConstants.kArmSpeedUp);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.start();
    
    while(timer.get()<ArmConstants.kArmSpeedUp/ArmConstants.kAccelerationTime){
      Robot.arm.moveUp(1-((ArmConstants.kArmSpeedUp/ArmConstants.kAccelerationTime)*timer.get()));}


    Robot.arm.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}