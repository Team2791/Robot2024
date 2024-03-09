// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FullExtensionAmp extends Command {
  Timer timer = new Timer();
  boolean aPressedOnInit = false;
  /** Creates a new Fullextension. */
  public FullExtensionAmp() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Robot.arm.manualExtend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.arm.stopExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.arm.getExtensionPot()>70 || timer.get()>3 || !RobotContainer.m_operatorController.getHID().getBButton();// || (aPressedOnInit && !RobotContainer.m_operatorController.a().getAsBoolean());
  }
}
