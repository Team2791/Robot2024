// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class takeIn extends Command {

	private boolean intaked = false;
  /** Creates a new Intake. */
  public takeIn() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.led.setColor(255,0,0);
    Robot.intake.takeIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Robot.intake.Intakemotor.getOutputCurrent() > 5){
      Robot.led.setColor(0,255,0);
	  intaked=true;
    }
    

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intaked;
  }
}
