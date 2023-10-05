// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ConeInCubeOut extends CommandBase {
	/** Creates a new Outake. */
	public ConeInCubeOut() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Robot.intake.setIntakeMotor(Constants.OutakeSpeed);
		Constants.Intaking = true;

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Robot.intake.intakeMotor.getOutputCurrent() > Constants.IntakeMaxCurrent) {
			Robot.led.setColor(0, 255, 0);
		}
		else {
			Robot.led.setColor(255, 255, 255);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.intake.setIntakeMotor(-0.06);
		Constants.Intaking = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
