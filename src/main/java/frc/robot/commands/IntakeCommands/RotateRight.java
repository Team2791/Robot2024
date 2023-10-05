// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class RotateRight extends CommandBase {
	/** Creates a new rotateFront. */
	Timer timer = new Timer();

	public RotateRight() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Robot.intake.rotateMotor.setSmartCurrentLimit(70);
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Robot.arm.getPivotPot() < 50 && Robot.arm.getPivotPot() > -50) {
			Robot.intake.setRotateMotor(Constants.IntakeRotionSpeed);
		}
		Robot.intake.rotateMotor.setSmartCurrentLimit(10);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.intake.setRotateMotor(0.017);
		Robot.intake.rotateMotor.setSmartCurrentLimit(2);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Robot.intake.getLeftButton() || timer.get() > 2;
	}
}
