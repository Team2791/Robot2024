// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class RotateWrist extends CommandBase {
	/** Creates a new RotateWrist. */
	private boolean state;

	public RotateWrist() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (Robot.intake.getLeftButton())
			state = false;
		if (Robot.intake.getRightButton())
			state = true;
		if (!Robot.intake.getRightButton() && !Robot.intake.getLeftButton())
			state = true;

		Robot.intake.rotateMotor.setSmartCurrentLimit(70);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Robot.arm.getPivotPot() < 50 && Robot.arm.getPivotPot() > -50 && state) {
			Robot.intake.setRotateMotor(Constants.IntakeRotionSpeed);
		}

		if (Robot.arm.getPivotPot() < 50 && Robot.arm.getPivotPot() > -50 && !state) {
			Robot.intake.setRotateMotor(-Constants.IntakeRotionSpeed);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (state)
			Robot.intake.setRotateMotor(0.017);
		if (!state)
			Robot.intake.setRotateMotor(-0.017);

		Robot.intake.rotateMotor.setSmartCurrentLimit(2);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (state)
			return Robot.intake.getLeftButton();
		if (!state)
			return Robot.intake.getRightButton();
		return false;
	}
}
