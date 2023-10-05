// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoCubeinConeOut extends CommandBase {

	private static double useSpeed = 0.65;
	Timer timer = new Timer();
	double t;

	/** Creates a new AutoSpit. */
	public AutoCubeinConeOut(double time) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.intake);
		t = time;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Robot.intake.setIntakeMotor(useSpeed);
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		Robot.intake.setIntakeMotor(useSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.intake.setIntakeMotor(0.04);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > t;
	}
}
