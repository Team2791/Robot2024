package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Level extends CommandBase {
	/** Creates a new Level. */
	private double thrust;
	private PIDController pid;

	public Level() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);// 0.005
		pid = new PIDController(0.0225, 0.000, 0.004); // COMPETITION 0.022, 0.004
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		pid.setTolerance(0);
		pid.setSetpoint(0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		thrust = pid.calculate(Robot.drivetrain.getPitch());

		if (thrust > 0.3)
			thrust = 0.3;
		if (thrust < -0.3)
			thrust = -0.3;

		Robot.drivetrain.arcadeDrive(thrust, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.arcadeDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pid.atSetpoint();
	}
}