package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SlowAutoDrive extends CommandBase {
	/** Creates a new Trapezoid. */

	private double setpoint;
	private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints(120, 100);
	private ProfiledPIDController pid = new ProfiledPIDController(0.42, 0, 0, constraint);

	public SlowAutoDrive(double meters) {
		// Use addRequirements() here to declare subsystem dependencies.
		setpoint = meters;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Robot.drivetrain.resetEncoders();
		Robot.drivetrain.setBrakeMode();
		pid.setTolerance(0.8);
		pid.setGoal(setpoint);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double output = pid.calculate(Robot.drivetrain.getAveragePosition());

		if (output > 0.68)
			output = 0.68;
		if (output < -0.68)
			output = -0.68;

		Robot.drivetrain.arcadeDrive(output, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.setMotors(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pid.atGoal();
	}
}