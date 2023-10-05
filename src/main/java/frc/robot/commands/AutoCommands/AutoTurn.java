package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoTurn extends CommandBase {
	/** Creates a new Trapezoid. */
	private PIDController normalPID;
	private double setpoint;
	Timer timer = new Timer();

	public AutoTurn(double angle) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);
		setpoint = angle;
		normalPID = new PIDController(0.15, 0, 0.004);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Robot.drivetrain.resetGyro();
		normalPID.setTolerance(1);
		normalPID.setSetpoint(setpoint);
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double output = normalPID.calculate(Robot.drivetrain.getAngle());// chnged to normalpid
		if (output > 0.48)
			output = 0.48;
		if (output < -0.48)
			output = -0.48;
		Robot.drivetrain.arcadeDrive(0, -output);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.arcadeDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return normalPID.atSetpoint() || timer.get() > 0.6;// Changed
	}
}
