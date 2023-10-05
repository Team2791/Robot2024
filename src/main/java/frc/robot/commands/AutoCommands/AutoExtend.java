package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoExtend extends CommandBase {
	private PIDController PID;
	private double distance;

	/** Creates a new ExtendTime. */
	public AutoExtend(double length) {
		addRequirements(Robot.arm);
		distance = length;
		PID = new PIDController(Constants.ExtendKP, Constants.ExtendKI,
				Constants.ExtendKD);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		PID.setSetpoint(distance);
		PID.setTolerance(1.3);
		Constants.ManualExtend = false; 

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double output = PID.calculate(Robot.arm.getExtendPot());

		if (output > Constants.ExtendMaxSpeed)
			output = Constants.ExtendMaxSpeed;
		if (output < -Constants.ExtendMaxSpeed)
			output = -Constants.ExtendMaxSpeed;

		Robot.arm.extend(output);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.arm.extend(0);
		Constants.ManualExtend = true; 
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return PID.atSetpoint();
	}
}
