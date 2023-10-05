package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class PivotArm extends CommandBase {
	/** Creates a new rotatearm. */
	private Timer timer = new Timer();
	public double distance, output;
	private PIDController PID;
	private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints(500, 50);
	private ProfiledPIDController pid = new ProfiledPIDController(0.20, 0, 0, constraint);

	/** Creates a new ExtendTime. */
	public PivotArm(double pivot) {
		distance = pivot;
		PID = new PIDController(Constants.PivotKP, Constants.PivotKI, Constants.PivotKD, 0.02);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Constants.ManualPivot = false;
		PID.setTolerance(1.7);
		PID.setSetpoint(distance);
		pid.setTolerance(1.7);
		pid.setGoal(distance);
		Robot.arm.setPivotbrake(false);
		timer.start();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!Constants.ManualPivot && timer.get() > 0.15) {
			output = PID.calculate(Robot.arm.getPivotPot());
			if (output > Constants.PivotMaxSpeed)
				output = Constants.PivotMaxSpeed;
			if (output < -Constants.PivotMaxSpeed)
				output = -Constants.PivotMaxSpeed;
		}

		Robot.arm.pivotArm(output);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.arm.pivotArm(0);
		Robot.arm.setPivotbrake(true);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return PID.atSetpoint();
		// return pid.atGoal();
	}
}
