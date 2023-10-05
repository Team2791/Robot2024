package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class PivotSet extends CommandBase {
	/** Creates a new rotatearm. */
	private Timer timer = new Timer();
	public double distance, output, mode = 1;
	private PIDController PID;
	private boolean facing = false;

	// 1 = scoring
	// 2 = station
	// 3 = intaking
	// 4 = mid
	//5 = dif
	/** Creates a new ExtendTime. */
	public PivotSet(double set, boolean face) {
		PID = new PIDController(Constants.PivotKP, Constants.PivotKI,
				Constants.PivotKD, 0.02);
		mode = set;
		facing = face;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Constants.ManualPivot = false;
		PID.setTolerance(1.7);

		if (Constants.mode && Robot.orientation && mode == 1) {
			PID.setSetpoint(Constants.ConeScoringPivotFront);
		}
		if (Constants.mode && !Robot.orientation && mode == 1) {
			PID.setSetpoint(Constants.ConeScoringPivotBack);
		}

		if (!Constants.mode && Robot.orientation && mode == 1) {
			PID.setSetpoint(Constants.CubeScoringPivotFront);
		}
		if (!Constants.mode && !Robot.orientation && mode == 1) {
			PID.setSetpoint(Constants.CubeScoringPivotBack);
		}

		if (Robot.orientation && mode == 2) {
			PID.setSetpoint(Constants.NewStationPivotBack);
		}
		if (!Robot.orientation && mode == 2) {
			PID.setSetpoint(Constants.NewStationPivotFront);
		}

		if (Constants.mode && mode == 3 && facing) {
			PID.setSetpoint(Constants.ConeIntakePivotFront);
		}
		if (Constants.mode && mode == 3 && !facing) {
			PID.setSetpoint(Constants.ConeIntakePivotBack);
		}
		if (!Constants.mode && mode == 3 && facing) {
			PID.setSetpoint(Constants.CubeIntakePivotFront);
		}
		if (!Constants.mode && mode == 3 && !facing) {
			PID.setSetpoint(Constants.CubeIntakePivotBack);
		}

		if (mode == 4 && Robot.orientation) {
			PID.setSetpoint(Constants.MidConeScoringPivotFront);
		}
		if (mode == 4 && !Robot.orientation) {
			PID.setSetpoint(Constants.MidConeScoringPivotBack);
		}

		if(mode == 5 && Robot.orientation) {
			PID.setSetpoint(-50);
		}
		if(mode == 5 && !Robot.orientation) {
			PID.setSetpoint(45);
		}
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
	}
}
