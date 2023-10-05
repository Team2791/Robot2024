package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ScoringExtension extends CommandBase {

	private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints(950, 190); // 650, 60
	private ProfiledPIDController pid = new ProfiledPIDController(Constants.ExtendKP, 0, Constants.ExtendKD,
			constraint, 0.02);
	// 0.2, 0, 0

	/** Creates a new ExtendTime. */
	public ScoringExtension() {
		addRequirements(Robot.arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (Constants.mode)
			pid.setGoal(Constants.ConeUpperExtension);
		if (!Constants.mode)
			pid.setGoal(Constants.CubeExtension);

		pid.setTolerance(1.2);
		Constants.ManualExtend = false;

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double output = pid.calculate(Robot.arm.getExtendPot());
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
		return pid.atGoal();
	}
}
