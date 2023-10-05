
package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChargeStationDriveBack extends CommandBase {
	/** Creates a new Drive. */
	public ChargeStationDriveBack() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Robot.drivetrain.setMotors(0.28, 0.28); // 0.4 //0.34 did not work
		Robot.drivetrain.arcadeDrive(0.55, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(Robot.drivetrain.getPitch()) > 17;
	}
}