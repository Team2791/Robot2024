package frc.robot.commands.ClimberCommands.activate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class ClimberDeactivate extends Command {
	public void initialize() {
		Robot.climber.setAll(0.3);
	}

	public void end(boolean interrupted) {
		Robot.climber.setAll(0);
	}

	public boolean isFinished() {
		return Robot.climber.leftPos() <= ClimberConstants.RelativeEncoderWhileDown;
	}
}
