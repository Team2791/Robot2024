package frc.robot.commands.ClimberCommands.actuator;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class LinearUnlock extends Command {
	public void initialize() {
		Robot.climber.lock();
	}

	public boolean isFinished() {
		return true;
	}
}
