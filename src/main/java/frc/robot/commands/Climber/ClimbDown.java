package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
	private final Timer timer = new Timer();
	private final Climber climber;

	public ClimbDown(Climber climber) {
		this.climber = climber;
	}

	public void initialize() {
		climber.unlock();
		climber.climb(0.3);
	}

	public void end(boolean interrupted) {
		climber.climb(0);
	}

	public boolean isFinished() {
		return timer.get() >= 5.0;
	}
}
