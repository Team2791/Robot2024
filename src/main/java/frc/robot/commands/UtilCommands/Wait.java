package frc.robot.commands.UtilCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Wait extends Command {
	private final Timer timer = new Timer();
	private final double time;

	public Wait(double time) {
		this.time = time;
	}

	public void initialize() {
		timer.start();
	}

	public boolean isFinished() {
		return timer.get() >= time;
	}
}
