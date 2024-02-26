package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shintake;

public class Take extends Command {
	private final Shintake shintake;
	private Timer duration;

	public Take(Shintake shintake) {
		this.shintake = shintake;
		this.duration = new Timer();
	}

	public void initialize() {
		this.shintake.setIntake(0.5);
		this.duration.reset();
	}

	public void end(boolean interrupted) {
		Spit.time = this.duration.get();
		this.shintake.setIntake(0);
	}

	public boolean isFinished() {
		return this.shintake.broken();
	}
}
