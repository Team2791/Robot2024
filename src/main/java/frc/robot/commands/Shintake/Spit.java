package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shintake;

public class Spit extends Command {
	public static double time = -1;

	private final Shintake shintake;
	private final Timer timer = new Timer();

	public Spit(Shintake shintake) {
		this.shintake = shintake;
	}

	public void initialize() {
		this.shintake.setIntake(-0.5);
		timer.reset();
	}

	public void end(boolean interrupted) {
		this.shintake.setIntake(0);
	}

	public boolean isFinished() {
		return timer.get() > time;
	}
}
