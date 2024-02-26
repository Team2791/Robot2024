package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shintake;

public class Shoot extends Command {
	private final Shintake shooter;

	private Timer timer;
	private boolean done;
	private boolean ready;

	public Shoot(Shintake shooter) {
		this.shooter = shooter;

		this.timer = new Timer();
		this.done = false;
		this.ready = false;

		addRequirements(shooter);
	}

	public void initialize() {
		// TODO: these all need to run at the same speed
		this.shooter.setIntake(Constants.Shooter.EqualSpeeds.Intake);
		this.shooter.setTop(Constants.Shooter.EqualSpeeds.Top);
		this.shooter.setBottom(Constants.Shooter.EqualSpeeds.Bottom);
	}

	public void execute() {
		if (this.ready && this.timer.get() > Constants.Shooter.TimeToShoot) {
			this.done = true;
		} else if (this.shooter.broken() && !this.ready) {
			this.ready = true;

			this.shooter.setIntake(0);
			this.shooter.setTop(1);
			this.shooter.setBottom(1);

			this.timer.start();
		}
	}

	public void end(boolean interrupted) {
		this.shooter.setIntake(0);
		this.shooter.setTop(0);
		this.shooter.setBottom(0);
	}

	public boolean isFinished() {
		return this.done;
	}
}
