package frc.robot.commands.Climber;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Climb extends SequentialCommandGroup {
	public Climb(Climber climber, AHRS gyro) {
		/*
		 * This is an anonymous class. It is a class that is created 
		 * without making a .java file, and doesn't have a 
		 * name (hence, "anonymous"). It is useful when you don't need a 
		 * full class. In this case, we really don't, since there is barely
		 * any logic here, and it is only used as part of another command.
		 */
		addCommands(new Command() {
			Timer timer = new Timer();

			public void initialize() {
				this.timer.reset();
				climber.climb(.3);
			}

			public void end(boolean interrupted) {
				climber.climb(0);
			}

			public boolean isFinished() {
				return this.timer.get() >= 10;
			}
		});

		addCommands(new Command() {
			public void execute() {
				double bias = -gyro.getRoll();

				if (bias > 0) climber.climb(bias * .01, bias * .05);
				else if (bias < 0) climber.climb(bias * .05, bias * .01);
				else climber.climb(.1);
			}

			public void end(boolean interrupted) {
				climber.climb(0);
				climber.lock();
			}

			public boolean isFinished() {
				boolean left = climber.leftAmps() <= Constants.Climber.Voltage;
				boolean right = climber.rightAmps() <= Constants.Climber.Voltage;

				return left || right;
			}
		});
	}
}