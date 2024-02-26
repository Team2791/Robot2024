package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ManualAngle extends Command {
	public static enum Direction {
		Up, Down;

		public boolean up() {
			return switch (this) {
				case Up -> true;
				case Down -> false;
			};
		}
	}

	private final Direction direction;
	private final Arm arm;

	public ManualAngle(Direction direction, Arm arm) {
		this.direction = direction;
		this.arm = arm;
	}

	public void initialize() {
		if (direction.up()) this.arm.up();
		else this.arm.down();
	}

	public void end(boolean interrupted) {
		this.arm.hold();
	}

	public boolean isFinished() {
		return true;
	}
}
