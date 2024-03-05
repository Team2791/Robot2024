package frc.robot.commands.ShintakeCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Intake extends SequentialCommandGroup {
	public Intake() {
		addCommands(new Command() {
			public void initialize() {
				Robot.shintake.intake();
			}

			public boolean isFinished() {
				return Robot.shintake.broken();
			}
		});

		addCommands(new Command() {
			public void initialize() {
				Robot.shintake.slowOut();
			}

			public void end(boolean interrupted) {
				Robot.shintake.stopIntake();
				RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.5);

				// Allows command to exit without taking time
				// even after turning off the controller after 100ms
				new Thread(() -> {
					try {
						Thread.sleep(100);
						RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.5);
					} catch (Exception e) {
						e.printStackTrace();
					}
				});
			}

			public boolean isFinished() {
				return Robot.shintake.broken();
			}
		});
	}
}
