package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberCommands.activate.ClimberActivate;
import frc.robot.commands.ClimberCommands.actuator.LinearLock;
import frc.robot.commands.ClimberCommands.climbing.ClimbUp;

public class Climb extends SequentialCommandGroup {
	public Climb(XboxController controller) {
		addCommands(new ClimberActivate());

		// Wait for lb press
		addCommands(new Command() {
			public void initialize() {
				controller.getAButton();
			}

			public boolean isFinished() {
				return controller.getAButtonPressed();
			}
		});

		addCommands(new ClimbUp());
		addCommands(new LinearLock());
	}
}
