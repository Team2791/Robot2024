package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberCommands.actuator.LinearLock;
import frc.robot.commands.ClimberCommands.actuator.LinearUnlock;
import frc.robot.commands.ClimberCommands.climbing.ClimbDown;

public class ClimbRelease extends SequentialCommandGroup {
	public ClimbRelease() {
		addCommands(new LinearUnlock());
		addCommands(new ClimbDown());
		addCommands(new LinearLock());
	}
}
