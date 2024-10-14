package frc.robot.commands.FunctionalCommands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that executes the given function once, in initialize(), then immediately stops
 */
public class ExecuteCommand extends FunctionalCommand {
    public ExecuteCommand(Runnable func, Subsystem... reqs) {
        super(func, () -> {}, (interrupted) -> {}, () -> true, reqs);
    }
}
