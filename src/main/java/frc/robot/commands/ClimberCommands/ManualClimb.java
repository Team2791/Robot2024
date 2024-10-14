package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.Climber;

import java.util.function.Consumer;

public class ManualClimb extends Command {
    final Climber climber;
    final ClimbSide side;
    final boolean reversed;
    final double speed;

    public ManualClimb(Climber climber, ClimbSide side, boolean up, double speed) {
        this.climber = climber;
        this.side = side;
        this.reversed = !up;
        this.speed = speed;
    }

    public ManualClimb(Climber climber, ClimbSide side, boolean up) {
        this(climber, side, up, ClimberConstants.Speeds.kClimb);
    }

    public void initialize() {
        Consumer<Double> fun = switch (this.side) {
            case kLeft -> climber::setLeft;
            case kRight -> climber::setRight;
            case kBoth -> climber::set;
        };

        if (reversed) fun.accept(-speed);
        else fun.accept(speed);
    }

    public void end(boolean interrupted) {
        climber.set(0);
    }

    public enum ClimbSide {
        kLeft, kBoth, kRight,
    }
}
