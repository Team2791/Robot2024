package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class ManualClimb extends Command {
    final Consumer<Double> climbFn;
    final Climber climber;
    final double speed;

    public ManualClimb(Climber climber, int side, boolean up, double speed) {
        Consumer<Double> fun = List.<Consumer<Double>>of(
                climber::setLeft,
                climber::set,
                climber::setRight
        ).get(side);

        if (!up) fun = compose(fun, n -> -n);

        this.climbFn = fun;
        this.climber = climber;
        this.speed = speed;
    }

    private static <T> Consumer<T> compose(Consumer<T> a, Function<T, T> b) {
        return (n) -> a.accept(b.apply(n));
    }

    public void initialize() {
        climbFn.accept(speed);
    }

    public void end(boolean interrupted) {
        climbFn.accept(0.0);
    }
}
