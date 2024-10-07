package frc.robot.commands.ClimberCommands;

import frc.robot.subsystems.Climber;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class ManualClimb {
    private ManualClimb(Climber climber, int side, boolean up) {
        Consumer<Double> fun = List.<Consumer<Double>>of(
                climber::setLeft,
                climber::set,
                climber::setRight
        ).get(side);

        if (up) fun = compose(fun, n -> -n);
    }

    private static <T> Consumer<T> compose(Consumer<T> a, Function<T, T> b) {
        return (n) -> a.accept(b.apply(n));
    }
}
