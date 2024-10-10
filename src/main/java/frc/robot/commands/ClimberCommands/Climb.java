package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    final Climber climber;

    public Climb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.set(-ClimberConstants.Speeds.kClimb);
    }

    @Override
    public void execute() {
        double bias = climber.bias();
        double left = -ClimberConstants.Speeds.kClimb;
        double right = -ClimberConstants.Speeds.kClimb;

        if (Math.signum(bias) == 1) left *= bias;
        else right *= -bias;

        climber.set(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        climber.set(0);
        climber.lock();
    }

    @Override
    public boolean isFinished() {
        return climber.getLeftPos() <= ClimberConstants.Motor.kEncoderMax;
    }
}
