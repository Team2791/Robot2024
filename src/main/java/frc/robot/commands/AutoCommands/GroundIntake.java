package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.ArmToGround;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shintake;
import frc.robotkt.subsystems.Arm;

public class GroundIntake extends ParallelCommandGroup {
    public GroundIntake(Arm arm, Shintake shintake, Led led) {
        addCommands(new ArmToGround(arm), new Intake(shintake, led));
    }
}
