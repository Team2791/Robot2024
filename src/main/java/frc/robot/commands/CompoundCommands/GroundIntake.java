package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.ArmToGround;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.subsystems.Shintake;
import frc.robotkt.subsystems.Arm;
import frc.robotkt.subsystems.Notifier;

public class GroundIntake extends ParallelCommandGroup {
    public GroundIntake(Arm arm, Shintake shintake, Notifier notifier) {
        addCommands(new ArmToGround(arm), new Intake(shintake, notifier), new ResetArm(arm));
    }
}
