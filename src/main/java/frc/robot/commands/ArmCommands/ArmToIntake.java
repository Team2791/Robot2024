package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.constants.ArmConstants;
import frc.robotkt.subsystems.Arm;

public class ArmToIntake extends SequentialCommandGroup {
    public ArmToIntake(Arm arm) {
        addCommands(new AngleArm(arm, ArmConstants.kIntakeAngle));
    }
}
