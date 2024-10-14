package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robotkt.constants.ArmConstants;
import frc.robotkt.subsystems.Arm;

public class ArmToGround extends SequentialCommandGroup {
    public ArmToGround(Arm arm) {
        addCommands(new AngleArm(arm, 20), new ExtendArm(arm, 100), new AngleArm(arm, ArmConstants.Pivot.kMinAngle));
    }
}
