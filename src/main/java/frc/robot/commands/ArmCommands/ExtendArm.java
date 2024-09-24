package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robotkt.subsystems.Arm;

public class ExtendArm extends Command {
    final Arm arm;
    final double percent;

    public ExtendArm(Arm arm, double percent) {
        this.arm = arm;
        this.percent = percent;
    }

    public void initialize() {
        arm.setExtension(percent);
    }

    public boolean isFinished() {
        return arm.atExtTarget();
    }
}
