package frc.robot.commands.VisionCommands;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FunctionalCommands.ExecuteCommand;
import frc.robotkt.constants.ArmConstants;
import frc.robotkt.constants.VisionConstants;
import frc.robotkt.subsystems.Arm;
import frc.robotkt.subsystems.Camera;
import frc.robotkt.subsystems.Drivetrain;
import frc.robotkt.subsystems.Notifier;

class AngleArm extends Command {
    final TagAlign aligner;
    final Arm arm;

    AngleArm(Drivetrain drivetrain, Arm arm, TagAlign aligner) {
        this.aligner = aligner;
        this.arm = arm;

        // lock the drivetrain to prevent controller use.
        addRequirements(drivetrain, arm);
    }

    /// Returns height, hypotenuse between piv pt, wall, speaker
    double[] speakerTriangle() {
        // base height
        double height = ArmConstants.kPivotHeight;

        // tag triangle hypotenuse
        Translation3d cam2target = aligner.target.getBestCameraToTarget().getTranslation();
        Translation3d target2cam = cam2target.unaryMinus();
        Translation3d cam2bot = VisionConstants.kCameraToRobot.getTranslation();
        Translation3d cam2botWithHeight = new Translation3d(
            cam2bot.getX(),
            cam2bot.getY(),
            ArmConstants.kPivotHeight - VisionConstants.kCameraHeight
        );
        double target2bot = target2cam.plus(cam2botWithHeight).getDistance(new Translation3d());

        // tag triangle height
        double tagHeight = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
            .getTagPose(aligner.target.getFiducialId())
            .get()
            .getZ() - height;

        // base
        double base = Math.sqrt(Math.pow(target2bot, 2) - Math.pow(tagHeight, 2));

        // add to base for further calculations. currently at robot center, move to pivot pos
        base += ArmConstants.kRobotToPivot;

        // speaker height
        // http://firstfrc.blob.core.windows.net/frc2024/Manual/Sections/2024GameManual-05ARENA.pdf page 11
        double speakerHeight = (198 * 0.001) - height; // 198cm to m, account for base height

        // recalculate hypot
        double hypot = Math.sqrt(Math.pow(base, 2) + Math.pow(speakerHeight, 2));

        return new double[]{height, hypot};
    }

    public void initialize() {
        // Calculate the speaker triangle
        double[] speakerTriangle = speakerTriangle();
        double stHeight = speakerTriangle[0];
        double stHypot = speakerTriangle[1];

        // wall-pivot-speaker
        double theta = Math.asin(stHeight / stHypot);

        // law of sines coming in clutch
        // pivot-speaker-shooter
        double speakerAngle = Math.asin(ArmConstants.kLength * (Math.sin(ArmConstants.kShintakeAngle) / stHypot));

        // triangle angles add up to pi rad. calculate the last angle
        double pivotAngle = Math.PI - theta - speakerAngle;

        // arm takes different angle as input
        double angle = Math.PI - theta - pivotAngle;

        // I never want to do this much geometry ever again
        arm.setAngleTarget(Math.toDegrees(angle));
    }

    public boolean isFinished() {
        return arm.atPivTarget();
    }
}

public class AngleShooter extends SequentialCommandGroup {
    public AngleShooter(
        Drivetrain drivetrain,
        Arm arm,
        Camera camera,
        Notifier notifier,
        CommandXboxController driverctl
    ) {
        TagAlign aligner = new TagAlign(drivetrain, camera, driverctl, 4, 7);
        AngleArm angler = new AngleArm(drivetrain, arm, aligner);

        addCommands(
            new ExecuteCommand(() -> notifier.getLed().setRGB(255, 0, 0)),
            aligner,
            new ExecuteCommand(() -> notifier.getLed().setRGB(0, 255, 0)),
            angler,
            new ExecuteCommand(() -> notifier.getLed().setRGB(0, 0, 255)),
            new ExecuteCommand(notifier::notify)
        );
    }
}
