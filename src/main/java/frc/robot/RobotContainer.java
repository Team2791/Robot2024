package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommands.ArmToAmp;
import frc.robot.commands.ArmCommands.ManualCommands.ManualAngleUp;
import frc.robot.commands.ArmCommands.ManualCommands.ManualExtension;
import frc.robot.commands.ArmCommands.ManualCommands.ManualRetraction;
import frc.robot.commands.AutoCommands.IntakeReset;
import frc.robot.commands.AutoCommands.IntakeSequence;
import frc.robot.commands.PitstickCommands.LeftClimbUp;
import frc.robot.commands.PitstickCommands.LeftRelease;
import frc.robot.commands.PitstickCommands.RightClimbUp;
import frc.robot.commands.PitstickCommands.RightRelease;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.commands.ShintakeCommands.SetShooter;
import frc.robot.commands.ShintakeCommands.Shoot;
import frc.robot.commands.ShintakeCommands.SpitOut;
import frc.robot.commands.VisionCommands.TagAlign;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.RGBLED;
import frc.robot.subsystems.Shintake;
import frc.robotkt.constants.VisionConstants;
import frc.robotkt.subsystems.Arm;
import frc.robotkt.subsystems.Camera;
import frc.robotkt.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    // Controllers
    public static CommandXboxController driverctl = new CommandXboxController(OIConstants.kDriverControllerPort);
    public static CommandXboxController operctl = new CommandXboxController(OIConstants.kOperatorControllerPort);

    // Robot subsystems
    public final Drivetrain drivetrain = new Drivetrain();
    public final Shintake shintake = new Shintake();
    public final Climber climber = new Climber();
    public final RGBLED led = new RGBLED();
    public final Arm arm = new Arm();

    // Cameras
    public final Camera camera = new Camera(new PhotonCamera(VisionConstants.kCameraName), drivetrain);

    private final PhotonCamera drivercam = new PhotonCamera(Constants.VisionConstants.kDriverCameraName);
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Auto commands
        NamedCommands.registerCommand("IntakeSequence", new IntakeSequence());
        NamedCommands.registerCommand("IntakeReset", new IntakeReset());
        NamedCommands.registerCommand("SetShooter", new SetShooter());
        NamedCommands.registerCommand("Shoot", new Shoot());
        NamedCommands.registerCommand("Intake", new Intake());
        NamedCommands.registerCommand("Angle", new PhotonAngleCommand());
        NamedCommands.registerCommand("ResetArm", new ResetPosition());

        // Auto Chooser
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Buttons
        bindings();

        // Default commands
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
        camera.setDefaultCommand(new RunCommand(camera::reset, camera));

        // Shuffleboard/Driver camera
        drivercam.setDriverMode(true);
        CameraServer.startAutomaticCapture().setVideoMode(new VideoMode(1, 320, 240, 60));
    }


    /**
     * Mappings
     */
    private void bindings() {
        // Tag stuff
        driverctl.b().whileTrue(new TagAlign(drivetrain, camera, driverctl));

        // Emergency
        driverctl.rightTrigger().whileTrue(new RunCommand(drivetrain::lock, drivetrain));

        // Climber
        driverctl.povUp().whileTrue(new RightClimbUp());
        driverctl.povDown().whileTrue(new RightRelease());
        driverctl.povRight().whileTrue(new LeftClimbUp());
        driverctl.povLeft().whileTrue(new LeftRelease());

        // Gyro
        driverctl.back().whileTrue(new ResetGyro());

        // Ground intake
        operctl.a().whileTrue(new ParallelCommandGroup(new IntakeSequence(), new Intake()));
        operctl.a().whileFalse(new IntakeReset());

        // Intake and shooter
        operctl.start().whileTrue(new Intake());
        operctl.x().toggleOnTrue(new SetShooter());
        operctl.y().whileTrue(new SpitOut());
        driverctl.leftBumper().onTrue(new Shoot());

        // Arm
        operctl.axisGreaterThan(1, 0.4).whileTrue(new ManualAngleUp());
        operctl.axisLessThan(1, -0.4).whileTrue(new ManualAngleDown());
        operctl.rightBumper().whileTrue(new ManualRetraction());
        operctl.leftBumper().whileTrue(new ManualExtension());

        // Amp Positioning
        operctl.b().whileTrue(new SequentialCommandGroup(new ArmToAmp(arm), new SetShooter()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
