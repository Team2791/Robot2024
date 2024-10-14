package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmCommands.ArmToGround;
import frc.robot.commands.ArmCommands.ManualCommands.ManualAngle;
import frc.robot.commands.ArmCommands.ManualCommands.ManualExtend;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.ClimberCommands.ManualClimb;
import frc.robot.commands.ClimberCommands.ManualClimb.ClimbSide;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.commands.ShintakeCommands.Outtake;
import frc.robot.commands.ShintakeCommands.SetShooter;
import frc.robot.commands.ShintakeCommands.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shintake;
import frc.robotkt.constants.IOConstants;
import frc.robotkt.subsystems.Arm;
import frc.robotkt.subsystems.Drivetrain;

public class RobotContainer {
    // Controllers
    final CommandXboxController driverctl = new CommandXboxController(IOConstants.Controller.kDriverPort);
    final CommandXboxController operctl = new CommandXboxController(IOConstants.Controller.kOperatorPort);

    // Robot subsystems
    final Drivetrain drivetrain = new Drivetrain();
    final Shintake shintake = new Shintake();
    final Climber climber = new Climber(drivetrain);
    final Led led = new Led();
    final Arm arm = new Arm();

    // Cameras
    // final Camera camera = new Camera(new PhotonCamera(VisionConstants.kCameraName), drivetrain);

    // Auto
    final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Auto commands
        NamedCommands.registerCommand("SetShooter", new SetShooter(shintake, driverctl));
        NamedCommands.registerCommand("Shoot", new Shoot(shintake));
        autoChooser = AutoBuilder.buildAutoChooser();

        // Auto Chooser
        ShuffleboardTab tab = Shuffleboard.getTab("Competition");
        tab.add("Choose Auto", autoChooser);

        // Buttons
        bindings();

        // Default commands
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
        // camera.setDefaultCommand(new RunCommand(camera::reset, camera));
    }

    private void bindings() {
        // Emergency
        driverctl.rightTrigger().whileTrue(new RunCommand(drivetrain::lock, drivetrain));

        // Individual Climber Control
        driverctl.povUp().whileTrue(new ManualClimb(climber, ClimbSide.kLeft, true));
        driverctl.povDown().whileTrue(new ManualClimb(climber, ClimbSide.kLeft, false));
        driverctl.povRight().whileTrue(new ManualClimb(climber, ClimbSide.kRight, true));
        driverctl.povLeft().whileTrue(new ManualClimb(climber, ClimbSide.kRight, false));

        // Climber
        driverctl.y().whileTrue(new ManualClimb(climber, ClimbSide.kBoth, true));
        driverctl.a().whileTrue(new ManualClimb(climber, ClimbSide.kBoth, false));

        // Gyro
        driverctl.back().whileTrue(new ResetGyro(drivetrain));

        // Ground intake
        operctl.a().whileTrue(new ParallelCommandGroup(new ArmToGround(arm), new Intake(shintake, led, driverctl, operctl)));
        operctl.a().whileFalse(new ResetArm(arm));

        // Intake and shooter
        operctl.start().whileTrue(new Intake(shintake, led));
        operctl.x().toggleOnTrue(new SetShooter(shintake, driverctl));
        operctl.y().whileTrue(new Outtake(shintake));
        driverctl.leftBumper().whileTrue(new Shoot(shintake));

        // Arm
        operctl.axisLessThan(1, -0.4).whileTrue(new ManualAngle(arm, true));
        operctl.axisGreaterThan(1, 0.4).whileTrue(new ManualAngle(arm, false));
        operctl.rightBumper().whileTrue(new ManualExtend(arm, true));
        operctl.leftBumper().whileTrue(new ManualExtend(arm, false));

        // Amp Positioning
        operctl.b().whileTrue(new Intake(shintake, led, driverctl, operctl));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
