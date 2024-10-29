package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FunctionalCommands.ExecuteCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShooterCommands.Intake;
import frc.robot.commands.ShooterCommands.SetShooter;
import frc.robot.subsystems.Shooter;
import frc.robotkt.constants.IOConstants;
import frc.robotkt.subsystems.Drivetrain;
import frc.robotkt.subsystems.Notifier;

import java.util.concurrent.atomic.AtomicBoolean;

public class RobotContainer {
    // Controllers
    final CommandXboxController driverctl = new CommandXboxController(IOConstants.Controller.kDriverPort);

    // Robot subsystems
    final Drivetrain drivetrain = new Drivetrain();
    final Notifier notifier = new Notifier(driverctl);
    final Shooter shooter = new Shooter();

    // Auto
    final SendableChooser<Command> autoChooser;

    // Shooter and shoot command (signaling)
    AtomicBoolean shootSignal = new AtomicBoolean(false);
    Command setShooter = new SetShooter(shooter, shootSignal);
    Command shoot = new ExecuteCommand(() -> shootSignal.set(true));

    public RobotContainer() {
        // Auto commands
        NamedCommands.registerCommand("SetShooter", setShooter);
        NamedCommands.registerCommand("Shoot", shoot);
        autoChooser = AutoBuilder.buildAutoChooser();

        // Auto Chooser
        ShuffleboardTab tab = Shuffleboard.getTab("Competition");
        tab.add("Choose Auto", autoChooser);

        // Buttons
        bindings();

        // Default commands
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
    }

    private void bindings() {
        // Gyro
        driverctl.back().whileTrue(new ResetGyro(drivetrain));

        // Intake and shooter
        driverctl.rightBumper().whileTrue(new Intake(shooter, notifier));
        driverctl.leftBumper().onTrue(shoot);
        driverctl.x().onTrue(new SetShooter(shooter, shootSignal));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
