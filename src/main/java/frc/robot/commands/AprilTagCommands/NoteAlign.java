package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robotkt.constants.PidConstants;
import frc.robotkt.subsystems.Camera;
import frc.robotkt.subsystems.Drivetrain;

public class NoteAlign extends Command {
    final PIDController pid = new PIDController(PidConstants.PhotonAlign.kP, PidConstants.PhotonAlign.kI, PidConstants.PhotonAlign.kD);
    final Drivetrain drivetrain;
    final Camera camera;
    final CommandXboxController driverctl;

    public NoteAlign(Drivetrain drivetrain, Camera camera, CommandXboxController driverctl) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.driverctl = driverctl;

        camera.setMode(Camera.CameraMode.Note);

        pid.setTolerance(1);
        pid.setSetpoint(300);

        addRequirements(drivetrain);
        addRequirements(camera);
    }

    public void execute() {
        if (!camera.hasTargets()) {
            // We are capturing the drivetrain, so make sure we default drive
            drivetrain.drive(driverctl);
            return;
        }

        double yaw = camera.getBestTarget().getYaw();
        double output = pid.calculate(yaw);

        drivetrain.drive(driverctl, -output);
    }

    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
