package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robotkt.constants.PidConstants;
import frc.robotkt.subsystems.Camera;
import frc.robotkt.subsystems.Drivetrain;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Predicate;

public class TagAlign extends Command {
    final PIDController pid = new PIDController(PidConstants.PhotonAlign.kP, PidConstants.PhotonAlign.kI, PidConstants.PhotonAlign.kD);
    final Drivetrain drivetrain;
    final Camera camera;
    final CommandXboxController driverctl;
    final Predicate<PhotonTrackedTarget> filter;

    // make the target accessible to not recalculate. see AngleShooter.java
    PhotonTrackedTarget target;

    public TagAlign(Drivetrain drivetrain, Camera camera, CommandXboxController driverctl, Set<Integer> targetIds) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.driverctl = driverctl;
        this.filter = (target) -> targetIds.contains(target.getFiducialId());

        pid.setTolerance(1);
        pid.setSetpoint(300);
        camera.setMode(Camera.CameraMode.AprilTag);

        addRequirements(drivetrain, camera);
    }

    public TagAlign(Drivetrain drivetrain, Camera camera, CommandXboxController driverctl) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.driverctl = driverctl;
        this.filter = (_t) -> true;

        pid.setTolerance(1);
        pid.setSetpoint(300);
        camera.setMode(Camera.CameraMode.AprilTag);

        addRequirements(drivetrain, camera);
    }

    public void execute() {
        if (!camera.hasTargets()) {
            // We are capturing the drivetrain, so make sure we default drive
            drivetrain.drive(driverctl);
            return;
        }

        List<PhotonTrackedTarget> targets = camera.getTargets();
        Optional<PhotonTrackedTarget> target = targets.stream().filter(t -> this.filter.test(t) && t.getPoseAmbiguity() <= 0.6).findFirst();
        if (target.isEmpty()) return;

        this.target = target.get();
        double yaw = this.target.getYaw();
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
