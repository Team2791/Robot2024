package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TagAlignAll extends Command {
	PhotonCamera camera;
	DriveSubsystem drivetrain;
	AHRS gyro;
	PIDController pid_rotation =
			new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);

	PIDController pid_x =
			new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);

	PIDController pid_y =
			new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);

	public TagAlignAll(PhotonCamera camera, DriveSubsystem drivetrain) {
		this.camera = camera;
		this.drivetrain = drivetrain;
		this.gyro = DriveSubsystem.m_gyro;
	}

	@Override
	public void initialize() {
		this.pid_rotation.setSetpoint(0); // changed to be camera-relative
		this.pid_rotation.setTolerance(0.1); // in RADIANS

		this.pid_x.setSetpoint(2); // TODO: Callibrate: Distance away from tag
		this.pid_y.setSetpoint(2); // TODO: Callibrate: Distance away from tag
	}

	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets())
			return;

		Transform3d transform = result.getBestTarget().getBestCameraToTarget();
		Translation2d linear = transform.getTranslation().toTranslation2d();

		double rotation = transform.getRotation().getZ();
		double x = linear.getX();
		double y = linear.getY();

		double rotation_power = this.pid_rotation.calculate(rotation);
		double x_power = this.pid_x.calculate(x);
		double y_power = this.pid_y.calculate(y);

		drivetrain.drive(x_power, y_power, rotation_power, false, false);

		SmartDashboard.putNumber("(X) Distance from tag", x);
		SmartDashboard.putNumber("(Y) Distance from tag", y);
		SmartDashboard.putNumber("(R) Radians from tag", rotation);

		SmartDashboard.putNumber("(X) Power", x_power);
		SmartDashboard.putNumber("(Y) Power", y_power);
		SmartDashboard.putNumber("(R) Power", rotation_power);
	}
}
