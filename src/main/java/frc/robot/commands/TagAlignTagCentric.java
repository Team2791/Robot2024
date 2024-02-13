package frc.robot.commands;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Three forces acting on the robot.
 * 
 * 1. a system that keeps the robot a set distance away from the apriltag (areactl)
 * 2. a system that moves it horizontally depending on the apriltag's rotation (yawctl)
 * 3. a system that keeps the apriltag in the center by rotating the bot around BOT center (rotctl)
 */
public class TagAlignTagCentric extends Command {
	PhotonCamera camera;
	DriveSubsystem drivetrain;
	AHRS gyro;

	public static final double area_kP = .001;
	public static final double area_kI = .0001;
	public static final double area_kD = .00004;

	public static final double yaw_kP = .001;
	public static final double yaw_kI = .0001;
	public static final double yaw_kD = .00004;

	public static final double rot_kP = .001;
	public static final double rot_kI = .0001;
	public static final double rot_kD = .00004;

	PIDController areactl = new PIDController(area_kP, area_kI, area_kD); // in percentage
	PIDController yawctl = new PIDController(yaw_kP, yaw_kI, yaw_kD); // radians i think
	PIDController rotctl = new PIDController(rot_kP, rot_kI, rot_kD);

	public TagAlignTagCentric(PhotonCamera camera, DriveSubsystem drivetrain) {
		this.camera = camera;
		this.drivetrain = drivetrain;
		this.gyro = DriveSubsystem.m_gyro;
	}

	@Override
	public void initialize() {
		this.areactl.setSetpoint(2);
		this.areactl.setTolerance(0.1);

		this.yawctl.setSetpoint(2 * Math.PI);
		this.yawctl.setTolerance(0.2);

		this.rotctl.setSetpoint(0);
		this.rotctl.setTolerance(camera.getCameraMatrix().get().getNumCols() / 50); // todo: use a constant if this doesn't work
	}

	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		if (!result.hasTargets()) {
			return;
		}

		PhotonTrackedTarget target = result.getBestTarget();

		double area = target.getArea();
		double area_power = areactl.calculate(area);

		double yaw = target.getYaw() - (2 * Math.PI); // camera-relative yaw (-2pi < yaw < 2pi)
		double yaw_power = yawctl.calculate(yaw);

		double av_x = target.getDetectedCorners().stream().mapToDouble(a -> a.x).sum() / 4;
		double x_camrel = av_x - camera.getCameraMatrix().get().getNumCols(); // todo: use a constant if this doesn't work
		double rot_power = rotctl.calculate(x_camrel);

		this.drivetrain.drive(area_power, yaw_power, rot_power, false, true);

		SmartDashboard.putNumber("(Photon) Target Area", area);
		SmartDashboard.putNumber("(Photon) Commanded Area Power (x)", area_power);

		SmartDashboard.putNumber("(Photon) Target Yaw", yaw);
		SmartDashboard.putNumber("(Photon) Commanded Yaw Power (y)", yaw_power);

		SmartDashboard.putNumber("(Photon) Camera-Relative Average X of Target", av_x);
		SmartDashboard.putNumber("(Photon) Commanded Rotation Power", rot_power);
	}
}
