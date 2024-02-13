// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.MAXSwerveModule;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	// Create MAXSwerveModules
	private final MAXSwerveModule front_left = new MAXSwerveModule(
			DriveConstants.CANId.Driving.FRONT_LEFT,
			DriveConstants.CANId.Turning.FRONT_LEFT,
			DriveConstants.AngularOffset.FRONT_LEFT);

	private final MAXSwerveModule front_right = new MAXSwerveModule(
			DriveConstants.CANId.Driving.FRONT_RIGHT,
			DriveConstants.CANId.Turning.FRONT_RIGHT,
			DriveConstants.AngularOffset.FRONT_RIGHT);

	private final MAXSwerveModule rear_left = new MAXSwerveModule(
			DriveConstants.CANId.Driving.REAR_LEFT,
			DriveConstants.CANId.Turning.REAR_LEFT,
			DriveConstants.AngularOffset.REAR_LEFT);

	private final MAXSwerveModule rear_right = new MAXSwerveModule(
			DriveConstants.CANId.Driving.REAR_RIGHT,
			DriveConstants.CANId.Turning.REAR_RIGHT,
			DriveConstants.AngularOffset.REAR_RIGHT);

	// The gyro sensor
	private static AHRS gyro = new AHRS(Port.kMXP);

	// Slew rate filter variables for controlling lateral acceleration
	private double rot_power = 0.0;
	private double translation_dir = 0.0;
	private double current_translation_mag = 0.0;

	private SlewRateLimiter mag_limiter = new SlewRateLimiter(DriveConstants.SLEW_MAGNITUDE);
	private SlewRateLimiter rot_limiter = new SlewRateLimiter(DriveConstants.SLEW_ROTATIONAL);
	private double previous_time = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			DriveConstants.KINEMATICS,
			Rotation2d.fromDegrees(gyro.getAngle()),
			new SwerveModulePosition[] {
					front_left.getPosition(),
					front_right.getPosition(),
					rear_left.getPosition(),
					rear_right.getPosition()
			});

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		gyro.reset();
	}

	@Override
	public void periodic() {

		// Update the odometry in the periodic block
		odometry.update(
				Rotation2d.fromDegrees(gyro.getAngle()),
				new SwerveModulePosition[] {
						front_left.getPosition(),
						front_right.getPosition(),
						rear_left.getPosition(),
						rear_right.getPosition()
				});

		SmartDashboard.putNumber("(Drivetrain/Gyro) Yaw", gyro.getYaw());
		SmartDashboard.putNumber("(Drivetrain/Gyro) Angle", gyro.getAngle());
		SmartDashboard.putString("(Drivetrain) Pose", getPose().getTranslation().toString());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose
	 *            The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
				Rotation2d.fromDegrees(gyro.getAngle()),
				new SwerveModulePosition[] {
						front_left.getPosition(),
						front_right.getPosition(),
						rear_left.getPosition(),
						rear_right.getPosition()
				},
				pose);
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.KINEMATICS.toChassisSpeeds(front_left.getState(),
				front_right.getState(),
				rear_left.getState(),
				rear_right.getState());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param x
	 *            Speed of the robot in the x direction (forward).
	 * @param y
	 *            Speed of the robot in the y direction (sideways).
	 * @param rot
	 *            Angular rate of the robot.
	 * @param fieldRelative
	 *            Whether the provided x and y speeds are relative to the
	 *            field.
	 * @param rateLimit
	 *            Whether to enable rate limiting for smoother control.
	 */
	public void drive(double x, double y, double rot, boolean fieldRelative, boolean rateLimit) {

		// Exponential transform for smoother translation control
		// Joystick output and motor input are both -1.0 to 1.0
		// So both J and M in the math are 1, which simplifies.

		double exponent = 5.0;

		x = Math.signum(x) * Math.pow(Math.abs(x), exponent);
		y = Math.signum(y) * Math.pow(Math.abs(y), exponent);

		double x_power;
		double y_power;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double input_translation_dir = Math.atan2(y, x);
			double inputTranslationMag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

			// Calculate the direction slew rate based on an estimate of the lateral
			// acceleration
			double dir_slew;
			if (current_translation_mag != 0.0) {
				dir_slew = Math.abs(DriveConstants.SLEW_DIRECTIONAL / current_translation_mag);
			} else {
				dir_slew = 500.0; // some high number that means the slew rate is effectively instantaneous
			}

			double time = WPIUtilJNI.now() * 1e-6;
			double elapsed = time - previous_time;
			double angleDif = SwerveUtils.AngleDifference(input_translation_dir, translation_dir);

			if (angleDif < 0.45 * Math.PI) {
				translation_dir = SwerveUtils.StepTowardsCircular(translation_dir, input_translation_dir,
						dir_slew * elapsed);
				current_translation_mag = mag_limiter.calculate(inputTranslationMag);
			} else if (angleDif > 0.85 * Math.PI) {
				if (current_translation_mag > 1e-4) {
					current_translation_mag = mag_limiter.calculate(0.0);
				} else {
					translation_dir = SwerveUtils.WrapAngle(translation_dir + Math.PI);
					current_translation_mag = mag_limiter.calculate(inputTranslationMag);
				}
			} else {
				translation_dir = SwerveUtils.StepTowardsCircular(translation_dir, input_translation_dir,
						dir_slew * elapsed);

				current_translation_mag = mag_limiter.calculate(0.0);
			}
			previous_time = time;

			x_power = current_translation_mag * Math.cos(translation_dir);
			y_power = current_translation_mag * Math.sin(translation_dir);
			rot_power = rot_limiter.calculate(rot);

		} else {
			x_power = x;
			y_power = y;
			rot_power = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double x_power_ltd = x_power * DriveConstants.MAX_SPEED;
		double y_power_ltd = y_power * DriveConstants.MAX_SPEED;
		double rot_power_ltd = rot_power * DriveConstants.MAX_SPEED_ANGULAR;

		var states = DriveConstants.KINEMATICS.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(x_power_ltd, y_power_ltd, rot_power_ltd,
								Rotation2d.fromDegrees(-gyro.getAngle()))
						: new ChassisSpeeds(x_power_ltd, y_power_ltd, rot_power_ltd));
		SwerveDriveKinematics.desaturateWheelSpeeds(
				states, DriveConstants.MAX_SPEED);
		front_left.setDesiredState(states[0]);
		front_right.setDesiredState(states[1]);
		rear_left.setDesiredState(states[2]);
		rear_right.setDesiredState(states[3]);
	}

	/**
	 * Sets the wheels into an x formation to prevent movement.
	 */
	public void setX() {
		front_left.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		front_right.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rear_left.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rear_right.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates
	 *            The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, DriveConstants.MAX_SPEED);
		front_left.setDesiredState(desiredStates[0]);
		front_right.setDesiredState(desiredStates[1]);
		rear_left.setDesiredState(desiredStates[2]);
		rear_right.setDesiredState(desiredStates[3]);
	}

	public void stopModules() {
		front_left.stop();
		front_right.stop();
		rear_left.stop();
		rear_right.stop();
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		front_left.resetEncoders();
		rear_left.resetEncoders();
		front_right.resetEncoders();
		rear_right.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 * 
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
	}
}