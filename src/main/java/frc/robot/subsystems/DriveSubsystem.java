// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	// Create MAXSwerveModules
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
			DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
			DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
			DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
			DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
			DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
			DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
			DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
			DriveConstants.kBackRightChassisAngularOffset);

	// The gyro sensor
	private static AHRS m_gyro = new AHRS(Port.kMXP);

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	public static PIDConstants trans = new PIDConstants(3, 0.02, 0.0315); // 5, 0.02, 0.03235
	public static PIDConstants rot = new PIDConstants(0.75, 0.015, 0.5);

	private Field2d field;

	// Odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
			Rotation2d.fromDegrees(m_gyro.getAngle()),
			new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
					m_rearLeft.getPosition(), m_rearRight.getPosition()});

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		field = new Field2d();

		m_gyro.reset();
		auto();
	}

	public void auto() {
		try {
			AutoBuilder.class.getDeclaredField("configured").set(new AutoBuilder(), false);
		} catch (Exception e) {
		}

		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
						trans, rot, // Constants class
						// Rotation PID constants
						1, // Max module speed, in m/s
						0.38608, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				), () -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					//if (alliance == null || alliance != null) {
					//	return alliance == DriverStation.Alliance.Red;
					//}
					return false;
				}, this // Reference to this subsystem to set requirements
		);
	}

	@Override
	public void periodic() {


		// Update the odometry in the periodic block
		m_odometry.update(m_gyro.getRotation2d(),
				new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_rearLeft.getPosition(), m_rearRight.getPosition()});
		SmartDashboard.putNumber("yaw:", m_gyro.getYaw());
		SmartDashboard.putNumber("angle:", m_gyro.getAngle());
		SmartDashboard.putString("Robot X", m_odometry.getPoseMeters().toString());
		putField(m_odometry.getPoseMeters());
		SmartDashboard.putData("Field", field);

	}

	public void putField(Pose2d pose) {
		field.setRobotPose(pose);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle()),
				new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_rearLeft.getPosition(), m_rearRight.getPosition()},
				pose);
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
				false, false);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
				m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
			boolean rateLimit) {

		// Exponential transform for smoother translation control
		// Joystick output and motor input are both -1.0 to 1.0
		// So both J and M in the math are 1, which simplifies.

		double exponent = 5.0;

		double Y = xSpeed;
		double X = ySpeed;

		double Ysign = Math.signum(Y);
		double Xsign = Math.signum(X);

		Y = Math.abs(Y);
		X = Math.abs(X);

		xSpeed = Ysign * Math.pow(Y, exponent);
		ySpeed = Xsign * Math.pow(X, exponent);

		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral
			// acceleration
			double directionSlewRate;
			if (m_currentTranslationMag != 0.0) {
				directionSlewRate =
						Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
			}

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif =
					SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
			if (angleDif < 0.45 * Math.PI) {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
						inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > 0.85 * Math.PI) {
				if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
														// checking
														// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				} else {
					m_currentTranslationDir =
							SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			} else {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
						inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}
			m_prevTime = currentTime;

			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
						rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
				DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	public void stopModules() {
		m_frontLeft.stop();
		m_frontRight.stop();
		m_rearLeft.stop();
		m_rearRight.stop();
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 * 
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}
}
