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
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	// Create MAXSwerveModules
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(DriveConstants.CanIds.kNwDrive, DriveConstants.CanIds.kNwTurn, DriveConstants.Chassis.kNwAngularOffset);
	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(DriveConstants.CanIds.kNeDrive, DriveConstants.CanIds.kNeTurn, DriveConstants.Chassis.kNeAngularOffset);
	private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(DriveConstants.CanIds.kSwDrive, DriveConstants.CanIds.kSwTurn, DriveConstants.Chassis.kSwAngularOffset);
	private final MAXSwerveModule m_rearRight = new MAXSwerveModule(DriveConstants.CanIds.kSeDrive, DriveConstants.CanIds.kSeTurn, DriveConstants.Chassis.kSeAngularOffset);

	// The gyro sensor
	private static AHRS m_gyro = new AHRS(Port.kMXP);

	// Slew rate filter variables for controlling lateral acceleration
	private double m_rotation = 0.0;
	private double m_translationDir = 0.0;
	private double m_translationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.Slew.kMagnitude);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.Slew.kRotational);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.Driving.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), m_modulePositions);

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		m_gyro.reset();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()});

		// Update SmartDashboard
		SmartDashboard.putNumber("yaw:", m_gyro.getYaw());
		SmartDashboard.putNumber("angle:", m_gyro.getAngle());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
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
	 * reset module positions
	 */
	public void resetModulePositions() {
		m_modulePositions = new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		resetModulePositions();

		m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle()), m_modulePositions, pose);
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
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
		/* 
		 * Exponential transform for smoother translation control
		 * Joystick output and motor input are both -1.0 to 1.0
		 * So both J and M in the math are 1, which simplifies. 
		 */

		double exponent = 3.0;

		xSpeed = Math.signum(xSpeed) * Math.pow(Math.abs(xSpeed), exponent);
		ySpeed = Math.signum(ySpeed) * Math.pow(Math.abs(ySpeed), exponent);

		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_translationMag != 0.0) {
				directionSlewRate = Math.abs(DriveConstants.Slew.kDirection / m_translationMag);
			} else {
				// make the slew rate instantaneous if the robot is stopped
				directionSlewRate = 500.0;
			}


			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_translationDir);

			if (angleDif < 0.45 * Math.PI) {
				m_translationDir = SwerveUtils.StepTowardsCircular(m_translationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_translationMag = m_magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > 0.85 * Math.PI) {
				// some small number to avoid floating-point errors with equality checking
				if (m_translationMag > 1e-4) {
					m_translationMag = m_magLimiter.calculate(0.0);
				} else {
					m_translationDir = SwerveUtils.WrapAngle(m_translationDir + Math.PI);
					m_translationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			} else {
				m_translationDir = SwerveUtils.StepTowardsCircular(m_translationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_translationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;

			xSpeedCommanded = m_translationMag * Math.cos(m_translationDir);
			ySpeedCommanded = m_translationMag * Math.sin(m_translationDir);
			m_rotation = m_rotLimiter.calculate(rot);
		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_rotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.Driving.kMaxSpeed;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.Driving.kMaxSpeed;
		double rotDelivered = m_rotation * DriveConstants.Driving.kMaxAngularSpeed;

		ChassisSpeeds speeds;

		// If field relative, convert the desired x and y speeds into field-relative
		if (fieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()));
		} else {
			speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
		}

		SwerveModuleState[] states = DriveConstants.Driving.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.Driving.kMaxSpeed);

		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_rearLeft.setDesiredState(states[2]);
		m_rearRight.setDesiredState(states[3]);
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
	 * @param desired The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desired) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desired, DriveConstants.Driving.kMaxSpeed);
		m_frontLeft.setDesiredState(desired[0]);
		m_frontRight.setDesiredState(desired[1]);
		m_rearLeft.setDesiredState(desired[2]);
		m_rearRight.setDesiredState(desired[3]);
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
