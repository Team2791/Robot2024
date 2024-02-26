package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.MAXSwerveModule;
import frc.robot.swerve.SwerveUtils;

import java.util.Arrays;

public class Drivetrain extends SubsystemBase {
	// The gyro sensor
	private final AHRS gyro;

	// Create MAXSwerveModules
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
	    Constants.Ids.Drive.FrontLeft,
	    Constants.Ids.Turn.FrontLeft,
	    Constants.Drive.Dimensions.AngularOffsetFrontLeft
	);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
	    Constants.Ids.Drive.FrontRight,
	    Constants.Ids.Turn.FrontRight,
	    Constants.Drive.Dimensions.AngularOffsetFrontRight
	);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
	    Constants.Ids.Drive.RearLeft,
	    Constants.Ids.Turn.RearLeft,
	    Constants.Drive.Dimensions.AngularOffsetRearLeft
	);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
	    Constants.Ids.Drive.RearRight,
	    Constants.Ids.Turn.RearRight,
	    Constants.Drive.Dimensions.AngularOffsetRearRight
	);

	private final SlewRateLimiter magLimiter = new SlewRateLimiter(Constants.Drive.Slew.Magnitude);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Drive.Slew.Rotational);

	// Odometry class for tracking robot pose
	private final SwerveDriveOdometry odometry;

	// Slew rate filter variables for controlling lateral acceleration
	private double translationDir = 0.0;
	private double translationMag = 0.0;
	private double lastRun = WPIUtilJNI.now() * 1e-6;

	/**
	 * Creates a new DriveSubsystem.
	 */
	public Drivetrain(AHRS gyro) {
		this.gyro = gyro;

		this.odometry = new SwerveDriveOdometry(
		    Constants.Drive.Kinematics,
		    Rotation2d.fromDegrees(gyro.getAngle()),
		    new SwerveModulePosition[]{
		        frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
			}
		);

		AutoBuilder.configureHolonomic(
		    this::pose,
		    this::reset,
		    this::speeds,
		    this::drive,
		    new HolonomicPathFollowerConfig(
		        new PIDConstants(
		            Constants.PID.Auto.Translation.P,
		            Constants.PID.Auto.Translation.I,
		            Constants.PID.Auto.Translation.D
		        ),
		        new PIDConstants(
		            Constants.PID.Auto.Rotation.P,
		            Constants.PID.Auto.Rotation.I,
		            Constants.PID.Auto.Rotation.D
		        ),
		        Constants.Drive.Limits.MaxSpeed,
		        Constants.Drive.Dimensions.DriveBaseRadius,
		        new ReplanningConfig()
		    ),
		    () -> {
			    // Boolean supplier that controls when the path will be mirrored for the red
			    // alliance
			    // This will flip the path being followed to the red side of the field.
			    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
			    // var alliance = DriverStation.getAlliance();
			    //if (alliance == null || alliance != null) {
			    //	return alliance == DriverStation.Alliance.Red;
			    //}

			    return false;
		    },
		    this
		);

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	@Override
	public void periodic() {
		odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[]{
		    frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
		});
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d pose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void reset(Pose2d to) {
		odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), new SwerveModulePosition[]{
		    frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
		}, to);
	}

	/**
	 * Drives the robot at given speeds. Not field relative. Not rate limited.
	 *
	 * @param speeds The speeds at which to set the wheels.
	 */
	public void drive(ChassisSpeeds speeds) {
		this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
	}

	/**
	 * @return Robot-relative chassis speeds
	 */
	public ChassisSpeeds speeds() {
		return Constants.Drive.Kinematics.toChassisSpeeds(
		    frontLeft.getState(),
		    frontRight.getState(),
		    rearLeft.getState(),
		    rearRight.getState()
		);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param x             Speed of the robot in the x direction (forward).
	 * @param y             Speed of the robot in the y direction (sideways).
	 * @param r             Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double x, double y, double r, boolean fieldRelative, boolean rateLimit) {
		double xPower = x;
		double yPower = y;
		double rPower = r;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double translationDirRaw = Math.atan2(y, x);
			double translationMagRaw = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			// Avoid division by zero. Math.max(...) is safe because Math.sqrt(...) is always positive
			double directionSlew = Math.abs(Constants.Drive.Slew.Direction / Math.max(0.001, translationMagRaw));

			double now = WPIUtilJNI.now() * 1e-6;
			double elapsed = now - lastRun;
			double angleDif = SwerveUtils.AngleDifference(translationDirRaw, this.translationDir);

			if (angleDif < 0.45 * Math.PI) {
				this.translationDir = SwerveUtils.StepTowardsCircular(
				    this.translationDir,
				    translationDirRaw,
				    directionSlew * elapsed
				);

				this.translationMag = magLimiter.calculate(translationMagRaw);
			} else if (angleDif > 0.85 * Math.PI) {
				// Round to 3 decimal places to avoid floating point errors
				this.translationMag = Math.round(translationMagRaw * 1000.0) / 1000.0;

				if (this.translationMag == 0.0) {
					this.translationMag = magLimiter.calculate(0.0);
				} else {
					this.translationDir = SwerveUtils.WrapAngle(translationDir + Math.PI);
					this.translationMag = magLimiter.calculate(translationMagRaw);
				}
			} else {
				this.translationDir = SwerveUtils.StepTowardsCircular(
				    translationDir,
				    translationDirRaw,
				    directionSlew * elapsed
				);

				this.translationMag = magLimiter.calculate(0.0);
			}

			lastRun = now;

			xPower = translationMag * Math.cos(translationDir);
			yPower = translationMag * Math.sin(translationDir);
			rPower = rotLimiter.calculate(r);
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		xPower *= Constants.Drive.Limits.MaxSpeed;
		yPower *= Constants.Drive.Limits.MaxSpeed;
		rPower *= Constants.Drive.Limits.MaxAngularSpeed;

		ChassisSpeeds speeds = fieldRelative
		    ? ChassisSpeeds.fromFieldRelativeSpeeds(xPower, yPower, rPower, Rotation2d.fromDegrees(-gyro.getAngle()))
		    : new ChassisSpeeds(xPower, yPower, rPower);


		this.setModuleStates(Constants.Drive.Kinematics.toSwerveModuleStates(speeds));
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void stopX() {
		frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.Limits.MaxSpeed);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);
	}

	/**
	 * Stops each individual module
	 */
	public void stop() {
		frontLeft.stop();
		frontRight.stop();
		rearLeft.stop();
		rearRight.stop();
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void reset() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}


	public SwerveModulePosition[] getModulePositions() {
		MAXSwerveModule[] mods = new MAXSwerveModule[]{
		    frontLeft, frontRight, rearLeft, rearRight
		};

		return Arrays.stream(mods).map(MAXSwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
	}
}