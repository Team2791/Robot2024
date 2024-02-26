package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class MAXSwerveModule {
	private final CANSparkMax driving;
	private final CANSparkMax turning;

	private final RelativeEncoder drivingEncoder;
	private final AbsoluteEncoder turningEncoder;

	private final SparkPIDController drivectl;
	private final SparkPIDController turnctl;

	private double angularOffset = 0;
	private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int drivingId, int turningId, double angularOffset) {
		driving = new CANSparkMax(drivingId, MotorType.kBrushless);
		turning = new CANSparkMax(turningId, MotorType.kBrushless);

		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		driving.restoreFactoryDefaults();
		turning.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		drivingEncoder = driving.getEncoder();
		turningEncoder = turning.getAbsoluteEncoder(Type.kDutyCycle);
		drivectl = driving.getPIDController();
		turnctl = turning.getPIDController();
		drivectl.setFeedbackDevice(drivingEncoder);
		turnctl.setFeedbackDevice(turningEncoder);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		drivingEncoder.setPositionConversionFactor(Constants.Module.Encoder.DrivingPositionFactor);
		drivingEncoder.setVelocityConversionFactor(Constants.Module.Encoder.DrivingVelocityFactor);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		turningEncoder.setPositionConversionFactor(Constants.Module.Encoder.TurningPositionFactor);
		turningEncoder.setVelocityConversionFactor(Constants.Module.Encoder.TurningVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		turningEncoder.setInverted(Constants.Module.Encoder.TurningInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		turnctl.setPositionPIDWrappingEnabled(true);
		turnctl.setPositionPIDWrappingMinInput(Constants.Module.Encoder.TurningPositionPIDMin);
		turnctl.setPositionPIDWrappingMaxInput(Constants.Module.Encoder.TurningPositionPIDMax);

		// Set the PID gains for the driving motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		drivectl.setP(Constants.PID.Module.Driving.P);
		drivectl.setI(Constants.PID.Module.Driving.I);
		drivectl.setD(Constants.PID.Module.Driving.D);
		drivectl.setFF(Constants.PID.Module.Driving.FF);
		drivectl.setOutputRange(Constants.PID.Module.Driving.MinOutput, Constants.PID.Module.Driving.MaxOutput);

		// Set the PID gains for the turning motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		turnctl.setP(Constants.PID.Module.Turning.P);
		turnctl.setI(Constants.PID.Module.Turning.I);
		turnctl.setD(Constants.PID.Module.Turning.D);
		turnctl.setFF(Constants.PID.Module.Turning.FF);
		turnctl.setOutputRange(Constants.PID.Module.Turning.MinOutput, Constants.PID.Module.Turning.MaxOutput);

		driving.setIdleMode(Constants.Module.Idle.Driving);
		turning.setIdleMode(Constants.Module.Idle.Turning);
		driving.setSmartCurrentLimit(Constants.Module.Limits.DrivingMotorCurrent);
		turning.setSmartCurrentLimit(Constants.Module.Limits.TurningMotorCurrent);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		driving.burnFlash();
		turning.burnFlash();

		this.angularOffset = angularOffset;
		desiredState.angle = new Rotation2d(turningEncoder.getPosition());
		drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(
		    drivingEncoder.getVelocity(),
		    new Rotation2d(turningEncoder.getPosition() - angularOffset)
		);
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
		    drivingEncoder.getPosition(),
		    new Rotation2d(turningEncoder.getPosition() - angularOffset)
		);
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
		    correctedDesiredState,
		    new Rotation2d(turningEncoder.getPosition())
		);

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		drivectl.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
		turnctl.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

		this.desiredState = desiredState;
	}

	public void stop() {
		driving.set(0);
		turning.set(0);
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		drivingEncoder.setPosition(0);
	}
}
