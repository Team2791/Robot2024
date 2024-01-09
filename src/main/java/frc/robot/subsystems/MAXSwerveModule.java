// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
	private final CANSparkMax m_drivingMotor;
	private final CANSparkMax m_turningMotor;

	private final RelativeEncoder m_drivingEncoder;
	private final AbsoluteEncoder m_turningEncoder;

	private final SparkMaxPIDController m_drivingPID;
	private final SparkMaxPIDController m_turningPID;

	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int driving_id, int turning_id, double chassisAngularOffset) {
		m_drivingMotor = new CANSparkMax(driving_id, MotorType.kBrushless);
		m_turningMotor = new CANSparkMax(turning_id, MotorType.kBrushless);

		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		m_drivingMotor.restoreFactoryDefaults();
		m_turningMotor.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingMotor.getEncoder();
		m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
		m_drivingPID = m_drivingMotor.getPIDController();
		m_turningPID = m_turningMotor.getPIDController();
		m_drivingPID.setFeedbackDevice(m_drivingEncoder);
		m_turningPID.setFeedbackDevice(m_turningEncoder);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
		m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
		m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		m_turningPID.setPositionPIDWrappingEnabled(true);
		m_turningPID.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
		m_turningPID.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

		// Set the PID gains for the driving motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		m_drivingPID.setP(ModuleConstants.kDrivingP);
		m_drivingPID.setI(ModuleConstants.kDrivingI);
		m_drivingPID.setD(ModuleConstants.kDrivingD);
		m_drivingPID.setFF(ModuleConstants.kDrivingFF);
		m_drivingPID.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

		// Set the PID gains for the turning motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		m_turningPID.setP(ModuleConstants.kTurningP);
		m_turningPID.setI(ModuleConstants.kTurningI);
		m_turningPID.setD(ModuleConstants.kTurningD);
		m_turningPID.setFF(ModuleConstants.kTurningFF);
		m_turningPID.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

		m_drivingMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
		m_turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
		m_drivingMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
		m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		m_drivingMotor.burnFlash();
		m_turningMotor.burnFlash();

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		Rotation2d ang = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
		double mps = desiredState.speedMetersPerSecond;
		SwerveModuleState corrected = new SwerveModuleState(mps, ang);

		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimized = SwerveModuleState.optimize(corrected, new Rotation2d(m_turningEncoder.getPosition()));

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingPID.setReference(optimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
		m_turningPID.setReference(optimized.angle.getRadians(), CANSparkMax.ControlType.kPosition);

		m_desiredState = desiredState;
	}

	public void stop() {
		m_drivingMotor.set(0);
		m_turningMotor.set(0);
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingEncoder.setPosition(0);
	}
}
