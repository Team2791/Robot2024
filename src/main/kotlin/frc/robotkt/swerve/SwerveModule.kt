package frc.robotkt.swerve

import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkAbsoluteEncoder
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robotkt.constants.ModuleConstants
import frc.robotkt.constants.PidConstants

class SwerveModule(driveId: Int, turnId: Int, val angularOffset: Double) {
    val driveMotor = CANSparkMax(driveId, MotorType.kBrushless)
    val turnMotor = CANSparkMax(turnId, MotorType.kBrushless)

    val driveEncoder: RelativeEncoder
    val turnEncoder: SparkAbsoluteEncoder

    val drivePid: SparkPIDController
    val turnPid: SparkPIDController

    var desiredState = SwerveModuleState(0.0, Rotation2d())
        set(desired) {
            val corrected = SwerveModuleState()
            corrected.speedMetersPerSecond = desired.speedMetersPerSecond
            corrected.angle = desired.angle.plus(Rotation2d(angularOffset))

            val optimized = SwerveModuleState.optimize(corrected, Rotation2d(turnEncoder.position))!!
            drivePid.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity)
            turnPid.setReference(optimized.angle.radians, ControlType.kPosition)

            field = desired
        }

    init {
        driveMotor.restoreFactoryDefaults()
        turnMotor.restoreFactoryDefaults()

        // Setup encoders and PID controllers
        driveEncoder = driveMotor.encoder
        turnEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

        drivePid = driveMotor.getPIDController()
        turnPid = turnMotor.getPIDController()

        drivePid.setFeedbackDevice(driveEncoder)
        turnPid.setFeedbackDevice(turnEncoder)

        // Apply position and velocity conversion factors
        driveEncoder.positionConversionFactor = ModuleConstants.DriveEncoder.kPositionFactor
        driveEncoder.velocityConversionFactor = ModuleConstants.DriveEncoder.kVelocityFactor
        turnEncoder.setPositionConversionFactor(ModuleConstants.TurnEncoder.kPositionFactor)
        turnEncoder.setVelocityConversionFactor(ModuleConstants.TurnEncoder.kVelocityFactor)

        // Invert the turn encoder
        turnEncoder.setInverted(ModuleConstants.TurnEncoder.kInverted)

        // Turn PID wrap
        turnPid.setPositionPIDWrappingEnabled(true)
        turnPid.setPositionPIDWrappingMinInput(ModuleConstants.TurnEncoder.kMinPidInput)
        turnPid.setPositionPIDWrappingMaxInput(ModuleConstants.TurnEncoder.kMaxPidInput)

        // Apply PID constants
        drivePid.p = PidConstants.DriveMotor.kP
        drivePid.i = PidConstants.DriveMotor.kI
        drivePid.d = PidConstants.DriveMotor.kD
        drivePid.ff = PidConstants.DriveMotor.kFF
        drivePid.setOutputRange(PidConstants.DriveMotor.kPidMin, PidConstants.DriveMotor.kPidMax)

        turnPid.p = PidConstants.TurnMotor.kP
        turnPid.i = PidConstants.TurnMotor.kI
        turnPid.d = PidConstants.TurnMotor.kD
        turnPid.ff = PidConstants.TurnMotor.kFF
        turnPid.setOutputRange(PidConstants.TurnMotor.kPidMin, PidConstants.TurnMotor.kPidMax)

        // idle mode and smart current
        driveMotor.setIdleMode(ModuleConstants.kIdleMode)
        turnMotor.setIdleMode(ModuleConstants.kIdleMode)
        driveMotor.setSmartCurrentLimit(ModuleConstants.DriveMotor.kCurrentLimit)
        turnMotor.setSmartCurrentLimit(ModuleConstants.TurnMotor.kCurrentLimit)

        // One module is f*ed up
        if (driveId == 40) driveMotor.inverted = true

        // Maintain configs
        driveMotor.burnFlash()
        turnMotor.burnFlash()

        // Finish setup
        desiredState.angle = Rotation2d(turnEncoder.position)
        driveEncoder.position = 0.0
    }

    val state
        get() = SwerveModuleState(driveEncoder.velocity, Rotation2d(turnEncoder.position))

    val position
        get() = SwerveModulePosition(driveEncoder.position, Rotation2d(turnEncoder.position - angularOffset))

    fun stop() {
        driveMotor.stopMotor()
        turnMotor.stopMotor()
    }

    fun resetEncoders() {
        driveEncoder.position = 0.0
    }
}