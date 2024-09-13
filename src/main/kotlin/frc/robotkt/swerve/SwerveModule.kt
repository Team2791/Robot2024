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

class SwerveModule(driveID: Int, turnID: Int, val angularOffset: Double) {
    val drive = CANSparkMax(driveID, MotorType.kBrushless)
    val turn = CANSparkMax(turnID, MotorType.kBrushless)

    val driveEncoder: RelativeEncoder
    val turnEncoder: SparkAbsoluteEncoder

    val drivePID: SparkPIDController
    val turnPID: SparkPIDController

    var desiredState = SwerveModuleState(0.0, Rotation2d())
        set(desired) {
            val corrected = SwerveModuleState()
            corrected.speedMetersPerSecond = desired.speedMetersPerSecond
            corrected.angle = desired.angle.plus(Rotation2d(angularOffset))

            val optimized = SwerveModuleState.optimize(corrected, Rotation2d(turnEncoder.position))
            drivePID.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity)
            turnPID.setReference(optimized.angle.radians, ControlType.kPosition)

            field = desired
        }

    init {
        drive.restoreFactoryDefaults()
        turn.restoreFactoryDefaults()

        // Setup encoders and PID controllers
        driveEncoder = drive.encoder
        turnEncoder = turn.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

        drivePID = drive.getPIDController()
        turnPID = turn.getPIDController()

        drivePID.setFeedbackDevice(driveEncoder)
        turnPID.setFeedbackDevice(turnEncoder)

        // Apply position and velocity conversion factors
        driveEncoder.positionConversionFactor = ModuleConstants.DriveEncoder.kPositionFactor
        driveEncoder.velocityConversionFactor = ModuleConstants.DriveEncoder.kVelocityFactor
        turnEncoder.setPositionConversionFactor(ModuleConstants.TurnEncoder.kPositionFactor)
        turnEncoder.setVelocityConversionFactor(ModuleConstants.TurnEncoder.kVelocityFactor)

        // Invert the turn encoder
        turnEncoder.setInverted(ModuleConstants.TurnEncoder.kInverted)

        // Turn PID wrap
        turnPID.setPositionPIDWrappingEnabled(true)
        turnPID.setPositionPIDWrappingMinInput(ModuleConstants.TurnEncoder.kMinPidInput)
        turnPID.setPositionPIDWrappingMaxInput(ModuleConstants.TurnEncoder.kMaxPidInput)

        // Apply PID constants
        drivePID.setP(ModuleConstants.DriveMotor.kP)
        drivePID.setI(ModuleConstants.DriveMotor.kI)
        drivePID.setD(ModuleConstants.DriveMotor.kD)
        drivePID.setFF(ModuleConstants.DriveMotor.kFF)
        drivePID.setOutputRange(ModuleConstants.DriveMotor.kPidMin, ModuleConstants.DriveMotor.kPidMax)

        turnPID.setP(ModuleConstants.TurnMotor.kP)
        turnPID.setI(ModuleConstants.TurnMotor.kI)
        turnPID.setD(ModuleConstants.TurnMotor.kD)
        turnPID.setFF(ModuleConstants.TurnMotor.kFF)
        turnPID.setOutputRange(ModuleConstants.TurnMotor.kPidMin, ModuleConstants.TurnMotor.kPidMax)

        // idle mode and smart current
        drive.setIdleMode(ModuleConstants.kIdleMode)
        turn.setIdleMode(ModuleConstants.kIdleMode)
        drive.setSmartCurrentLimit(ModuleConstants.DriveMotor.kCurrentLimit)
        turn.setSmartCurrentLimit(ModuleConstants.TurnMotor.kCurrentLimit)

        // Maintain configs
        drive.burnFlash()
        turn.burnFlash()

        // Finish setup
        desiredState.angle = Rotation2d(turnEncoder.position)
        driveEncoder.position = 0.0
    }

    val state
        get() = SwerveModuleState(driveEncoder.velocity, Rotation2d(turnEncoder.position))

    val position
        get() = SwerveModulePosition(driveEncoder.position, Rotation2d(turnEncoder.position - angularOffset))

    fun stop() {
        drive.stopMotor()
        turn.stopMotor()
    }

    fun resetEncoders() {
        driveEncoder.position = 0.0
    }
}