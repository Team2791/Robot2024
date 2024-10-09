package frc.robotkt.subsystems

import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.ArmConstants
import frc.robotkt.constants.IOConstants
import frc.robotkt.constants.PidConstants
import kotlin.math.abs

class Arm : SubsystemBase() {
    private val leftMotor = CANSparkMax(IOConstants.ArmCan.kLeftMotor, MotorType.kBrushless)
    private val rightMotor = CANSparkMax(IOConstants.ArmCan.kRightMotor, MotorType.kBrushless)
    private val extMotor = CANSparkMax(IOConstants.ArmCan.kExtMotor, MotorType.kBrushless)

    private val pivenc = leftMotor.encoder!!
    private val extenc = extMotor.encoder!!

    private val pivctl = leftMotor.pidController!!
    private val extctl = extMotor.pidController!!

    var angle
        get() = pivenc.position
        private set(target) = let { pivctl.setReference(normalizePiv(target), ControlType.kPosition) }

    var extension
        get() = extenc.position
        private set(target) = let { extctl.setReference(normalizeExt(target), ControlType.kPosition) }

    var angleTarget = 0.0
        set(value) = let { angle = value; field = value }

    var extTarget = 0.0
        set(value) = let { extension = value; field = value }

    init {
        rightMotor.follow(leftMotor, true)

        leftMotor.idleMode = IdleMode.kBrake
        rightMotor.idleMode = IdleMode.kBrake
        extMotor.idleMode = IdleMode.kBrake

        pivenc.positionConversionFactor = ArmConstants.Pivot.kPositionFactor
        pivenc.position = 0.0

        extenc.positionConversionFactor = ArmConstants.Extension.kPositionFactor
        extenc.position = 0.0

        pivctl.p = PidConstants.Arm.kPivP
        pivctl.i = PidConstants.Arm.kPivI
        pivctl.d = PidConstants.Arm.kPivD
        pivctl.ff = PidConstants.Arm.kPivFF

        pivctl.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
        pivctl.setSmartMotionMaxAccel(ArmConstants.Pivot.kMaxAccel, 0)
        pivctl.setSmartMotionMaxVelocity(ArmConstants.Pivot.kMaxSpeed, 0)
        pivctl.setFeedbackDevice(pivenc)

        extctl.p = PidConstants.Arm.kExtP
        extctl.i = PidConstants.Arm.kExtI
        extctl.d = PidConstants.Arm.kExtD
        extctl.ff = PidConstants.Arm.kExtFF

        extctl.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
        extctl.setSmartMotionMaxAccel(ArmConstants.Extension.kMaxAccel, 0)
        extctl.setSmartMotionMaxVelocity(ArmConstants.Extension.kMaxSpeed, 0)
        extctl.setFeedbackDevice(extenc)
    }

    companion object {
        @JvmStatic
        private fun normalizeExt(value: Double) =
            value.coerceIn(ArmConstants.Extension.kMin, ArmConstants.Extension.kMax)

        @JvmStatic
        private fun normalizePiv(value: Double) =
            value.coerceIn(ArmConstants.Pivot.kMinAngle, ArmConstants.Pivot.kMaxAngle)
    }

    fun atPivTarget() = abs(pivenc.position - angleTarget) < ArmConstants.kValueTolerance
    fun atExtTarget() = abs(extenc.position - extTarget) < ArmConstants.kValueTolerance

    // Manual control of the arm
    fun angleUp() = let { angleTarget = ArmConstants.Pivot.kMaxAngle }
    fun angleDown() = let { angleTarget = ArmConstants.Pivot.kMinAngle }
    fun holdAngle() = let { angleTarget = angle }

    fun extend() = let { extension = ArmConstants.Extension.kMax }
    fun retract() = let { extension = ArmConstants.Extension.kMin }
    fun holdExtension() = let { extension = extTarget }

    fun hold() {
        holdAngle()
        holdExtension()
    }

    override fun periodic() {
        SmartDashboard.putNumber("Pivot Angle", angle)
        SmartDashboard.putNumber("Pivot Encoder", pivenc.position)
        SmartDashboard.putNumber("Pivot Target", angleTarget)

        SmartDashboard.putNumber("Extension Percent", extension)
        SmartDashboard.putNumber("Extension Encoder", extenc.position)
        SmartDashboard.putNumber("Extension Target", extTarget)
    }
}