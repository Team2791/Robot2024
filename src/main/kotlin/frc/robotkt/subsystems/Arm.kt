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

    private val pivenc = leftMotor.absoluteEncoder!!
    private val extenc = extMotor.absoluteEncoder!!

    private val pivctl = leftMotor.pidController!!
    private val extctl = extMotor.pidController!!

    var angle
        get() = pivenc.position
        private set(target) = let { pivctl.setReference(normalizePiv(target), ControlType.kPosition) }

    var extension
        get() = extenc.position
        private set(target) = let { extctl.setReference(normalizeExt(target), ControlType.kPosition) }

    var pivTarget = 0.0
        set(value) = let { angle = value; field = value }

    var extTarget = 0.0
        set(value) = let { extension = value; field = value }

    init {
        rightMotor.follow(leftMotor, true)

        leftMotor.idleMode = IdleMode.kBrake
        rightMotor.idleMode = IdleMode.kBrake
        extMotor.idleMode = IdleMode.kBrake

        pivctl.p = PidConstants.Arm.kPivP
        pivctl.i = PidConstants.Arm.kPivI
        pivctl.d = PidConstants.Arm.kPivD
        pivctl.ff = PidConstants.Arm.kPivFF
        pivctl.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kSCurve, 0)
        pivctl.setSmartMotionMaxAccel(ArmConstants.kMaxAccel, 0)
        pivctl.setSmartMotionMaxVelocity(ArmConstants.kPivSpeed, 0)
        pivctl.setFeedbackDevice(pivenc)

        pivenc.positionConversionFactor = ArmConstants.Pivot.kSlope
        pivenc.zeroOffset = ArmConstants.Pivot.kIntercept

        extctl.p = PidConstants.Arm.kExtP
        extctl.i = PidConstants.Arm.kExtI
        extctl.d = PidConstants.Arm.kExtD
        extctl.ff = PidConstants.Arm.kExtFF
        extctl.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kSCurve, 0)
        extctl.setSmartMotionMaxAccel(ArmConstants.kMaxAccel, 0)
        extctl.setSmartMotionMaxVelocity(ArmConstants.kExtSpeed, 0)
        extctl.setFeedbackDevice(extenc)

        extenc.positionConversionFactor = ArmConstants.Extension.kSlope
        extenc.zeroOffset = ArmConstants.Extension.kIntercept
    }

    companion object {
        @JvmStatic
        private fun normalizeExt(value: Double) =
            value.coerceIn(ArmConstants.Extension.kMin, ArmConstants.Extension.kMax)

        @JvmStatic
        private fun normalizePiv(value: Double) =
            value.coerceIn(ArmConstants.Pivot.kMinAngle, ArmConstants.Pivot.kMaxAngle)

        @JvmStatic
        private fun limitExt(value: Double) =
            value.coerceIn(-ArmConstants.kExtSpeed, ArmConstants.kExtSpeed)

        @JvmStatic
        private fun limitPiv(value: Double) =
            value.coerceIn(-ArmConstants.kPivSpeed, ArmConstants.kPivSpeed)
    }

    fun atPivTarget() = abs(pivenc.position - pivTarget) < ArmConstants.kValueTolerance
    fun atExtTarget() = abs(extenc.position - extTarget) < ArmConstants.kValueTolerance

    fun angleUp(speed: Double = ArmConstants.kPivSpeed) = pivctl.setReference(limitPiv(speed), ControlType.kDutyCycle)
    fun angleDown(speed: Double = ArmConstants.kPivSpeed) = angleUp(-speed)

    fun extend(speed: Double = ArmConstants.kExtSpeed) = extctl.setReference(limitExt(speed), ControlType.kDutyCycle)
    fun retract(speed: Double = ArmConstants.kExtSpeed) = extend(-speed)

    fun hold() {
        pivTarget = pivenc.position
        extTarget = extenc.position
    }

    override fun periodic() {
        SmartDashboard.putNumber("Pivot Angle", angle)
        SmartDashboard.putNumber("Pivot Encoder", pivenc.position)
        SmartDashboard.putNumber("Pivot Target", pivTarget)

        SmartDashboard.putNumber("Extension Percent", extension)
        SmartDashboard.putNumber("Extension Encoder", extenc.position)
        SmartDashboard.putNumber("Extension Target", extTarget)
    }
}