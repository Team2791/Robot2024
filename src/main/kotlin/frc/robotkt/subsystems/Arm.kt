package frc.robotkt.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.ArmConstants
import frc.robotkt.constants.IOConstants
import frc.robotkt.constants.PidConstants
import kotlin.math.abs

class Arm : SubsystemBase() {
    val leftMotor = CANSparkMax(IOConstants.ArmCan.kLeftMotor, MotorType.kBrushless)
    val rightMotor = CANSparkMax(IOConstants.ArmCan.kRightMotor, MotorType.kBrushless)
    val extMotor = CANSparkMax(IOConstants.ArmCan.kExtMotor, MotorType.kBrushless)

    val pivctl = leftMotor.pidController!!
    var extctl = PIDController(PidConstants.Arm.kExtP, PidConstants.Arm.kExtI, PidConstants.Arm.kExtD)

    val pivenc = leftMotor.encoder!!
    val extpot = AnalogPotentiometer(
        IOConstants.ArmCan.kPotChannel,
        ArmConstants.Extension.kSlope,
        ArmConstants.Extension.kIntercept
    )

    var angle
        get() = rawPivot * ArmConstants.Pivot.kSlope + ArmConstants.Pivot.kIntercept
        set(value) {
            pivTarget = (normalizePiv(value) - ArmConstants.Pivot.kIntercept) / ArmConstants.Pivot.kSlope
        }

    var extension
        get() = rawExtension * ArmConstants.Extension.kSlope + ArmConstants.Extension.kIntercept
        set(value) {
            extTarget = (normalizeExt(value) - ArmConstants.Extension.kIntercept) / ArmConstants.Extension.kSlope
        }

    private var pivTarget = rawPivot
        set(value) {
            pivctl.setReference(value, CANSparkBase.ControlType.kPosition)
            field = value
        }

    private var extTarget = rawExtension
        set(value) {
            extctl.setSetpoint(value)
            field = value
        }

    val rawPivot
        get() = pivenc.position

    val rawExtension
        get() = (extpot.get() - ArmConstants.Extension.kIntercept) / ArmConstants.Extension.kSlope

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
        extctl.setTolerance(ArmConstants.Extension.kTolerance)
    }

    companion object {
        @JvmStatic
        private fun normalizeExt(value: Double) =
            value.coerceIn(ArmConstants.Extension.kMin, ArmConstants.Extension.kMax)

        @JvmStatic
        private fun normalizePiv(value: Double) =
            value.coerceIn(ArmConstants.Pivot.kMinAngle, ArmConstants.Pivot.kMaxAngle)
    }

    fun atPivTarget() = abs(pivenc.position - pivTarget) < ArmConstants.kValueTolerance
    fun atExtTarget() = abs(rawExtension - extTarget) < ArmConstants.kValueTolerance

    fun moveUp(speed: Double = ArmConstants.kArmSpeed) = leftMotor.set(-speed)
    fun moveDown(speed: Double = ArmConstants.kArmSpeed) = leftMotor.set(speed)

    fun extend(speed: Double = ArmConstants.kExtSpeed) = extMotor.set(speed)
    fun retract(speed: Double = ArmConstants.kExtSpeed) = extMotor.set(-speed)

    fun hold() {
        leftMotor.set(0.0)
        pivTarget = rawPivot
    }

    private fun matchExtTarget() = extMotor.set(extctl.calculate(rawExtension))

    override fun periodic() {
        matchExtTarget()

        SmartDashboard.putNumber("Pivot Angle", angle)
        SmartDashboard.putNumber("Pivot Encoder", rawPivot)
        SmartDashboard.putNumber("Pivot Target", pivTarget)

        SmartDashboard.putNumber("Extension Percent", extpot.get())
        SmartDashboard.putNumber("Extension Encoder", rawExtension)
    }
}