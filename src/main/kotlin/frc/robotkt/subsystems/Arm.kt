package frc.robotkt.subsystems

import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
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

    val angle
        get() = pivenc.position * ArmConstants.Pivot.kPositionFactor

    val extension
        get() = extenc.position * ArmConstants.Extension.kPositionFactor

    var angleTarget = 0.0
        set(value) {
            field = normalizePiv(value)
            pivctl.setReference(field, ControlType.kPosition)
        }

    var extTarget = 0.0
        set(value) = let { field = normalizeExt(value) }

    init {
        rightMotor.follow(leftMotor, true)

        leftMotor.idleMode = IdleMode.kBrake
        rightMotor.idleMode = IdleMode.kBrake
        extMotor.idleMode = IdleMode.kBrake

        pivenc.position = 0.0
        extenc.position = 0.0

        pivctl.p = PidConstants.Arm.kPivP
        pivctl.i = PidConstants.Arm.kPivI
        pivctl.d = PidConstants.Arm.kPivD
        pivctl.ff = PidConstants.Arm.kPivFF

        pivctl.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
        pivctl.setSmartMotionMaxAccel(ArmConstants.Pivot.kMaxAccel, 0)
        pivctl.setSmartMotionMaxVelocity(ArmConstants.Pivot.kMaxSpeed, 0)
        pivctl.setOutputRange(PidConstants.Arm.kMinOut, PidConstants.Arm.kMaxOut)

        var tab = Shuffleboard.getTab("Arm")!!

        tab.addNumber("Pivot Angle") { angle }
        tab.addNumber("Pivot Encoder") { pivenc.position }
        tab.addNumber("Pivot Target") { angleTarget }

        tab.addNumber("Extension Percent") { extension }
        tab.addNumber("Extension Encoder") { extenc.position }
        tab.addNumber("Extension Target") { extTarget }
    }

    companion object {
        @JvmStatic
        private fun normalizeExt(value: Double) =
            value.coerceIn(ArmConstants.Extension.kMin, ArmConstants.Extension.kMax)

        @JvmStatic
        private fun normalizePiv(value: Double) =
            value.coerceIn(ArmConstants.Pivot.kMinAngle, ArmConstants.Pivot.kMaxAngle)
    }

    fun atPivTarget() = abs(angle - angleTarget) < ArmConstants.kValueTolerance
    fun atExtTarget() = abs(extension - extTarget) < ArmConstants.kValueTolerance

    // Manual control of the arm
    fun angleUp() = let { angleTarget = ArmConstants.Pivot.kMaxAngle }
    fun angleDown() = let { angleTarget = ArmConstants.Pivot.kMinAngle }
    fun holdAngle() = let { angleTarget = angle }

    fun extend() = let { extTarget = ArmConstants.Extension.kMax }
    fun retract() = let { extTarget = ArmConstants.Extension.kMin }
    fun holdExtension() = let { extTarget = extension }

    fun hold() {
        holdAngle()
        holdExtension()
    }

    fun extendSetpoint() {
        if (atExtTarget()) extMotor.set(0.0)
        else if (extTarget < extension && extension > ArmConstants.Extension.kMin + 10) extMotor.set(-ArmConstants.Extension.kSpeed)
        else if (extTarget > extension && extension < ArmConstants.Extension.kMax - 10) extMotor.set(ArmConstants.Extension.kSpeed)
    }

    override fun periodic() {
        extendSetpoint()
    }
}