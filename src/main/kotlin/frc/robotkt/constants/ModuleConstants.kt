package frc.robotkt.constants

import com.revrobotics.CANSparkBase.IdleMode
import kotlin.math.PI

object ModuleConstants {
    object Neo {
        const val kFreeSpeedRpm = 5676.0
        const val kFreeSpeedRps = kFreeSpeedRpm / 60
    }

    object Wheel {
        const val kDiameter = 3.0 * kMetersPerInch
        const val kCircumference = kDiameter * PI
        const val kFreeSpeed = (Neo.kFreeSpeedRps * kCircumference) / DriveMotor.kReduction
    }

    object DriveMotor {
        const val kPinionTeeth = 14.0
        const val kBevelGearTeeth = 45.0
        const val kSpurTeeth = 22.0
        const val kBevelPinionTeeth = 15.0
        const val kReduction = (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth)
        const val kCurrentLimit = 40
    }

    object DriveEncoder {
        const val kPositionFactor = (Wheel.kDiameter * PI) / DriveMotor.kReduction
        const val kVelocityFactor = kPositionFactor / 60
    }

    object TurnMotor {
        const val kCurrentLimit = 40
    }

    object TurnEncoder {
        const val kPositionFactor = kTau
        const val kVelocityFactor = kTau / 60
        const val kInverted = true

        const val kMinPidInput = 0.0
        const val kMaxPidInput = kPositionFactor
    }

    const val kMaxAutoSpeed = 6.0
    val kIdleMode = IdleMode.kBrake
}