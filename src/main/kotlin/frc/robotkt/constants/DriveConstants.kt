package frc.robotkt.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import kotlin.math.PI
import kotlin.math.sqrt

object DriveConstants {
    object AngularOffsets {
        const val kFrontLeft = -PI / 2
        const val kFrontRight = 0.0
        const val kRearLeft = PI
        const val kRearRight = PI / 2
    }

    object SlewRate {
        const val kMagnitude = 5.0
        const val kRotation = 5.0
        const val kDirection = 3.0
    }

    object Dimensions {
        const val kWheelBase = 23.25 * kMetersPerInch
        val kDriveBaseRadius = 0.5 * kWheelBase * sqrt(2.0)
    }

    val kDriveKinematics = SwerveDriveKinematics(
        Translation2d(Dimensions.kWheelBase / 2, Dimensions.kWheelBase / 2),
        Translation2d(Dimensions.kWheelBase / 2, -Dimensions.kWheelBase / 2),
        Translation2d(-Dimensions.kWheelBase / 2, Dimensions.kWheelBase / 2),
        Translation2d(-Dimensions.kWheelBase / 2, -Dimensions.kWheelBase / 2)
    )

    const val kMaxSpeedMps = 20.0
    const val kMaxSpeedAnglular = kTau
    const val kGyroFactor = -1.0
}