package frc.robotkt.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import kotlin.math.PI
import kotlin.math.hypot

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
        const val kWheelBase = 24.5 * kMetersPerInch
        const val kTrackWidth = 22.5 * kMetersPerInch
        val kDriveBaseRadius = 0.5 * hypot(kWheelBase, kTrackWidth)
    }

    val kDriveKinematics = SwerveDriveKinematics(
        Translation2d(Dimensions.kWheelBase / 2, Dimensions.kTrackWidth / 2),
        Translation2d(Dimensions.kWheelBase / 2, -Dimensions.kTrackWidth / 2),
        Translation2d(-Dimensions.kWheelBase / 2, Dimensions.kTrackWidth / 2),
        Translation2d(-Dimensions.kWheelBase / 2, -Dimensions.kTrackWidth / 2)
    )

    const val kMaxSpeedMps = 15.0
    const val kMaxSpeedAnglular = kTau
    const val kGyroFactor = -1.0
}