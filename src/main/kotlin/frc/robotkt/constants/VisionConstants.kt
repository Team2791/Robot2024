package frc.robotkt.constants

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N3
import kotlin.math.PI

object VisionConstants {
    // TODO: check these values
    const val kCameraPitch = 30 * kRadiansPerDegree
    val kCameraError: Vector<N3> = VecBuilder.fill(0.1, 0.1, 0.1)!!
    val kCameraToRobot = Transform3d(
        Translation3d(12.25 * kMetersPerInch, 12.25 * kMetersPerInch, 0.0),
        Rotation3d(0.0, 15.0 * kRadiansPerDegree, -3.0 * kRadiansPerDegree)
    )

    const val kFieldLength = 16.54
    const val kFieldWidth = 8.21
    val kFlippingPose = Pose2d(Translation2d(kFieldLength, kFieldWidth), Rotation2d(PI))
}