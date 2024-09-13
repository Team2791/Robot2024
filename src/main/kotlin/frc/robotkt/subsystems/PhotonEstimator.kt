package frc.robotkt.subsystems

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.VisionConstants
import org.photonvision.PhotonCamera

class PhotonEstimator(val camera: PhotonCamera, val drivetrain: Drivetrain) : SubsystemBase() {
    val odometry
        get() = drivetrain.odometry

    var alliance = DriverStation.Alliance.Red
        set(value) {
            if (field == value) return
            else field = value

            odometry.resetPosition(
                drivetrain.gyroAngle,
                drivetrain.modulePositions,
                odometry.estimatedPosition.relativeTo(VisionConstants.kFlippingPose)
            )
        }

    override fun periodic() {
        val result = camera.latestResult!!
        val timestamp = result.timestampSeconds

        if (result.hasTargets()) {
            val targetTag = result.bestTarget
            val tagId = targetTag.fiducialId

            if (targetTag.poseAmbiguity <= 0.2 && tagId != 0) {
                val targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(tagId).get()

                val transform = targetTag.bestCameraToTarget
                val pose = targetPose.transformBy(transform)
                val measurement = pose.transformBy(VisionConstants.kCameraToRobot)

                odometry.addVisionMeasurement(measurement.toPose2d(), timestamp, VisionConstants.kCameraError)
            }
        }
    }
}