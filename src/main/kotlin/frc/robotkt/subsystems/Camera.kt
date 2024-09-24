package frc.robotkt.subsystems

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.VisionConstants
import org.photonvision.PhotonCamera

class Camera(val camera: PhotonCamera, val drivetrain: Drivetrain) : SubsystemBase() {
    enum class CameraMode {
        AprilTag,
        Note;

        companion object {
            @JvmStatic
            fun from(pipeline: Int) = when (pipeline) {
                0 -> AprilTag
                1 -> Note
                else -> throw IllegalArgumentException("Invalid pipeline: $pipeline")
            }
        }
    }

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

    var mode
        get() = CameraMode.from(camera.pipelineIndex)
        set(value) {
            if (value == mode) return
            camera.pipelineIndex = value.ordinal
        }

    fun hasTargets() = camera.latestResult!!.hasTargets()

    val bestTarget
        get() = camera.latestResult!!.bestTarget

    val targets
        get() = camera.latestResult!!.targets!!

    val timestamp
        get() = camera.latestResult!!.timestampSeconds

    override fun periodic() {
        if (!hasTargets()) return
        if (mode != CameraMode.AprilTag) return

        val targetTag = bestTarget!!
        val tagId = targetTag.fiducialId

        if (targetTag.poseAmbiguity <= 0.2 && tagId != -1) {
            val targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(tagId).get()

            val transform = targetTag.bestCameraToTarget!!
            val pose = targetPose.transformBy(transform)!!
            val measurement = pose.transformBy(VisionConstants.kCameraToRobot)!!

            odometry.addVisionMeasurement(measurement.toPose2d(), timestamp, VisionConstants.kCameraError)
        }
    }
}