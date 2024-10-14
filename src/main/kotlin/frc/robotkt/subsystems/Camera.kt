package frc.robotkt.subsystems

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.VisionConstants
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class Camera(name: String, val drivetrain: Drivetrain) : SubsystemBase() {
    private val camera = PhotonCamera(name)

    private val estimator = PhotonPoseEstimator(
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()!!,
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
        camera,
        VisionConstants.kCameraToRobot.inverse(),
    )

    private val odometry
        get() = drivetrain.odometry

    fun hasTargets() = camera.latestResult!!.hasTargets()

    val bestTarget
        get() = if (hasTargets()) camera.latestResult!!.bestTarget else null

    val targets
        get() = camera.latestResult!!.targets!!

    val timestamp
        get() = if (hasTargets()) camera.latestResult!!.timestampSeconds else null

    override fun periodic() {
        estimator.update().ifPresent { est ->
            odometry.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds
            )
        }
    }
}