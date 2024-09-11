package frckt.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frckt.robot.constants.AutoConstants
import frckt.robot.constants.CanIds
import frckt.robot.constants.DriveConstants
import frckt.robot.swerve.SwerveModule
import frckt.robot.swerve.angleDifference
import frckt.robot.swerve.normalizeAngle
import frckt.robot.swerve.stepTowardsAngle
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class Drivetrain : SubsystemBase() {
    val frontLeft = SwerveModule(
        CanIds.Drivetrain.kFrontLeftDrive,
        CanIds.Drivetrain.kFrontLeftTurn,
        DriveConstants.AngularOffsets.kFrontLeft
    )

    val frontRight = SwerveModule(
        CanIds.Drivetrain.kFrontRightDrive,
        CanIds.Drivetrain.kFrontRightTurn,
        DriveConstants.AngularOffsets.kFrontRight
    )

    val rearLeft = SwerveModule(
        CanIds.Drivetrain.kRearLeftDrive,
        CanIds.Drivetrain.kRearLeftTurn,
        DriveConstants.AngularOffsets.kRearLeft
    )

    val rearRight = SwerveModule(
        CanIds.Drivetrain.kRearRightDrive,
        CanIds.Drivetrain.kRearRightTurn,
        DriveConstants.AngularOffsets.kRearRight
    )

    val gyro = AHRS(SPI.Port.kMXP)

    val magLimiter = SlewRateLimiter(DriveConstants.SlewRate.kMagnitude)
    val rotLimiter = SlewRateLimiter(DriveConstants.SlewRate.kRotation)

    val heading: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.angle * DriveConstants.kGyroFactor)

    val modules
        get() = listOf(frontLeft, frontRight, rearLeft, rearRight)

    val modulePositions
        get() = modules.map(SwerveModule::position).toTypedArray()

    val moduleStates
        get() = modules.map(SwerveModule::state).toTypedArray()

    var chassisSpeeds: ChassisSpeeds
        get() = DriveConstants.kDriveKinematics.toChassisSpeeds(*moduleStates)
        set(speeds) {
            val states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMps)

            modules.zip(states).forEach { (module, state) ->
                module.desiredState = state
            }
        }

    var pose: Pose2d
        get() = odometry.estimatedPosition
        set(value) = odometry.resetPosition(heading, modulePositions, value)

    val odometry = SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        heading,
        modulePositions,
        Pose2d()
    )

    var timer = WPIUtilJNI.now() * 1e-6

    var rotation = 0.0
    var translationDir = 0.0
    var translationMag = 0.0

    init {
        gyro.reset()

        AutoBuilder.configureHolonomic(
            this::pose::get,
            this::pose::set,
            this::chassisSpeeds::get,
            this::chassisSpeeds::set,
            HolonomicPathFollowerConfig(
                PIDConstants(
                    AutoConstants.TranslationPid.kP,
                    AutoConstants.TranslationPid.kI,
                    AutoConstants.TranslationPid.kD
                ),
                PIDConstants(
                    AutoConstants.RotationPid.kP,
                    AutoConstants.RotationPid.kI,
                    AutoConstants.RotationPid.kD
                ),
                AutoConstants.kMaxModuleSpeed,
                DriveConstants.Dimensions.kDriveBaseRadius,
                ReplanningConfig()
            ),
            {
                val alliance = DriverStation.getAlliance()
                if (alliance.isPresent) alliance.get() == DriverStation.Alliance.Red
                else false
            },
            this
        )
    }

    fun drive(speeds: ChassisSpeeds, rateLimit: Boolean) {
        var xspeed = speeds.vxMetersPerSecond
        var yspeed = speeds.vyMetersPerSecond
        var rspeed = speeds.omegaRadiansPerSecond

        val time = WPIUtilJNI.now() * 1e-6
        val elapsed = time - timer
        timer = time

        if (rateLimit) {
            val theta = atan2(yspeed, xspeed)
            val mag = hypot(yspeed, xspeed)
            val angleDiff = angleDifference(theta, translationDir)
            val slew = when {
                translationMag == 0.0 -> 500.0
                else -> abs(DriveConstants.SlewRate.kDirection / translationMag)
            }

            when {
                angleDiff < 0.45 * PI -> {
                    translationDir = stepTowardsAngle(translationDir, theta, slew * elapsed)
                    translationMag = magLimiter.calculate(mag)
                }

                angleDiff > 0.85 * PI -> {
                    if (translationMag > 1e-4) {
                        translationDir = normalizeAngle(translationDir + PI)
                        translationMag = magLimiter.calculate(mag)
                    } else {
                        translationMag = magLimiter.calculate(0.0)
                    }
                }

                else -> {
                    translationDir = stepTowardsAngle(translationDir, theta, slew * elapsed)
                    translationMag = magLimiter.calculate(0.0)
                }
            }

            xspeed = translationMag * cos(translationDir)
            yspeed = translationMag * sin(translationDir)
            rspeed = rotLimiter.calculate(rspeed)
        }

        xspeed *= DriveConstants.kMaxSpeedMps
        yspeed *= DriveConstants.kMaxSpeedMps
        rspeed *= DriveConstants.kMaxSpeedAnglular

        chassisSpeeds = ChassisSpeeds(xspeed, yspeed, rspeed)
    }

    fun drive(speeds: ChassisSpeeds) = drive(speeds, true)
    fun drive(xspeed: Double, yspeed: Double, rspeed: Double) = drive(ChassisSpeeds(xspeed, yspeed, rspeed))
}