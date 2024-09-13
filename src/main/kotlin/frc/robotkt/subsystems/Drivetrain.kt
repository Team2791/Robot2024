package frc.robotkt.subsystems

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
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robotkt.constants.AutoConstants
import frc.robotkt.constants.CanIds
import frc.robotkt.constants.DriveConstants
import frc.robotkt.swerve.SwerveModule
import frc.robotkt.swerve.angleDifference
import frc.robotkt.swerve.normalizeAngle
import frc.robotkt.swerve.stepTowardsAngle

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

    val heading
        get() = Rotation2d.fromDegrees(gyro.angle * DriveConstants.kGyroFactor)!!

    val gyroAngle
        get() = Rotation2d.fromDegrees(gyro.angle)!!

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

    val field = Field2d()

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
                val alliance = DriverStation.getAlliance()!!
                if (alliance.isPresent) alliance.get() == DriverStation.Alliance.Red
                else false
            },
            this
        )
    }

    /**
     * Swerve drive control
     *
     * @param speeds The desired field-relative speeds
     * @param fieldRelative Whether the speeds are field-relative
     * @param rateLimit Whether to rate limit the translation and rotation speeds
     */
    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean = true, rateLimit: Boolean = true) {
        var xspeed = speeds.vxMetersPerSecond
        var yspeed = speeds.vyMetersPerSecond
        var rspeed = speeds.omegaRadiansPerSecond

        val time = WPIUtilJNI.now() * 1e-6
        val elapsed = time - timer
        timer = time

        if (rateLimit) {
            val theta = atan2(yspeed, xspeed)
            val mag = hypot(yspeed, xspeed)
            val offset = angleDifference(theta, translationDir)
            val slew = when {
                translationMag == 0.0 -> 500.0
                else -> abs(DriveConstants.SlewRate.kDirection / translationMag)
            }

            when {
                offset < 0.45 * PI -> {
                    translationDir = stepTowardsAngle(translationDir, theta, slew * elapsed)
                    translationMag = magLimiter.calculate(mag)
                }

                offset > 0.85 * PI -> {
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

        chassisSpeeds = when {
            fieldRelative -> ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, rspeed, heading)
            else -> ChassisSpeeds(xspeed, yspeed, rspeed)
        }
    }

    /**
     * Swerve drive control with manual speed inputs
     *
     * @param xspeed The desired x-axis speed in meters per second
     * @param yspeed The desired y-axis speed in meters per second
     * @param rspeed The desired rotation speed in radians per second
     * @param fieldRelative Whether the speeds are field-relative
     * @param rateLimit Whether to rate limit the translation and rotation speeds
     */
    fun drive(xspeed: Double, yspeed: Double, rspeed: Double, fieldRelative: Boolean, rateLimit: Boolean) =
        drive(ChassisSpeeds(xspeed, yspeed, rspeed), fieldRelative, rateLimit)

    /**
     * Reset the gyro angle to zero
     */
    fun resetGyro() = gyro.reset()

    /**
     * Reset the swerve encoders
     */
    fun resetEncoders() = modules.forEach(SwerveModule::resetEncoders)

    /**
     * Stop the movement of the drivetrain
     */
    fun stop() = modules.forEach(SwerveModule::stop)

    /**
     * Lock the drivetrain in place with an X formation of the wheels
     */
    fun lock() {
        frontLeft.desiredState = SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        frontRight.desiredState = SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0))
        rearLeft.desiredState = SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0))
        rearRight.desiredState = SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
    }

    override fun periodic() {
        // Update the odometry
        odometry.updateWithTime(WPIUtilJNI.now() * 1e-6, heading, modulePositions)
        field.robotPose = odometry.estimatedPosition

        // SmartDashboard updates
        SmartDashboard.putNumber("Heading (degrees)", heading.degrees)
        SmartDashboard.putNumber("X Speed (m/s)", chassisSpeeds.vxMetersPerSecond)
        SmartDashboard.putNumber("Y Speed (m/s)", chassisSpeeds.vyMetersPerSecond)
        SmartDashboard.putNumber("Rotation Speed (rad/s)", chassisSpeeds.omegaRadiansPerSecond)
        SmartDashboard.putData(field)
    }
}