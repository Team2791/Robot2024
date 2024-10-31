package frc.robotkt.subsystems

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.MathUtil
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robotkt.constants.DriveConstants
import frc.robotkt.constants.IOConstants
import frc.robotkt.constants.ModuleConstants
import frc.robotkt.constants.PidConstants
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
    private val frontLeft = SwerveModule(
        IOConstants.DrivetrainCan.kFrontLeftDrive,
        IOConstants.DrivetrainCan.kFrontLeftTurn,
        DriveConstants.AngularOffsets.kFrontLeft
    )

    private val frontRight = SwerveModule(
        IOConstants.DrivetrainCan.kFrontRightDrive,
        IOConstants.DrivetrainCan.kFrontRightTurn,
        DriveConstants.AngularOffsets.kFrontRight
    )

    private val rearLeft = SwerveModule(
        IOConstants.DrivetrainCan.kRearLeftDrive,
        IOConstants.DrivetrainCan.kRearLeftTurn,
        DriveConstants.AngularOffsets.kRearLeft
    )

    private val rearRight = SwerveModule(
        IOConstants.DrivetrainCan.kRearRightDrive,
        IOConstants.DrivetrainCan.kRearRightTurn,
        DriveConstants.AngularOffsets.kRearRight
    )

    val gyro = AHRS(SPI.Port.kMXP)

    private val magLimiter = SlewRateLimiter(DriveConstants.SlewRate.kMagnitude)
    private val rotLimiter = SlewRateLimiter(DriveConstants.SlewRate.kRotation)

    val heading
        get() = Rotation2d.fromDegrees(gyro.angle * DriveConstants.kGyroFactor)!!

    private val modules
        get() = listOf(frontLeft, frontRight, rearLeft, rearRight)

    val modulePositions
        get() = modules.map(SwerveModule::position).toTypedArray()

    val moduleStates
        get() = modules.map(SwerveModule::state).toTypedArray()

    var chassisSpeeds: ChassisSpeeds
        get() = DriveConstants.kDriveKinematics.toChassisSpeeds(*moduleStates)
        private set(speeds) {
            val states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMps)

            modules.zip(states).forEach { (module, state) ->
                module.desiredState = state
            }
        }

    var pose: Pose2d
        get() = odometry.estimatedPosition
        private set(value) = odometry.resetPosition(heading, modulePositions, value)

    internal val odometry = SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        heading,
        modulePositions,
        Pose2d()
    )

    val field = Field2d()

    private var timer = WPIUtilJNI.now() * 1e-6
    private var translationDir = 0.0
    private var translationMag = 0.0

    init {
        gyro.reset()

        AutoBuilder.configureHolonomic(
            this::pose::get,
            this::pose::set,
            this::chassisSpeeds::get,
            this::chassisSpeeds::set,
            HolonomicPathFollowerConfig(
                PIDConstants(
                    PidConstants.AutoTranslation.kP,
                    PidConstants.AutoTranslation.kI,
                    PidConstants.AutoTranslation.kD
                ),
                PIDConstants(
                    PidConstants.AutoRotation.kP,
                    PidConstants.AutoRotation.kI,
                    PidConstants.AutoRotation.kD
                ),
                ModuleConstants.kMaxAutoSpeed,
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

        var driveTab = Shuffleboard.getTab("Drive Odometry")!!

        driveTab.addNumber("Heading (degrees)") { heading.degrees }
        driveTab.addNumber("X Speed") { chassisSpeeds.vxMetersPerSecond }
        driveTab.addNumber("Y Speed") { chassisSpeeds.vyMetersPerSecond }
        driveTab.addNumber("Rotation Speed (rad per sec)") { chassisSpeeds.omegaRadiansPerSecond }
        driveTab.add("Field") { field }
    }

    /**
     * Swerve drive control
     *
     * @param speeds The desired field-relative speeds
     * @param fieldRelative Whether the speeds are field-relative, defaults to true
     * @param rateLimit Whether to rate limit the translation and rotation speeds, defaults to false
     */
    @JvmOverloads
    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean = true, rateLimit: Boolean = false) {
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
     * @param fieldRelative Whether the speeds are field-relative, defaults to true
     * @param rateLimit Whether to rate limit the translation and rotation speeds, defaults to false
     */
    @JvmOverloads
    fun drive(
        xspeed: Double,
        yspeed: Double,
        rspeed: Double,
        fieldRelative: Boolean = true,
        rateLimit: Boolean = false
    ) =
        drive(ChassisSpeeds(xspeed, yspeed, rspeed), fieldRelative, rateLimit)

    /**
     * Driving with controller input
     *
     * @param controller The controller to use for driving
     * @param rot Optional rotation speed for note/tag alignment, will use controller input if not provided
     */
    @JvmOverloads
    fun drive(
        controller: CommandXboxController,
        rot: Double = -MathUtil.applyDeadband(controller.rightX, IOConstants.Controller.kDeadband)
    ) =
        drive(
            -MathUtil.applyDeadband(controller.leftY, IOConstants.Controller.kDeadband),
            -MathUtil.applyDeadband(controller.leftX, IOConstants.Controller.kDeadband),
            rot,
        )

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
    }
}