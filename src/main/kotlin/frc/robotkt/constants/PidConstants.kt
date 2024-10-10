package frc.robotkt.constants

object PidConstants {
    object AutoTranslation {
        const val kP = 2.0
        const val kI = 0.0
        const val kD = 0.0
    }

    object AutoRotation {
        const val kP = 2.0
        const val kI = 0.0
        const val kD = 0.0
    }

    object DriveMotor {
        const val kP = 0.15
        const val kI = 0.0
        const val kD = 0.01
        const val kFF = 1 / ModuleConstants.Wheel.kFreeSpeed
        const val kPidMin = -1.0
        const val kPidMax = 1.0
    }

    object TurnMotor {
        const val kP = 0.3
        const val kI = 0.0
        const val kD = 0.02
        const val kFF = 0.0
        const val kPidMin = -1.0
        const val kPidMax = 1.0
    }

    object PhotonAlign {
        const val kP = 0.1
        const val kI = 0.0
        const val kD = 0.0
    }

    object Arm {
        const val kPivP = 0.01
        const val kPivI = 0.0
        const val kPivD = 0.0
        const val kPivFF = 0.0
    }

    object Shintake {
        const val kShooterP = 0.2
        const val kShooterI = 0.0
        const val kShooterD = 0.1
        const val kShooterFF = 0.0
    }

    object Climber {
        const val kP = 0.0
        const val kI = 0.0
        const val kD = 0.0
    }
}