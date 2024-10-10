package frc.robotkt.constants

object ArmConstants {
    object Pivot {
        const val kEncoderMin = -3.54
        const val kEncoderMax = 28.9

        const val kMinAngle = -11.0
        const val kMaxAngle = 90.0

        const val kPositionFactor = 1 / ((kMaxAngle - kMinAngle) / (kEncoderMax - kEncoderMin))

        const val kMaxSpeed = 0.15
        const val kMaxAccel = 0.1
    }

    object Extension {
        const val kEncMin = 0.1395
        const val kEncMax = 0.4725

        const val kMin = 0.0
        const val kMax = 100.0

        const val kPositionFactor = 1 / ((kMax - kMin) / (kEncMax - kEncMin))
        const val kSpeed = 0.8
    }

    const val kValueTolerance = 5.0

    // TODO: check
    const val kAmpAngle = 70
    const val kIntakeAngle = 20

    // TODO: check
    const val kPivotHeight = 0.0
    const val kRobotToPivot = 0.0
    const val kShintakeAngle = 0.0
    const val kLength = 0.0
}