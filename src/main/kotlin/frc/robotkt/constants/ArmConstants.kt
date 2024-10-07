package frc.robotkt.constants

object ArmConstants {
    // TODO: check
    object Pivot {
        const val kEncoderMin = -3.54
        const val kEncoderMax = 28.9

        const val kMinAngle = -11.0
        const val kMaxAngle = 90.0

        const val kPositionFactor = (kMaxAngle - kMinAngle) / (kEncoderMax - kEncoderMin)

        const val kMaxSpeed = 0.15
        const val kMaxAccel = 0.1
    }

    // TODO: check
    object Extension {
        const val kPotMin = 0.1395
        const val kPotMax = 0.4725

        const val kMin = 0.0
        const val kMax = 100.0

        const val kPositionFactor = (kMax - kMin) / (kPotMax - kPotMin)

        const val kMaxSpeed = 0.8
        const val kMaxAccel = 0.2
    }

    const val kValueTolerance = 2.0

    // TODO: check
    const val kAmpAngle = 70
    const val kIntakeAngle = 20

    // TODO: check
    const val kPivotPosHeightMeter = 0.0
    const val kRobotCenterToPivotPosMeter = 0.0
    const val kShintakeArmAngleRad = 0.0
    const val kArmLength = 0.0
}