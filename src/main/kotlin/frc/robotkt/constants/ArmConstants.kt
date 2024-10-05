package frc.robotkt.constants

object ArmConstants {
    // TODO: check
    object Pivot {
        const val kEncoderMin = 0.0
        const val kEncoderMax = 35.73

        const val kMinAngle = -10.0
        const val kMaxAngle = 110.0

        const val kSlope = (kMaxAngle - kMinAngle) / (kEncoderMax - kEncoderMin)
        const val kIntercept = kMinAngle - kSlope * kEncoderMin

        const val kMaxSpeed = 0.15
    }

    // TODO: check
    object Extension {
        const val kPotMin = 0.1395
        const val kPotMax = 0.4725

        const val kMin = 0.0
        const val kMax = 100.0

        const val kSlope = (kMax - kMin) / (kPotMax - kPotMin)
        const val kIntercept = kMin - kSlope * kPotMax

        const val kMaxSpeed = 0.8
        const val kTolerance = (kPotMax - kPotMin) / 30.0
    }

    const val kExtSpeed = 0.8
    const val kPivSpeed = 0.15
    const val kMaxAccel = 0.2

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