package frc.robotkt.constants

object IOConstants {
    object DrivetrainCan {
        const val kFrontLeftDrive = 40
        const val kFrontRightDrive = 30
        const val kRearLeftDrive = 20
        const val kRearRightDrive = 10

        const val kFrontLeftTurn = 45
        const val kFrontRightTurn = 35
        const val kRearLeftTurn = 25
        const val kRearRightTurn = 15
    }

    object Controller {
        const val kDriverPort = 0
        const val kDeadband = 0.1
    }

    object Shooter {
        const val kTop = 21
        const val kBottom = 22
        const val kMiddle = 11
    }
}