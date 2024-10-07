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

    object ArmCan {
        const val kRightMotor = 31
        const val kLeftMotor = 32
        const val kExtMotor = 33
        const val kPotChannel = 0
    }

    object Led {
        const val kLength = 25
        const val kPort = 0
    }

    object Controller {
        const val kDriverPort = 0
        const val kOperatorPort = 1
        const val kDeadband = 0.1
    }

    object Shintake {
        const val kLeftShooter = 21
        const val kRightShooter = 22
        const val kIntake = 11
    }
}