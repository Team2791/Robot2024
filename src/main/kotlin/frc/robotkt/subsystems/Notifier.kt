package frc.robotkt.subsystems

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class Notifier(
    private var driverctl: CommandXboxController,
) : SubsystemBase() {
    private val timer = Timer()

    fun vibrate() {
        driverctl.hid.setRumble(RumbleType.kBothRumble, 1.0)
        timer.start()
    }

    private fun end() {
        driverctl.hid.setRumble(RumbleType.kBothRumble, 0.0)
        timer.stop()
        timer.reset()
    }

    override fun periodic() {
        if (timer.hasElapsed(1.0)) end()
    }
}