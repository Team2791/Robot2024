package frc.robotkt.subsystems

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.Led

class Notifier(
    var led: Led,
    private var driverctl: CommandXboxController,
    private var operctl: CommandXboxController
) : SubsystemBase() {
    private val timer = Timer()

    fun notify() {
        driverctl.hid.setRumble(RumbleType.kBothRumble, 1.0)
        operctl.hid.setRumble(RumbleType.kBothRumble, 1.0)
        timer.start()
    }

    private fun end() {
        driverctl.hid.setRumble(RumbleType.kBothRumble, 0.0)
        operctl.hid.setRumble(RumbleType.kBothRumble, 0.0)
        timer.stop()
        timer.reset()
    }

    override fun periodic() {
        if (timer.hasElapsed(1.0)) end()
    }
}