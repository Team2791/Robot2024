// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Nothing to see here, go to RobotContainer for useful stuff.
 * Try not to put too much logic here
 */
public class Robot extends TimedRobot {
    private final RobotContainer container = new RobotContainer();
    private Command auto;

    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public void disabledInit() {
        container.led.setRGB(255, 0, 0);
    }

    public void autonomousInit() {
        auto = this.container.getAutonomousCommand();
        if (auto == null) return;

        auto.schedule();
    }

    public void teleopInit() {
        if (auto != null) auto.cancel();
        container.led.setRGB(255, 255, 255);
    }

    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
