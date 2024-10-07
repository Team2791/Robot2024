// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer container = new RobotContainer();
    private Command auto;

    /**
     * initialization code.
     */
    public void robotInit() {
    }

    /**
     * Called every 20ms
     */
    @Override
    public void robotPeriodic() {
        // Run the command scheduler
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    public void disabledInit() {
        container.led.setRGB(255, 0, 0);
    }

    /**
     * Runs the autonomous command.
     */
    public void autonomousInit() {
        auto = this.container.getAutonomousCommand();
        if (auto == null) return;

        auto.schedule();
    }

    public void teleopInit() {
        auto.cancel();

        RobotContainer.driverctl.getHID().setRumble(RumbleType.kBothRumble, 0);
        RobotContainer.operctl.getHID().setRumble(RumbleType.kBothRumble, 0);

        container.led.setRGB(255, 255, 255);
    }

    /**
     * This function is called periodically during operator control.
     */
    public void teleopPeriodic() {
    }

    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
