package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private final RobotContainer container = new RobotContainer();
	private Command autonomous = container.autoCommand();

	/** Run once on startup */
	public void robotInit() {
		CameraServer.startAutomaticCapture();
	}

	/** Run every 20ms after startup */
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** Run once after disabling */
	public void disabledInit() {
		// container.setRainbow();
	}

	/** Runs once after switching to autonomous mode. Used to schedule auto commands */
	public void autonomousInit() {
		autonomous = container.autoCommand();

		if (autonomous != null) {
			autonomous.schedule();
		}
	}

	/** Runs once after switching to teleop mode */
	public void teleopInit() {
		if (autonomous != null) {
			autonomous.cancel();
		}
	}


	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	public void testPeriodic() {
	}
}