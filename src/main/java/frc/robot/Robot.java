// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import org.opencv.video.Video;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
//import frc.robot.subsystems.PhotonEstimator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RGBLED;
import frc.robot.subsystems.Shintake;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static Arm arm;
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;
	public static Shintake shintake;
	public static Climber climber;
	public static DriveSubsystem drivetrain;
	//public static PhotonEstimator estimator;
	//public static PhotonEstimator photonestimator;
	public static PhotonCamera camera1;
	public static PhotonCamera camera2;
	public static PowerDistribution pdp;
	public static RGBLED led;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {

		
		camera1 = new PhotonCamera("2791camera");
		camera1.setDriverMode(false);
		camera2 = new PhotonCamera("drivercam");
		camera2.setDriverMode(true);
		
		shintake = new Shintake();

		
		final UsbCamera drivercam = CameraServer.startAutomaticCapture();
		VideoMode videoMode = new VideoMode(1, 320, 240, 100);//VideoMode.PixelFormat.kMJPEG
		drivercam.setVideoMode(videoMode);
		led = new RGBLED();
		//CameraServer.startAutomaticCapture();
		arm = new Arm();
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		drivetrain = new DriveSubsystem();
		m_robotContainer = new RobotContainer();
		climber = new Climber();

		pdp = new PowerDistribution(1, ModuleType.kRev);
		led.setColor(0, 0, 255);
		//Robot.arm.armLeft.getEncoder().setPosition(0);
		//estimator = new PhotonEstimator(camera1, m_drivetrain);

		//photonestimator = new PhotonEstimator(camera1, m_drivetrain);

		//Constants.ArmConstants.kMinPot = Robot.arm.getRawPivotPot();
		
		arm.ResetPivotEncoder();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {

		// SmartDashboard.putNumber("Radio VRM", pdp.getCurrent(15));
		// SmartDashboard.putNumber("Brain Box", pdp.getCurrent(16));
		// SmartDashboard.putNumber("Raspi VRM", pdp.getCurrent(19));



		
		

		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		led.setColor(0, 255, 0);
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {



		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
		RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
		Robot.led.setColor(255,255, 255);

		//Constants.ArmConstants.kMinPot = Robot.arm.getRawPivotPot();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
