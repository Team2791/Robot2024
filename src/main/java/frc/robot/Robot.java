// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
enum RGBMode {
	RAINBOW, SHOOT, SHOOT_RAINBOW
}

public class Robot extends TimedRobot {

	private Command m_autonomousCommand;
	public static Drivetrain drivetrain;
	public static boolean orientation = true;
	private SendableChooser<Command> autoChooser;
	public static boolean currentcheck;
	public static Trajectory revgo, go, wcurve, parkReturn, outAndReturn;

	public static Command forward, back, TwoPieceLeft1, TwoPieceLeft2, TwoPieceRight1, TwoPieceRight2;
	public static String trajectoryJSON = "paths/Joe.wpilib.json";
	public static PhotonCamera camera;
	private static Compressor compressor;
	public PowerDistribution pdp;
	private Timer wristTimer = new Timer();
	private boolean wristChange = false;
	public static double scoringPreset;
	public static double intakePresetFront;
	public static double intakePresetBack;
	public static double stationPreset;
	public static double midScoringPreset;

	@Override
	public void robotInit() {
		CameraServer.startAutomaticCapture(0);
		CameraServer.startAutomaticCapture(1);

		pdp = new PowerDistribution(1, ModuleType.kRev);
		compressor = new Compressor(PneumaticsModuleType.REVPH);
		drivetrain = new Drivetrain();
		new RobotContainer();
		autoChooser = new SendableChooser<>();
		drivetrain.resetGyro();
		drivetrain.resetEncoders();

		// autoChooser.setDefaultOption("(CENTER) Balance No Mobility", new
		// MiddleBalanceAuto());
		// autoChooser.addOption("(CENTER) CUBE + Balance + Mobility", new
		// CubeBalance());

		// autoChooser.addOption("(Non-Wire side) Cone + Cube", new ConeCube());
		// autoChooser.addOption("(RED)(Wire side) Cone Cube", new ConeCubeWire());
		// autoChooser.addOption("(BLUE)(Wire Side) Cone + Cube", new
		// ConeCubeWireBLUE());
		SmartDashboard.putData(autoChooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		compressor.enableAnalog(60, 110);

		// if (intake.intakeMotor.getOutputCurrent() > Constants.IntakeMaxCurrent)
		// 	RobotContainer.driver.setRumble(RumbleType.kBothRumble, 1);
		// else
		// 	RobotContainer.driver.setRumble(RumbleType.kBothRumble, 0);

		SmartDashboard.putNumber("PReSSURE", compressor.getPressure());

		// if (intake.getWristCurrent() > 40) {
		// wristTimer.reset();
		// wristTimer.start();
		// wristChange = true;
		// }
		// if (wristTimer.get() > 3 && wristChange) {
		// intake.setRotateMotor(0);
		// }

		// if (wristChange && intake.getWristCurrent() < 30) {
		// wristChange = false;
		// wristTimer.reset();
		// }
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		Robot.drivetrain.setBrakeMode();
		pdp.setSwitchableChannel(false);
	}

	// @Override
	// public void disabledPeriodic() {
	// 	led.setMode("rainbow");
	// }

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		drivetrain.setRampUp(0);

		pdp.setSwitchableChannel(true);
		Constants.ManualExtend = true;
		drivetrain.resetGyro();
		m_autonomousCommand = autoChooser.getSelected();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		Constants.ManualExtend = true;
		drivetrain.setRampUp(0.25);
		pdp.setSwitchableChannel(true);
		Robot.drivetrain.setBrakeMode();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		if (drivetrain.getYaw() > -90 && drivetrain.getYaw() < 90)
			orientation = false;
		else
			orientation = true;

		// if (drivetrain.getYaw() > -90 || drivetrain.getYaw() < 90) // Back
		// orientation = false;
		// if (drivetrain.getYaw() < -90 || drivetrain.getYaw() > 90) // Front
		// orientation = true;

		if (Constants.mode) {
			intakePresetFront = Constants.ConeIntakePivotFront;
			intakePresetBack = Constants.ConeIntakePivotBack;
			if (orientation) {
				scoringPreset = Constants.ConeScoringPivotFront;
				stationPreset = Constants.NewStationPivotBack;
				midScoringPreset = Constants.MidConeScoringPivotFront;
			} else {
				scoringPreset = Constants.ConeScoringPivotBack;
				stationPreset = Constants.NewStationPivotFront;
				midScoringPreset = Constants.MidConeScoringPivotBack;
			}
		}

		else {
			intakePresetFront = Constants.CubeIntakePivotFront;
			intakePresetBack = Constants.CubeIntakePivotBack;
			if (orientation) {
				scoringPreset = Constants.CubeScoringPivotFront;
				stationPreset = Constants.NewStationPivotBack;
				midScoringPreset = Constants.MidConeScoringPivotFront;

			} else {
				scoringPreset = Constants.CubeScoringPivotBack;
				stationPreset = Constants.NewStationPivotFront;
				midScoringPreset = Constants.MidConeScoringPivotBack;

			}
		}

		double thrust = 0;
		double turn = 0;
		if (RobotContainer.driver.getRawButton(6))
			thrust = -Constants.kCreep;
		else if (RobotContainer.driver.getRawButton(5))
			thrust = Constants.kCreep;
		else {
			thrust = -(RobotContainer.driver.getRawAxis(3) - RobotContainer.driver.getRawAxis(2));
			if (Math.abs(thrust) < 0.25)
				thrust = 0;
			if (thrust > Constants.MaxSpeed)
				thrust = Constants.MaxSpeed;
			if (thrust < -Constants.MaxSpeed)
				thrust = -Constants.MaxSpeed;
		}
		if (Math.abs(RobotContainer.driver.getRawAxis(0)) > 0.1)
			turn = RobotContainer.driver.getLeftX() * Constants.TURN_FACTOR;
		if (Math.abs(RobotContainer.driver.getRightX()) > 0.12)
			turn = RobotContainer.driver.getRightX() * -.35;

		if (RobotContainer.driver.getRawButton(2))
			turn /= 3.5;

		Robot.drivetrain.arcadeDrive(thrust, turn);

		// if (Constants.mode && !Constants.Intaking)
		// 	Robot.led.setColor(255, 255, 0); // Cone Lights
		// if (!Constants.mode && !Constants.Intaking)
		// 	Robot.led.setColor(128, 0, 128); // Cube Lights

	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
