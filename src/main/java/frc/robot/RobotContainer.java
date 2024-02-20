// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

	// Shared subsystems in RobotContainer
	private final PhotonCamera camera = new PhotonCamera("2791camera");
	private final AHRS gyro = new AHRS(Port.kMXP);
	// private final Climber climber = new Climber(gyro);
	// private final Turret turret = new Turret();
	// private final Shooter shooter = new Shooter();
	private final Drivetrain drivetrain = new Drivetrain(gyro);
	// private final Intake intake = new Intake();
	// private final RGBLED led = new RGBLED();

	// The driver's controller
	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT;
	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;

	// Commands
	private final Command faceTag = new FaceTag(camera, drivetrain, false);
	private final Command faceTagForever = new FaceTag(camera, drivetrain, true);
	// private final Command climbUp = new Climb(climber, gyro, led, true);
	// private final Command climbDown = new Climb(climber, gyro, led, false);
	// private final Command shoot = new Shoot(shooter);
	// private final Command aimTurret = new AimTurret(camera, turret);
	// private final Command spitOut = new SpitOut(intake);
	// private final Command takeIn = new TakeIn(intake, led);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureButtonBindings();
		driverB.toggleOnTrue(faceTagForever);
		// driverDPadUp.toggleOnTrue(climbUp);
		// driverDPadDown.toggleOnTrue(climbDown);
		// driverLT.toggleOnTrue(shoot);

		// NamedCommands.registerCommand("Shoot", shoot);
		// NamedCommands.registerCommand("Climb", climbUp);
		// NamedCommands.registerCommand("AimTurret", aimTurret);
		NamedCommands.registerCommand("FaceTag", faceTag);
		// NamedCommands.registerCommand("SpitOut", spitOut);
		// NamedCommands.registerCommand("TakeIn", takeIn);

		this.drivetrain.setDefaultCommand(
		    new RunCommand(
		        () -> this.drivetrain.drive(
		            -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
		            -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
		            -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband),
		            true,
		            true
		        ),
		        this.drivetrain
		    )
		);
	}

	/**
	 * Sets rgb leds to rainbow
	 */
	public void setRainbow() {
	}


	/**
	 * Use this method to define your button->command mappings. Buttons can be created by instantiating a
	 * {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
	 * {@link XboxController}), and then calling passing it to a {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(controller, Button.kR1.value).whileTrue(
		    new RunCommand(() -> drivetrain.stopX(), drivetrain)
		);
		driverA = new JoystickButton(controller, XboxController.Button.kA.value);
		driverB = new JoystickButton(controller, XboxController.Button.kB.value);
		driverDPadUp = new POVButton(controller, 0);
		driverDPadDown = new POVButton(controller, 180);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	/**
	 * public Command getAutonomousCommand() { // Create config for trajectory TrajectoryConfig config = new
	 * TrajectoryConfig( AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
	 * // Add kinematics to ensure max speed is actually obeyed .setKinematics(DriveConstants.kDriveKinematics); // An
	 * example trajectory to follow. All units in meters. Trajectory exampleTrajectory =
	 * TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction new Pose2d(0, 0, new
	 * Rotation2d(0)), // Pass through these two interior waypoints, making an 's' curve path List.of(new
	 * Translation2d(1, 1), new Translation2d(2, 0)), // End 3 meters straight ahead of where we started, facing forward
	 * new Pose2d(3, 0, new Rotation2d(90)), config); PIDController xController = new
	 * PIDController(AutoConstants.kPXController, 0, 0); PIDController yController = new
	 * PIDController(AutoConstants.kPYController, 0, 0); ProfiledPIDController thetaController = new
	 * ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
	 * thetaController.enableContinuousInput(-Math.PI, Math.PI); SwerveControllerCommand swerveControllerCommand = new
	 * SwerveControllerCommand(exampleTrajectory, m_robotDrive::getPose, DriveConstants.kDriveKinematics, xController,
	 * yController, thetaController, m_robotDrive::setModuleStates, m_robotDrive); return new SequentialCommandGroup(
	 * new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
	 * swerveControllerCommand, new InstantCommand(() -> m_robotDrive.stopModules())); }
	 */

	public Command getAutonomousCommand() {
		return new PathPlannerAuto("New straight Auto");
	}
}