// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbRelease;
import frc.robot.commands.Shoot;
import frc.robot.commands.AprilTagDistance;
import frc.robot.commands.AprilTagRotateCommand;
import frc.robot.commands.AprilTagRotateContinuous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.photonvision.PhotonCamera;
import java.security.spec.KeySpec;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final XboxController m_driverController;
	private final PhotonCamera camera1 = new PhotonCamera("2791camera");

	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();



	//commands
	private final Climber climb = new Climber();

	// The driver's controller
	
	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT;
	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_driverController  = new XboxController(OIConstants.kDriverControllerPort);
		configureButtonBindings();
		driverB.toggleOnTrue(new AprilTagRotateContinuous(camera1, m_robotDrive));
		driverDPadUp.toggleOnTrue(new Climb());
		driverDPadDown.toggleOnTrue(new ClimbRelease());
		driverLT.toggleOnTrue(new Shoot());

		

		NamedCommands.registerCommand("Shoot", new Shoot());
        NamedCommands.registerCommand("Climb", new Climb());
        NamedCommands.registerCommand("TurretAllign", new AprilTagDistance(camera1));
		NamedCommands.registerCommand("TagAllignCommand", new AprilTagRotateCommand(camera1, m_robotDrive));
		// Configure the button bindings
		//driverB.whileTrue(chaseTagCommand);

		// Configure default commands
		m_robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> m_robotDrive.drive(
								-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								true, true),
						m_robotDrive));

	
	}
	

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(m_driverController, Button.kR1.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
		driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
		driverB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
		driverDPadUp = new POVButton(m_driverController, 0);
		driverDPadDown = new POVButton(m_driverController, 180);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	/**
	 * public Command getAutonomousCommand() {
	 * // Create config for trajectory
	 * TrajectoryConfig config = new TrajectoryConfig(
	 * AutoConstants.kMaxSpeedMetersPerSecond,
	 * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
	 * // Add kinematics to ensure max speed is actually obeyed
	 * .setKinematics(DriveConstants.kDriveKinematics);
	 * 
	 * // An example trajectory to follow. All units in meters.
	 * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
	 * // Start at the origin facing the +X direction
	 * new Pose2d(0, 0, new Rotation2d(0)),
	 * // Pass through these two interior waypoints, making an 's' curve path
	 * List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
	 * // End 3 meters straight ahead of where we started, facing forward
	 * new Pose2d(3, 0, new Rotation2d(90)),
	 * config);
	 * 
	 * PIDController xController = new PIDController(AutoConstants.kPXController, 0,
	 * 0);
	 * PIDController yController = new PIDController(AutoConstants.kPYController, 0,
	 * 0);
	 * ProfiledPIDController thetaController = new
	 * ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
	 * AutoConstants.kThetaControllerConstraints);
	 * thetaController.enableContinuousInput(-Math.PI, Math.PI);
	 * 
	 * SwerveControllerCommand swerveControllerCommand = new
	 * SwerveControllerCommand(exampleTrajectory,
	 * m_robotDrive::getPose, DriveConstants.kDriveKinematics, xController,
	 * yController, thetaController,
	 * m_robotDrive::setModuleStates, m_robotDrive);
	 * 
	 * return new SequentialCommandGroup(
	 * new InstantCommand(() ->
	 * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
	 * swerveControllerCommand, new InstantCommand(() ->
	 * m_robotDrive.stopModules()));
	 * 
	 * }
	 */

	public Command getAutonomousCommand() {
		return new PathPlannerAuto("New straight Auto");
	}
}