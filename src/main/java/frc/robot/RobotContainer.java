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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagCommands.TagAllignContinuous;
import frc.robot.commands.ArmCommands.ManualExtension;
import frc.robot.commands.ArmCommands.ManualAngleDown;
import frc.robot.commands.ArmCommands.ManualAngleUp;
import frc.robot.commands.ArmCommands.Retraction;
import frc.robot.commands.ClimberCommands.Climb;
import frc.robot.commands.ClimberCommands.actuator.LinearLock;
import frc.robot.commands.PitstickCommands.LeftClimbUp;
import frc.robot.commands.PitstickCommands.LeftRelease;
import frc.robot.commands.PitstickCommands.RightClimbUp;
import frc.robot.commands.PitstickCommands.RightRelease;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.commands.ShintakeCommands.Shoot;
// import frc.robot.commands.ShintakeCommands.Shoot;
import frc.robot.commands.ShintakeCommands.SpitOut;
// import frc.robot.commands.servocontrol;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.security.spec.KeySpec;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static XboxController m_driverController =
			new XboxController(OIConstants.kDriverControllerPort);
	public static XboxController m_operatorController =
			new XboxController(OIConstants.kOperatorControllerPort);
	public static XboxController m_pitController =
			new XboxController(OIConstants.kPitStickControllerPort);
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = Robot.m_drivetrain;
	private final PhotonCamera camera1 = new PhotonCamera("2791camera");


	//Commands
	private final TagAllignContinuous tagallign =
			new TagAllignContinuous(camera1, m_robotDrive, m_driverController);
	private final Intake intake = new Intake();
	private final SpitOut spitout = new SpitOut();
	private final Shoot shoot = new Shoot();
	private final ManualAngleUp manualangleup = new ManualAngleUp();
	private final ManualAngleDown manualangledown = new ManualAngleDown();
	private final LeftClimbUp leftclimbup = new LeftClimbUp();
	private final LeftRelease leftrelease = new LeftRelease();
	private final RightClimbUp rightclimbup = new RightClimbUp();
	private final RightRelease rightrelease = new RightRelease();
	private final LinearLock actuate = new LinearLock();

	// private final ManualAngle armup = new ManualAngle(true);
	// private final ManualAngle armdown = new ManualAngle(false);

	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT,
			driverStart, driverBack;
	private Trigger operatorX, operatorY, operatorA, operatorB, operatorLB, operatorRB, operatorLT,
			operatorRT;

	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;
	private Trigger operatorDPadUp, operatorDPadDown, operatorDPadLeft, operatorDPadRight,
			operatorLeftStick;

	private Trigger pitStickRB, pitStickLB, pitDpadRight, pitDpadLeft;



	// The driver's controller


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		NamedCommands.registerCommand("Align", tagallign);
		NamedCommands.registerCommand("Shoot", shoot);
		NamedCommands.registerCommand("Intake", intake);



		// Configure the button bindings
		configureButtonBindings();
		driverB.whileTrue(tagallign);
		driverRB.toggleOnTrue(new Climb(m_driverController));
		driverLB.whileTrue(new frc.robot.commands.ClimberCommands.ClimbRelease());
		operatorX.whileTrue(new Shoot());
		operatorA.whileTrue(intake);
		operatorY.whileTrue(spitout);
		driverDPadRight.whileTrue(new ManualExtension());
		driverDPadLeft.whileTrue(new Retraction());
		driverDPadUp.whileTrue(manualangleup);
		driverDPadDown.whileTrue(manualangledown);


		pitStickLB.whileTrue(leftclimbup);
		pitStickRB.whileTrue(leftrelease);
		pitDpadLeft.whileTrue(rightclimbup);
		pitDpadRight.whileTrue(rightrelease);



		// Configure default commands
		m_robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(() -> m_robotDrive.drive(
						-MathUtil.applyDeadband(m_driverController.getLeftY(),
								OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(m_driverController.getLeftX(),
								OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(m_driverController.getRightX(),
								OIConstants.kDriveDeadband),
						true, true), m_robotDrive));
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
		new JoystickButton(m_driverController, Button.kR1.value)
				.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

		//driver configs
		driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
		driverB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
		driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
		driverY = new JoystickButton(m_driverController, XboxController.Button.kY.value);
		driverDPadUp = new POVButton(m_driverController, 180);
		driverDPadDown = new POVButton(m_driverController, 0);
		driverDPadRight = new POVButton(m_driverController, 90);
		driverDPadLeft = new POVButton(m_driverController, 270);
		driverLB = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
		driverRB = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
		driverRT = new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value);
		driverLT = new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value);

		//operator configs
		operatorA = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
		operatorB = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
		operatorX = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
		operatorY = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
		operatorDPadUp = new POVButton(m_operatorController, 180);
		operatorDPadDown = new POVButton(m_operatorController, 0);
		operatorDPadRight = new POVButton(m_operatorController, 90);
		operatorDPadLeft = new POVButton(m_operatorController, 270);

		//pitStick buttons
		pitStickLB = new JoystickButton(m_pitController, XboxController.Button.kLeftBumper.value);
		pitStickRB = new JoystickButton(m_pitController, XboxController.Button.kRightBumper.value);
		pitDpadLeft = new POVButton(m_pitController, 270);
		pitDpadRight = new POVButton(m_pitController, 90);

	}

	public Command getAutonomousCommand() {
		return new PathPlannerAuto("MidAuto");
	}         
}
