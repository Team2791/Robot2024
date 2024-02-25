// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathFinder;
import frc.robot.commands.AprilTagCommands.AprilTagDistance;
import frc.robot.commands.AprilTagCommands.AprilTagRotateCommand;
import frc.robot.commands.AprilTagCommands.AprilTagRotateContinuous;
import frc.robot.commands.AprilTagCommands.AprilTagTranslate;
import frc.robot.commands.ArmCommands.ManualAngle;
import frc.robot.commands.ArmCommands.TurretAngle;
import frc.robot.commands.ClimbCommands.Climb;
import frc.robot.commands.ClimbCommands.ClimbRelease;
import frc.robot.commands.ClimbCommands.ClimberActivate;
import frc.robot.commands.ClimbCommands.ManualClimb;
import frc.robot.commands.IntakeCommands.spitOut;
import frc.robot.commands.IntakeCommands.takeIn;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import java.util.logging.Level;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {



	private final SendableChooser<Command> autoChooser;

	private final XboxController m_driverController;
	private final XboxController m_operatorController;
	public static PhotonCamera camera1 = new PhotonCamera("2791camera");

	public static PathFinder pathToAmp = new PathFinder(new Pose2d(14.57,7.21,new Rotation2d(90)));
	public static PathFinder pathToSpeaker = new PathFinder(new Pose2d(15.15,5.52, new Rotation2d(0)));

	// The robot's subsystems
	//private final DriveSubsystem m_robotDrive = new DriveSubsystem();


	// The driver's controller
	
	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT, driverStart, driverBack;
	private Trigger operatorX, opeY, operatorA, operatorB, operatorLB, operatorRB, operatorLT, operatorRT;

	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;
	private Trigger operatorDPadUp, operatorDPadDown, operatorDPadLeft, operatorDPadRight, operatorLeftStick;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_driverController  = new XboxController(OIConstants.kDriverControllerPort);
		m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);



		/* NamedCommands.registerCommand("Shoot", new Shoot());
        NamedCommands.registerCommand("Climb", new Climb());
        NamedCommands.registerCommand("TurretAllign", new TurretAngle(camera1));
		NamedCommands.registerCommand("TagAllignCommand", new AprilTagRotateCommand(camera1)); */

		configureButtonBindings();

		//driver buttons
		driverB.whileTrue(new AprilTagRotateContinuous(camera1));
		driverA.whileTrue(new AprilTagRotateCommand(camera1));
		driverX.whileTrue(new AprilTagTranslate(camera1));
		driverY.whileTrue(new AprilTagDistance(camera1));
		driverStart.toggleOnTrue(pathToAmp);
		driverBack.toggleOnTrue(pathToSpeaker);
		
		//driverDPadUp.toggleOnTrue(new Climb());
		//driverDPadDown.toggleOnTrue(new ClimbRelease());
		//driverRT.toggleOnTrue(new SequentialCommandGroup(new ParallelCommandGroup(new AprilTagRotateCommand(camera1), new AprilTagTranslate(camera1)), new TurretAngle(camera1), new Shoot()));
		//driverA.whileTrue(new takeIn());
		//driverY.whileTrue(new spitOut());


		//Operator buttons
		//operatorDPadUp.whileTrue(new ManualAngle(true));
		//operatorDPadDown.whileTrue(new ManualAngle(false));


		

		
	

		// Configure default commands
		Robot.m_robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(() -> Robot.m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),true, true),Robot.m_robotDrive));


		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);


	
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
		new JoystickButton(m_driverController, Button.kR1.value).whileTrue(new RunCommand(() -> Robot.m_robotDrive.setX(),Robot.m_robotDrive));

		//driver initializations
		driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
		driverB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
		driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
		driverY = new JoystickButton(m_driverController, XboxController.Button.kY.value);
		
		driverDPadUp = new POVButton(m_driverController, 0);
		driverDPadDown = new POVButton(m_driverController, 180);
		driverDPadRight = new POVButton(m_driverController, 90);
		driverDPadLeft = new POVButton(m_driverController, 270);

		driverRT = new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value);
		driverLT = new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value);

		driverStart = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
		driverBack = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

		//operator initializations
		operatorDPadUp = new POVButton(m_operatorController, 0);
		operatorDPadDown = new POVButton(m_operatorController, 180);
	}


	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	
}