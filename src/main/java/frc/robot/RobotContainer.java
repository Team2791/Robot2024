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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.AprilTagCommands.NoteAlign;
import frc.robot.commands.AprilTagCommands.TagAllignContinuous;
import frc.robot.commands.ArmCommands.AmpPivot;
import frc.robot.commands.ArmCommands.ArmSetAngle;
import frc.robot.commands.ArmCommands.FullExtensionAmp;
import frc.robot.commands.ArmCommands.FullExtensionIntake;
import frc.robot.commands.ArmCommands.FullRetraction;
import frc.robot.commands.ArmCommands.IntakeDownCommand;
import frc.robot.commands.ArmCommands.IntakePivot;
import frc.robot.commands.ArmCommands.PhotonAngle;
import frc.robot.commands.ArmCommands.PhotonAngleCommand;
import frc.robot.commands.ArmCommands.ResetPosition;
import frc.robot.commands.ArmCommands.ManualCommands.ManualAngleDown;
import frc.robot.commands.ArmCommands.ManualCommands.ManualAngleUp;
import frc.robot.commands.ArmCommands.ManualCommands.ManualExtension;
import frc.robot.commands.ArmCommands.ManualCommands.ManualRetraction;
import frc.robot.commands.AutoCommands.IntakeReset;
import frc.robot.commands.AutoCommands.IntakeSequence;
import frc.robot.commands.ClimberCommands.Climb;
import frc.robot.commands.ClimberCommands.activate.ClimberActivate;
import frc.robot.commands.ClimberCommands.activate.ClimberDeactivate;
// import frc.robot.commands.ClimberCommands.ClimbRelease;
import frc.robot.commands.ClimberCommands.actuator.LinearLock;
import frc.robot.commands.ClimberCommands.actuator.LinearUnlock;
import frc.robot.commands.PitstickCommands.LeftClimbUp;
import frc.robot.commands.PitstickCommands.LeftRelease;
import frc.robot.commands.PitstickCommands.RightClimbUp;
import frc.robot.commands.PitstickCommands.RightRelease;
import frc.robot.commands.ShintakeCommands.AmpShoot;
import frc.robot.commands.ShintakeCommands.Intake;
import frc.robot.commands.ShintakeCommands.SetShooter;
import frc.robot.commands.ShintakeCommands.Shoot;
import frc.robot.commands.ShintakeCommands.ShootCommand;
// import frc.robot.commands.ShintakeCommands.Shoot;
import frc.robot.commands.ShintakeCommands.SpitOut;
// import frc.robot.commands.servocontrol;
import frc.robot.subsystems.DriveSubsystem;
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
import java.security.spec.KeySpec;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


	public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	public static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
	public static CommandXboxController m_pitController = new CommandXboxController(2);
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = Robot.m_drivetrain;


	//commands
	private final Intake intake = new Intake();
	private final SpitOut spitout = new SpitOut();
	private final SetShooter shoot = new SetShooter();
	private final ManualAngleUp manualangleup = new ManualAngleUp();
	private final ManualAngleDown manualangledown = new ManualAngleDown();
	private final LeftClimbUp leftclimbup = new LeftClimbUp();
	private final LeftRelease leftrelease = new LeftRelease();
	private final RightClimbUp rightclimbup = new RightClimbUp();
	private final RightRelease rightrelease = new RightRelease();

	private final FullExtensionIntake fullextend = new FullExtensionIntake();
	private final IntakeDownCommand intakedown = new IntakeDownCommand();
	private final IntakePivot intakepivot = new IntakePivot();
	private final FullRetraction fullretraction = new FullRetraction();
	private final LinearLock actuate = new LinearLock();

	// private final ManualAngle armup = new ManualAngle(true);
	// private final ManualAngle armdown = new ManualAngle(false);

	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT,
			driverStart, driverTinyLeft;
	private Trigger operatorX, operatorY, operatorA, operatorB, operatorLB, operatorRB, operatorLT,
			operatorRT;

	private Trigger pitA, pitB, pitX, operatortinyright, operatorTinyLeft;
			
	private Trigger operatorLeftYPos, operatorLeftYNeg, operatorLeftXNeg, operatorLeftXPos, operatorRightXPos, operatorRightXNeg, operatorRightYPos, operatorRightYNeg;

	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;
	private Trigger operatorDPadUp, operatorDPadDown, operatorDPadLeft, operatorDPadRight,
			operatorLeftStick;

	private Trigger pitStickRB, pitStickLB, pitDpadRight, pitDpadLeft;

	private final SendableChooser<Command> autoChooser;

	// The driver's controller


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// NamedCommands.registerCommand("Align", new TagAllignContinuous(Robot.camera1, m_robotDrive, m_driverController));
		// NamedCommands.registerCommand("Shoot", new ShootCommand());
		// NamedCommands.registerCommand("Intake", new Intake());

		NamedCommands.registerCommand("IntakeSequence", new IntakeSequence());
		NamedCommands.registerCommand("IntakeReset", new IntakeReset());
		NamedCommands.registerCommand("SetShooter", new SetShooter());
		NamedCommands.registerCommand("Shoot", new Shoot());
		NamedCommands.registerCommand("Intake", new Intake());
		NamedCommands.registerCommand("Angle", new PhotonAngleCommand());
		NamedCommands.registerCommand("ResetArm", new ResetPosition());

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);





		// Configure the button bindings
		configureButtonBindings();

		driverB.whileTrue(new TagAllignContinuous(Robot.camera1, m_robotDrive, m_driverController));
		driverLB.whileTrue(new Shoot());
		driverDPadLeft.whileTrue(new LeftClimbUp());
		driverDPadRight.whileTrue(new LeftRelease());
		driverDPadUp.whileTrue(new RightRelease());
		driverDPadDown.whileTrue(new RightClimbUp());
		driverTinyLeft.whileTrue(new ResetGyro());

		//operatorX.whileTrue(new SetShooter());
		//operatorX.whileTrue(new PhotonAngle());
		//operatorX.whileFalse(new ArmSetAngle(0));
		operatorA.whileTrue(new ParallelCommandGroup(new IntakeSequence(), new Intake()));
		operatorA.whileFalse(new IntakeReset());
		operatortinyright.whileTrue(new Intake());
		operatorTinyLeft.whileTrue(new SetShooter());
	
		
		operatorY.whileTrue(new SpitOut());
		operatorB.whileTrue(new SequentialCommandGroup(new AmpPivot(), new FullExtensionAmp(), new SetShooter()));
		operatorB.whileFalse(new SequentialCommandGroup(new ParallelCommandGroup(new ResetPosition(), new FullRetraction())));
		operatorRB.whileTrue(new ManualExtension());
		operatorLB.whileTrue(new ManualRetraction());
		operatorLeftYNeg.whileTrue(manualangledown);
		operatorLeftYPos.whileTrue(manualangleup);//manualangleup

		//operatorRightYPos.whileTrue(new ClimberActivate());
		//operatorRightYNeg.whileTrue(new ClimberDeactivate());

		pitStickLB.whileTrue(leftclimbup);
		pitStickRB.whileTrue(leftrelease);
		pitDpadLeft.whileTrue(rightclimbup);
		pitDpadRight.whileTrue(rightrelease);
		pitX.whileTrue(new ShootCommand());
		pitA.toggleOnTrue(new LinearLock());
		pitB.toggleOnTrue(new LinearUnlock());





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
						true, false), m_robotDrive));


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
		driverTinyLeft = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

		//operator configs
		operatorA = m_operatorController.a();
		operatorB = m_operatorController.b();
		operatorY = m_operatorController.y();
		operatorX = m_operatorController.x();
		operatorRB = m_operatorController.rightBumper();
		operatorLB = m_operatorController.leftBumper();
		operatorLT = m_operatorController.leftTrigger(.5);
		operatorRT = m_operatorController.rightTrigger(0.5);
		operatorDPadUp = m_operatorController.povUp();
		operatorDPadDown = m_operatorController.povDown();
		operatorDPadRight = m_operatorController.povRight();
		operatorDPadLeft = m_operatorController.povLeft();
		operatorLeftYPos = m_operatorController.axisGreaterThan(1, 0.4);
		operatorLeftYNeg = m_operatorController.axisLessThan(1, -0.4);
		operatorLeftXPos = m_operatorController.axisGreaterThan(0, .4);
		operatorLeftXNeg = m_operatorController.axisLessThan(0, -.4);
		operatorRightXPos  = m_operatorController.axisGreaterThan(4, 0.4);
		operatorRightXNeg = m_operatorController.axisLessThan(4, -.4);
		operatorRightYPos = m_operatorController.axisGreaterThan(5, .4);
		operatorRightYNeg = m_operatorController.axisLessThan(5, -.4);
		operatortinyright = m_operatorController.start();
		operatorTinyLeft = m_operatorController.back();






		//pitStick buttons
		pitStickLB = m_pitController.leftBumper();
		pitStickRB = m_pitController.rightBumper();
		pitDpadLeft = m_pitController.povLeft();
		pitDpadRight = m_pitController.povRight();

		pitA = m_pitController.a();
		pitB = m_pitController.b();
		pitX = m_pitController.x();

	}

	public Command getAutonomousCommand() {

		return autoChooser.getSelected();

	}
}
