package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Vision.AprilRotate;
import frc.robot.commands.Vision.AprilTelemetry;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final XboxController driverctl = new XboxController(Constants.Controller.Ports.Driver);
	private final XboxController operctl = new XboxController(Constants.Controller.Ports.Operator);

	// Shared subsystems in RobotContainer
	private final PhotonCamera camera = new PhotonCamera("2791camera");
	private final AHRS gyro = new AHRS(Port.kMXP);
	// private final Climber climber = new Climber(gyro);
	// private final Turret turret = new Turret();
	// private final Shooter shooter = new Shooter();
	private final Drivetrain drivetrain = new Drivetrain(gyro);
	// private final Intake intake = new Intake();
	// private final RGBLED led = new RGBLED();

	private final SendableChooser<Command> autonomous;

	// The robot's subsystems
	//private final DriveSubsystem m_robotDrive = new DriveSubsystem();


	// Controller triggers
	private Trigger driverA, driverB, driverX, driverY, driverLB, driverRB, driverLT, driverRT;
	private Trigger operA, operB, operX, operY, operLB, operRB, operLT, operRT;

	// Controller DPad
	private Trigger driverDPadUp, driverDPadRight, driverDPadDown, driverDPadLeft;
	private Trigger operDPadUp, operDPadRight, operDPadDown, operDPadLeft;

	// Start and Back
	private Trigger driverStart, driverBack;
	private Trigger operStart, operBack;

	public RobotContainer() {
		initTriggers();

		driverB.toggleOnTrue(new FaceTag(camera, drivetrain, true));
		driverY.whileTrue(new AprilTelemetry(camera));

		NamedCommands.registerCommand("FaceTag", new FaceTag(camera, drivetrain, false));

		this.drivetrain.setDefaultCommand(
		    new RunCommand(
		        () -> this.drivetrain.drive(
		            -MathUtil.applyDeadband(driverctl.getLeftY(), Constants.Controller.Deadband),
		            -MathUtil.applyDeadband(driverctl.getLeftX(), Constants.Controller.Deadband),
		            -MathUtil.applyDeadband(driverctl.getRightX(), Constants.Controller.Deadband),
		            true,
		            true
		        ),
		        this.drivetrain
		    )
		);

		autonomous = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autonomous);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by instantiating a
	 * {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
	 * {@link XboxController}), and then calling passing it to a {@link JoystickButton}.
	 */
	private void initTriggers() {
		new JoystickButton(driverctl, Button.kR1.value).whileTrue(new RunCommand(() -> drivetrain.stopX(), drivetrain));

		driverA = new JoystickButton(driverctl, XboxController.Button.kA.value);
		driverB = new JoystickButton(driverctl, XboxController.Button.kB.value);
		driverX = new JoystickButton(driverctl, XboxController.Button.kX.value);
		driverY = new JoystickButton(driverctl, XboxController.Button.kY.value);

		driverDPadUp = new POVButton(driverctl, 0);
		driverDPadDown = new POVButton(driverctl, 180);
		driverDPadRight = new POVButton(driverctl, 90);
		driverDPadLeft = new POVButton(driverctl, 270);

		driverRT = new JoystickButton(driverctl, XboxController.Axis.kRightTrigger.value);
		driverLT = new JoystickButton(driverctl, XboxController.Axis.kLeftTrigger.value);

		driverStart = new JoystickButton(driverctl, XboxController.Button.kStart.value);
		driverBack = new JoystickButton(driverctl, XboxController.Button.kBack.value);

		operDPadUp = new POVButton(operctl, 0);
		operDPadDown = new POVButton(operctl, 180);
	}

	public Command autoCommand() {
		return autonomous.getSelected();
	}
}