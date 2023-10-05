
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import frc.robot.commands.AutoCommands.Level;
import frc.robot.commands.DrivetrainCommands.ResetGyro;
import frc.robot.commands.VisionCommands.PhotonAim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	public static PhotonCamera camera, camera2;
	public static CommandXboxController operator, pitstick;
	public static XboxController driver;
	private Trigger driverX, driverY, driverA, driverB, driverLB, driverRB, driverLT, driverRT;
	private Trigger driverDPadUp, driverDPadDown, driverDPadLeft, driverDPadRight, driverLeftStick;
	private Trigger driverLeftX, driverLeftY, driverRightX, driverRightY, driverStart, driverTinyRight;

	private Trigger operatorX, operatorY, operatorA, operatorB, operatorLB, operatorRB, operatorLT,
			operatorRT, driverBack;
	private Trigger operatorLeftXPos, operatorLeftXNeg, operatorLeftYNeg, operatorLeftYPos, operatorLeftY,
			operatorRightX, operatorRightYPos, operatorRightYNeg;;
	private Trigger operatorDPadUp, operatorDPadDown, operatorDPadLeft, operatorDPadRight,
			operatorLeftStick, operatorRightStick, operatorTinyRight, operatorTinyLeft;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * // * @throws IOException
	 */
	public RobotContainer() {
		driver = new XboxController(0);
		operator = new CommandXboxController(1);
		pitstick = new CommandXboxController(2);
		configureBindings();
		pitstickBindings();
		camera = new PhotonCamera("2791photonvision1");
		camera2 = new PhotonCamera("2791photonvision");

		// DRIVER CONTROLS
		// driverX.toggleOnTrue(new ChangeMode());
		// driverA.whileTrue(new ConeOutCubeOut());
		// driverY.whileTrue(new ConeInCubeOut());
		driverStart.whileTrue(new Level());


		// OPERATOR CONTROLS
		// operatorDPadUp.toggleOnTrue(new ParallelCommandGroup(new PivotArm(0), new ExtendArm(2)));
		// operatorDPadDown.toggleOnTrue(new ExtendArm(3));
		// operatorA.toggleOnTrue(new RotateWrist());
		// operatorRT.toggleOnTrue(new ScoringExtension());
		// operatorLT.toggleOnTrue(new ExtendArm(Constants.ConeMidExtension));

		// operatorRB.toggleOnTrue(new PivotSet(1, false));
		// operatorLB.toggleOnTrue(new PivotSet(4, false));
		// operatorB.toggleOnTrue(new ParallelCommandGroup(new ExtendArm(0.8), new PivotSet(2, false)));
		// operatorY.toggleOnTrue(new PivotSet(5,false));
		// // Intake Presets
		// operatorTinyLeft.toggleOnTrue(
		// 		new ParallelCommandGroup(new PivotArm(Constants.FlangeIntakePivotBack), new ExtendArm(0.8)));
		// operatorTinyRight.toggleOnTrue(
		// 		new ParallelCommandGroup(new PivotArm(Constants.FlangeIntakePivotFront), new ExtendArm(0.8)));
		// operatorDPadRight.toggleOnTrue(
		// 		new ParallelCommandGroup(new ExtendArm(0.8), new PivotSet(3, true)));
		// operatorDPadLeft.toggleOnTrue(
		// 		new ParallelCommandGroup(new ExtendArm(0.8), new PivotSet(3, false)));

		// operatorLeftYNeg.whileTrue(new ManualPivotDown());
		// operatorLeftYPos.whileTrue(new ManualPivotUp());
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */

	private void pitstickBindings() {
		// pitstick.rightBumper().toggleOnTrue(new PivotArm(Robot.scoringPreset));
		// pitstick.b().toggleOnTrue(new PivotArm(Robot.stationPreset));

		// pitstick.povDownRight().toggleOnTrue(new PivotArm(Robot.intakePresetFront));
		// pitstick.povDownLeft().toggleOnTrue(new PivotArm(Robot.intakePresetBack));

		// pitstick.rightBumper().toggleOnTrue(new	PivotArm(Constants.ConeScoringPivotFront));
		// pitstick.leftBumper().toggleOnTrue(new PivotArm(Constants.ConeScoringPivotBack));
		// pitstick.rightTrigger().toggleOnTrue(new ExtendArm(Constants.ConeUpperExtension));
		// pitstick.leftTrigger().toggleOnTrue(new	ExtendArm(Constants.ConeMidExtension));
		// pitstick.povUp().toggleOnTrue(new ParallelCommandGroup(new PivotArm(0), new ExtendArm(4)));
		// pitstick.povDown().toggleOnTrue(new ExtendArm(3));


		// pitstick.povDownLeft().toggleOnTrue(new ParallelCommandGroup(new ExtendArm(0.3), new PivotArm(Constants.ConeIntakePivotBack)));
		// pitstick.povDownRight().toggleOnTrue(new ParallelCommandGroup(new ExtendArm(0.3), new PivotArm(Constants.ConeIntakePivotFront)));
	
	}

	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		driverA = new JoystickButton(driver, XboxController.Button.kA.value);
		driverB = new JoystickButton(driver, XboxController.Button.kB.value);
		driverX = new JoystickButton(driver, XboxController.Button.kX.value);
		driverY = new JoystickButton(driver, XboxController.Button.kY.value);

		driverLT = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
		driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
		driverRT = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
		driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
		driverDPadDown = new POVButton(driver, 180);
		driverDPadUp = new POVButton(driver, 0);
		driverDPadRight = new POVButton(driver, 90);
		driverDPadLeft = new POVButton(driver, 270);
		driverLeftX = new JoystickButton(driver, XboxController.Axis.kLeftX.value);
		driverLeftY = new JoystickButton(driver, XboxController.Axis.kLeftY.value);
		driverRightX = new JoystickButton(driver, XboxController.Axis.kRightX.value);
		driverRightY = new JoystickButton(driver, XboxController.Axis.kRightY.value);
		driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
		driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
		operatorA = operator.a();
		operatorB = operator.b();
		operatorY = operator.y();
		operatorX = operator.x();

		operatorDPadUp = operator.povUp();
		operatorDPadDown = operator.povDown();
		operatorDPadRight = operator.povRight();
		operatorDPadLeft = operator.povLeft();

		operatorLT = operator.leftTrigger(0.5);
		operatorRT = operator.rightTrigger(0.5);
		operatorRB = operator.rightBumper();
		operatorLB = operator.leftBumper();

		operatorLeftXPos = operator.axisGreaterThan(0, 0.4);
		operatorLeftXNeg = operator.axisLessThan(0, -0.4);

		operatorLeftYPos = operator.axisGreaterThan(1, 0.4);
		operatorLeftYNeg = operator.axisLessThan(1, -0.4);

		operatorTinyLeft = operator.back();
		operatorTinyRight = operator.start();
		operatorRightYPos = operator.axisGreaterThan(5, 0.3);
		operatorRightYNeg = operator.axisGreaterThan(5, -0.3);

	}
}
