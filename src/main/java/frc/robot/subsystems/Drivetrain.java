package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
	private CANSparkMax rightLeader, rightFollower1, rightFollower2;
	private CANSparkMax leftLeader, leftFollower1, leftFollower2;
	public AHRS gyro;
	private RelativeEncoder rightEncoder, leftEncoder;
	private MotorControllerGroup left, right;
	private DifferentialDrive drive;
	PIDController rightPIDController, leftPIDController;
	SimpleMotorFeedforward feedforward;
	DifferentialDriveKinematics kinematics;

	Pose2d pose;
	DifferentialDriveOdometry odometry;

	public Drivetrain() {
		gyro = new AHRS(Port.kMXP);
		// Motors instantiation
		rightLeader = new CANSparkMax(RobotMap.MOTOR4, MotorType.kBrushless);
		//rightFollower1 = new CANSparkMax(RobotMap.rightFollower1,
				//MotorType.kBrushless);
		//rightFollower2 = new CANSparkMax(RobotMap.rightFollower2, MotorType.kBrushless);
		leftLeader = new CANSparkMax(RobotMap.MOTOR1, MotorType.kBrushless);
		leftFollower1 = new CANSparkMax(RobotMap.MOTOR2,
				MotorType.kBrushless);
		leftFollower2 = new CANSparkMax(RobotMap.MOTOR3, MotorType.kBrushless);

		leftFollower1.follow(leftLeader);
		leftFollower2.follow(leftLeader);
		rightFollower1.follow(rightLeader);
		rightFollower2.follow(rightLeader);

		rightLeader.setSmartCurrentLimit(50);
		rightFollower1.setSmartCurrentLimit(50);
		rightFollower2.setSmartCurrentLimit(50);
		leftLeader.setSmartCurrentLimit(50);
		leftFollower1.setSmartCurrentLimit(50);
		leftFollower2.setSmartCurrentLimit(50);

		leftLeader.setInverted(true);
		leftFollower1.setInverted(true);
		leftFollower2.setInverted(true);

		rightLeader.setInverted(false);
		rightFollower1.setInverted(false);
		rightFollower2.setInverted(false);


		
		// leftEncoder = leftLeader.getAlternateEncoder(256); // 2048
		// rightEncoder = rightLeader.getAlternateEncoder(256);
		leftEncoder = leftFollower2.getEncoder();
		rightEncoder = rightFollower2.getEncoder();
		// leftEncoder = leftLeader.getEncoder();
		// rightEncoder = rightLeader.getEncoder();
		// leftEncoder.setInverted(true);
		// rightEncoder.setInverted(false);

		rightEncoder.setPositionConversionFactor(Constants.pconversion);
		leftEncoder.setPositionConversionFactor(Constants.pconversion);
		rightEncoder.setVelocityConversionFactor(Constants.vconversion);
		leftEncoder.setVelocityConversionFactor(Constants.vconversion);

		kinematics = new DifferentialDriveKinematics(Constants.TrackWidth);
		odometry = new DifferentialDriveOdometry(getHeading(), getLeftPosition(), getRightPosition());
		feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);
		leftPIDController = new PIDController(Constants.kP, 0.0, 0.0, 0.02);
		rightPIDController = new PIDController(Constants.kP, 0.0, 0.0, 0.02);
		pose = new Pose2d();

		left = new MotorControllerGroup(leftLeader, leftFollower2);
		right = new MotorControllerGroup(rightLeader, rightFollower2);
		drive = new DifferentialDrive(left, right);
		setBrakeMode();

	}

	public void setMotors(double leftSpeed){//, double rightSpeed) {
		leftLeader.set(leftSpeed);
		//rightLeader.set(rightSpeed);
	}

	public void setOutput(double leftVolts, double rightVolts) {
		leftLeader.set(leftVolts / 12);
		rightLeader.set(rightVolts / 12);
	}

	public void arcadeDrive(double thrust, double turn) {
		drive.arcadeDrive(thrust, turn);
	}

	public void setRampUp(double ramp) {
		leftLeader.setOpenLoopRampRate(ramp);
		leftFollower1.setOpenLoopRampRate(ramp);
leftFollower2.setOpenLoopRampRate(ramp);
		rightLeader.setOpenLoopRampRate(ramp);
		rightFollower1.setOpenLoopRampRate(ramp);
		rightFollower2.setOpenLoopRampRate(ramp);

	}
	public PIDController getLeftPidController() {
		return leftPIDController;
	}

	public PIDController getRightPidController() {
		return rightPIDController;
	}

	public SimpleMotorFeedforward getFeedforward() {
		return feedforward;
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	public Pose2d getPose() {
		return pose;
	}

	public Command followTrajectory(Trajectory t) {
		return new RamseteCommand(
				t,
				this::getPose,
				new RamseteController(),
				feedforward,
				kinematics,
				this::getSpeeds,
				leftPIDController,
				rightPIDController,
				this::setOutput,
				this);
	}

	// Gyro Getters
	public double getAngle() {
		return gyro.getAngle();
	}

	public double getYaw() {
		return gyro.getYaw();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public void resetGyro() {
		gyro.reset();
	}

	public DifferentialDriveWheelSpeeds getSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				getLeftVelocity(), getRightVelocity());
	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-gyro.getAngle());
	}

	public DifferentialDriveOdometry getOdometry() {
		return odometry;
	}

	public double getLeftPosition() {
		return leftEncoder.getPosition();
	}

	public double getRightPosition() {
		return rightEncoder.getPosition();
	}

	public double getAveragePosition() {
		return (getLeftPosition() + getRightPosition()) / 2;
	}

	public double getLeftVelocity() {
		return leftEncoder.getVelocity();
	}

	public double getRightVelocity() {
		return rightEncoder.getVelocity();
	}

	public void resetEncoders() {
		rightEncoder.setPosition(0);
		leftEncoder.setPosition(0);
	}

	public void setBrakeMode() {
		leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftFollower1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightFollower1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftFollower2.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightFollower2.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	public void setCoastMode() {
		leftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftFollower1.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightFollower1.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightFollower2.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftFollower2.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("RightPos", getRightPosition());
		// SmartDashboard.putNumber("LeftPos", getLeftPosition());
		// SmartDashboard.putNumber("PITCH", getPitch());
		// SmartDashboard.putNumber("ANGLE", getAngle());
		// SmartDashboard.putNumber("YAW", getYaw());

		pose = odometry.update(getHeading(), getLeftPosition(), getRightPosition());
	}
}