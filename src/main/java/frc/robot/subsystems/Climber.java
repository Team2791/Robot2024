package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
	private final CANSparkMax left;
	private final CANSparkMax right;
	private final Servo servo;
	private final AHRS gyro;

	/**
	 * Creates a new Climber.
	 */
	public Climber(AHRS gyro) {
		this.gyro = gyro;

		this.left = new CANSparkMax(Constants.Ids.Climb.Left, MotorType.kBrushless);
		this.right = new CANSparkMax(Constants.Ids.Climb.Right, MotorType.kBrushless);
		this.servo = new Servo(Constants.Ids.Climb.Servo);

		this.left.setIdleMode(IdleMode.kBrake);
		this.right.setIdleMode(IdleMode.kBrake);

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public int bias() {
		return (int) Math.signum(gyro.getRoll());
	}

	public void climb(double lspeed, double rspeed) {
		left.set(lspeed);
		right.set(rspeed);
	}

	public void climb(double speed) {
		this.climb(speed, speed);
	}

	public double leftAmps() {
		return left.getOutputCurrent();
	}

	public double rightAmps() {
		return right.getOutputCurrent();
	}

	// public void setMode(IdleMode mode) throws IllegalArgumentException {
	// 	if (mode != IdleMode.kBrake && mode != IdleMode.kCoast) {
	// 		throw new IllegalArgumentException("Invalid mode");
	// 	}

	// 	left.setIdleMode(mode);
	// 	right.setIdleMode(mode);
	// }

	public boolean locked() {
		return servo.getAngle() > Constants.Climber.DistanceLocked;
	}

	public void lock() {
		servo.set(.8);
	}

	public void unlock() {
		servo.set(-.8);
	}

	public void periodic() {
		SmartDashboard.putNumber("(Climber) Left Motor Amps", leftAmps());
		SmartDashboard.putNumber("(Climber) Right Motor Amps", rightAmps());
		SmartDashboard.putBoolean("(Climber) Locked?", locked());
	}
}
