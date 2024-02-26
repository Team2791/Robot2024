package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
	PhotonCamera camera;

	private final CANSparkMax left;
	private final CANSparkMax right;
	private final AnalogPotentiometer pot;

	private final PIDController leftctl;
	private final PIDController rightctl;

	private double ff;
	public double angle;

	/** Creates a new Turret. */
	public Arm() {
		left = new CANSparkMax(Constants.Ids.Arm.LeftMotor, MotorType.kBrushless);
		right = new CANSparkMax(Constants.Ids.Arm.RightMotor, MotorType.kBrushless);
		pot = new AnalogPotentiometer(Constants.Ids.Arm.Potentiometer, 90, 244);

		left.setIdleMode(IdleMode.kBrake);
		right.setIdleMode(IdleMode.kBrake);

		leftctl = new PIDController(Constants.PID.Arm.Left.P, Constants.PID.Arm.Left.I, Constants.PID.Arm.Left.D);
		rightctl = new PIDController(Constants.PID.Arm.Right.P, Constants.PID.Arm.Right.I, Constants.PID.Arm.Right.D);

		angle = pot.get();
	}


	public void setAngle(double angle) {
		left.setIdleMode(IdleMode.kCoast);
		right.setIdleMode(IdleMode.kCoast);

		this.angle = angle;

		while (!leftctl.atSetpoint() && !rightctl.atSetpoint()) {
			ff = Math.cos(angle);

			left.set(leftctl.calculate(pot.get(), angle));
			right.set(rightctl.calculate(pot.get(), angle));
		}
	}

	public void up() {
		// TODO: does this need to be here?
		left.setIdleMode(IdleMode.kCoast);
		right.setIdleMode(IdleMode.kCoast);

		left.set(.01);
		right.set(.01);

		angle = pot.get();
	}

	public void down() {
		// TODO: does this need to be here?
		left.setIdleMode(IdleMode.kCoast);
		right.setIdleMode(IdleMode.kCoast);

		left.set(-.01);
		right.set(-.01);

		angle = pot.get();
	}

	public void hold() {
		left.set(0);
		right.set(0);

		left.setIdleMode(IdleMode.kBrake);
		right.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Turret Angle", pot.get());
		SmartDashboard.putData("Left Turret PID", leftctl);
		SmartDashboard.putData("Right Turret PID", rightctl);

		left.set(leftctl.calculate(pot.get(), angle) + ff * Constants.PID.Arm.Left.FF);
		right.set(rightctl.calculate(pot.get(), angle) + ff * Constants.PID.Arm.Right.FF);
	}
}
