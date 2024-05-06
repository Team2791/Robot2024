// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	PhotonCamera camera;

	public final CANSparkMax LeftMotor;
	public final CANSparkMax RightMotor;
	public final CANSparkMax ExtensionMotor; 

	public final RelativeEncoder PivotEncoder;
	public final AnalogPotentiometer ExtPot;

	public final PIDController PivotCtl;
	// private PIDController pid;
	public PIDController extensionPID;

	public double setExtension;
	public double setpoint;

	public double power;

	private final double PivSlope, PivIntercept;
	private final double ExtSlope, ExtIntercept;

	/** Creates a new Turret. */
	public Arm() {
		LeftMotor = new CANSparkMax(32, MotorType.kBrushless);
		RightMotor = new CANSparkMax(31, MotorType.kBrushless);
		ExtensionMotor = new CANSparkMax(33, MotorType.kBrushless);

		RightMotor.follow(LeftMotor, true);
		ExtensionMotor.setIdleMode(IdleMode.kBrake);

		PivotEncoder = LeftMotor.getEncoder();

		PivSlope = (ArmConstants.AngleMax - ArmConstants.AngleMin) / (ArmConstants.EncoderMax - ArmConstants.EncoderMin);
		PivIntercept = ArmConstants.AngleMin - (ArmConstants.EncoderMin * PivSlope);

		LeftMotor.setIdleMode(IdleMode.kBrake);
		RightMotor.setIdleMode(IdleMode.kBrake);

		ExtSlope = 100 / (ArmConstants.kExtendMaxPot - ArmConstants.kExtendMinPot);
		ExtIntercept = -ArmConstants.kExtendMinPot * ExtSlope;
		ExtPot = new AnalogPotentiometer(0, ExtSlope, ExtIntercept);

		PivotCtl = new PIDController(.02, 0, 0);
		PivotCtl.setTolerance(1);

		setExtension = getExtensionPot();
		extensionPID = new PIDController(.7, 0, 0);

		setpoint = GetAngle();
	}

	/** MUST BE CALLED BY A PERIODIC OR EXECUTE */
	public void StepTowardsAngle(double angle) {
		LeftMotor.setIdleMode(IdleMode.kBrake);
		power = PivotCtl.calculate(GetAngle(), angle);

		if (power > .5) power = .5;
		else if (power < -.5) power = -.5;

		LeftMotor.set(power);
	}

	public void moveUp(double speed) {
		LeftMotor.set(-speed);
		// setpoint = getArmPot();
	}

	public void moveDown(double speed) {
		LeftMotor.set(speed);
		// setpoint = getArmPot();

	}

	public void hold() {
		LeftMotor.set(0);
		LeftMotor.setIdleMode(IdleMode.kBrake);
		RightMotor.setIdleMode(IdleMode.kBrake);
	}

	public double GetAngle() {
		return (PivotEncoder.getPosition() * PivSlope) + PivIntercept;
	}

	public void manualExtend() {
		ExtensionMotor.set(ArmConstants.kExtensionSpeed);
	}

	public void manualRetract() {
		ExtensionMotor.set(-ArmConstants.kRetractionSpeed);
	}

	public double getExtensionPot() {
		return ExtPot.get();
	}

	public void stopExtension() {
		ExtensionMotor.set(0);
	}

	public double RawPivotEncoder() {
		return PivotEncoder.getPosition();
	}

	public void ResetPivotEncoder() {
		PivotEncoder.setPosition(0);
	}

	@Override
	public void periodic() {
		double angle = GetAngle();
		double extPot = getExtensionPot();

		SmartDashboard.putNumber("Pivot Angle", angle);
		SmartDashboard.putNumber("Pivot Raw Encoder", RawPivotEncoder());
		SmartDashboard.putNumber("Extension", extPot);
		SmartDashboard.putNumber("Raw extension pot", (extPot - ExtIntercept) / ExtSlope);
	}
}