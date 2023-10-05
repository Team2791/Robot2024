// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
	public CANSparkMax intakeMotor, rotateMotor;
	public SparkMaxLimitSwitch leftButton, rightButton;

	/** Creates a new Intake. */
	public Intake() {

		intakeMotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
		rotateMotor = new CANSparkMax(RobotMap.intakeRotateMotor, MotorType.kBrushless);
		rotateMotor.setIdleMode(IdleMode.kBrake);

		intakeMotor.setIdleMode(IdleMode.kBrake);
		leftButton = intakeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		rightButton = intakeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

		intakeMotor.setSmartCurrentLimit(60);

	}

	public void setIntakeMotor(double speed) {
		intakeMotor.set(speed);
	}

	public boolean getRightButton() {
		return leftButton.isPressed();
	}

	public boolean getLeftButton() {
		return rightButton.isPressed();
	}

	public double getIntakeCurrent() {
		return intakeMotor.getOutputCurrent();
	}

	public void setRotateMotor(double speed) {
		if (rotateMotor.getMotorTemperature() > 50)
			rotateMotor.set(0);
		else
			rotateMotor.set(speed);
	}

	public double getWristCurrent() {
		return rotateMotor.getOutputCurrent();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("INTAKE BUTTON RIGHT", getRightButton());
		SmartDashboard.putBoolean("INTAKE BUTTON LEFT", getLeftButton());
		SmartDashboard.putNumber("ROTATECURRENT", rotateMotor.getOutputCurrent());
		SmartDashboard.putNumber("WRIST TEMPERATURE", rotateMotor.getMotorTemperature());
		SmartDashboard.putBoolean("TOOHOT", rotateMotor.getMotorTemperature() < 35);

	}
}