// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
	private CANSparkMax pivotMotor1, pivotMotor2, extendMotor;
	private AnalogPotentiometer pivotPot, extendPot;
	private DigitalInput pivotLimitSwitchFront, pivotLimitSwitchBack, maxExtendButton, minExtendButton;
	private Solenoid pivotbrake;
	private boolean atMaxExtend, atMinExtend, atMaxPivot, atMinPivot;

	public Arm() {
		pivotMotor1 = new CANSparkMax(RobotMap.pivotMotor1, MotorType.kBrushless);
		pivotMotor2 = new CANSparkMax(RobotMap.pivotMotor2, MotorType.kBrushless);
		extendMotor = new CANSparkMax(RobotMap.elevatorMotorID, MotorType.kBrushless);
		pivotMotor1.setSmartCurrentLimit(60);
		pivotMotor2.setSmartCurrentLimit(60);
		extendMotor.setSmartCurrentLimit(60);

		pivotMotor1.setInverted(false);
		pivotMotor2.follow(pivotMotor1, true);

		pivotLimitSwitchBack = new DigitalInput(RobotMap.pivotButtonBack);
		pivotLimitSwitchFront = new DigitalInput(RobotMap.pivotButtonFront);

		maxExtendButton = new DigitalInput(3);
		minExtendButton = new DigitalInput(2);

		pivotPot = new AnalogPotentiometer(0, 270, -148);
		extendPot = new AnalogPotentiometer(3, 40, 0);

		pivotbrake = new Solenoid(PneumaticsModuleType.REVPH, 8);
		// pivotPot = new AnalogPotentiometer(1, 270, -148); //Pivot Potentiometer 2

		setExtendBrakeMode();
		setPivotBrakeMode();
	}

	public void pivotArm(double speed) {
		if (atMaxPivot && speed > 0) {
			speed = 0;
		}
		if (atMinPivot && speed < 0) {
			speed = 0;
		}
		pivotMotor1.set(speed);
	}

	public void extend(double speed) {
		if (atMaxExtend && speed > 0) {
			speed = 0;
		}
		if (atMinExtend && speed < 0) {
			speed = 0;
		}
		extendMotor.set(speed);
	}

	public double getPivotPot() {
		return pivotPot.get() + Constants.newPivotOffset;
	}

	public double getExtendPot() {
		return extendPot.get() - Constants.newExtendOffset;
	}

	public void setPivotBrakeMode() {
		pivotMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pivotMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	public void setExtendBrakeMode() {
		extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	public boolean getPivotButtonBack() {
		return pivotLimitSwitchBack.get();
	}

	public boolean getPivotButtonFront() {
		return pivotLimitSwitchFront.get();
	}

	public void setPivotbrake(boolean state) {
		pivotbrake.set(!state);
	}

	public boolean getMaxExtendButton() {
		return maxExtendButton.get();
	}

	public boolean getMinExtendButton() {
		return minExtendButton.get();
	}

	public boolean getPivotBreak() {
		return !pivotbrake.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Extend Potentiometer", getExtendPot());
		SmartDashboard.putNumber("Pivot Potentiometer", getPivotPot());
		SmartDashboard.putNumber("RPM", extendMotor.getEncoder().getVelocity());
		// SmartDashboard.putBoolean("Pivot Button Front", getPivotButtonBack());
		// SmartDashboard.putBoolean("Pivot Button Back", getPivotButtonFront());

		// SmartDashboard.putBoolean("Minimum Extension Button", getMinExtendButton());
		// SmartDashboard.putBoolean("Max Extension Button", getMaxExtendButton());

		atMaxExtend = (getExtendPot() > Constants.maxExtension) || !getMaxExtendButton();
		atMinExtend = (getExtendPot() < Constants.minExtension);

		atMaxPivot = (getPivotPot() > Constants.maxPivot) || !getPivotButtonFront();
		atMinPivot = (getPivotPot() < Constants.minPivot) || !getPivotButtonBack();

	}

}
