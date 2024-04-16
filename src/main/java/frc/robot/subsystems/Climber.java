// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	// Climbing motors
	private final CANSparkMax leftMotor = new CANSparkMax(41, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(42, MotorType.kBrushless);

	// Linear actuators to lock the climber in place
	private final Servo leftLock;
	private final Servo rightLock;

	private final AHRS gyro = DriveSubsystem.m_gyro;

	public Climber() {
		CommandScheduler.getInstance().registerSubsystem(this);
		leftMotor.setIdleMode(IdleMode.kBrake);
		rightMotor.setIdleMode(IdleMode.kBrake);

		leftLock = new Servo(3);
		rightLock = new Servo(4);

		leftLock.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
		rightLock.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
		leftMotor.setIdleMode(IdleMode.kBrake);
		rightMotor.setIdleMode(IdleMode.kBrake);
		leftMotor.setOpenLoopRampRate(.1);
		rightMotor.setOpenLoopRampRate(.1);
	}

	/**  Used to tilt during climbing and unclimbing */
	public int bias() {
		return (int) Math.signum(this.gyro.getRoll());
	}

	public void setAll(double leftSpeed, double rightSpeed) {
		leftMotor.set(leftSpeed);
		rightMotor.set(rightSpeed);
	}

	public void setAll(double speed) {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public double leftAmps() {
		return leftMotor.getOutputCurrent();
	}

	public double rightAmps() {
		return rightMotor.getOutputCurrent();
	}

	// TODO: Tune
	public boolean locked() {
		return leftLock.getSpeed() > 30;
	}

	public void lock() {
		leftLock.setSpeed(1);
		rightLock.setSpeed(1);
	}

	public void unlock() {
		leftLock.setSpeed(-1);
		rightLock.setSpeed(-1);
	}

	public double getleftPos() {
		return leftMotor.getEncoder().getPosition();
	}

	public double getrightPos() {
		return rightMotor.getEncoder().getPosition();
	}

	public void setLeft(double speed) {
		leftMotor.set(speed);
	}

	public void setRight(double speed) {
		rightMotor.set(speed);
	}

	public void periodic() {
		// SmartDashboard.putNumber("(Clmber) Left Position", getleftPos());
		// SmartDashboard.putNumber("(Climber) Right Position", getrightPos());
	}
}
