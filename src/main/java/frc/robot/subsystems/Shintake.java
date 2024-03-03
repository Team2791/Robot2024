// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.ShintakeConstants;

public class Shintake extends SubsystemBase {
	private final CANSparkMax shootLeft;
	private final CANSparkMax shootRight;
	private final CANSparkMax intake;
	private final SparkLimitSwitch beamBreak;
	private final PIDController speedctl;

	/** Creates a new Shooter. */
	public Shintake() {
		shootLeft = new CANSparkMax(21, MotorType.kBrushless);
		shootRight = new CANSparkMax(22, MotorType.kBrushless);
		intake = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
		speedctl = new PIDController(ShintakeConstants.kShooterP, ShintakeConstants.kShooterI,
				ShintakeConstants.kShooterD);
		speedctl.setTolerance(.01);
		shootLeft.setIdleMode(IdleMode.kCoast);
		shootRight.setIdleMode(IdleMode.kCoast);

		beamBreak = shootRight.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
	}

	public void shoot(double leftSpeed, double rightSpeed) {
		shootLeft.set(leftSpeed);
		shootRight.set(-rightSpeed);
	}

	public void shoot(double speed) {
		shoot(speed, speed);
	}

	public void shoot() {
		shoot(1);
	}

	public void intake(double speed) {
		intake.set(speed);
	}

	public void intake() {
		intake(-1);
	}

	public void outtake() {
		intake(0.4);
	}

	public void stopIntake() {
		intake.set(0);
	}

	public void feedToShooter() {
		intake.set(-.3);
	}

	public boolean broken() {
		return !beamBreak.isPressed();
	}

	public void slowOut() {
		intake.set(.2);
	}

	public double getSpeedRight() {
		return shootRight.getEncoder().getVelocity();
	}

	public double getSpeedLeft() {
		return shootLeft.getEncoder().getVelocity();
	}

	public void periodic() {
		// SmartDashboard.putNumber("Left Motor actual velocity", leftMotor.getEncoder().getVelocity());
		// SmartDashboard.putNumber("Right Motor actual Velocity", rightMotor.getEncoder().getVelocity());
		// SmartDashboard.putNumber("Left motor set power", leftMotor.get());
		// SmartDashboard.putNumber("Right motor set power", rightMotor.get());
		// //SmartDashboard.putData("Speed PID", speedController);
		SmartDashboard.putBoolean("Beam Break?", broken());
		SmartDashboard.putNumber("left speed", getSpeedLeft());
		SmartDashboard.putNumber("Right speed", getSpeedRight());
	}
}
