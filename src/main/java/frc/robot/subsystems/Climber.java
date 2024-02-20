// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
	private final CANSparkMax left;
	private final CANSparkMax right;
	private final AHRS gyro;

	/**
	 * Creates a new Climber.
	 */
	public Climber(AHRS gyro) {
		this.gyro = gyro;

		this.left = new CANSparkMax(RobotMap.leftClimbMotor, MotorType.kBrushless);
		this.right = new CANSparkMax(RobotMap.rightClimbMotor, MotorType.kBrushless);

		this.left.setIdleMode(IdleMode.kBrake);
		this.right.setIdleMode(IdleMode.kBrake);
	}

	public int bias() {
		return (int) Math.signum(gyro.getRoll());
	}

	public void climb(double speedLeft, double speedRight) {
		left.set(speedLeft);
		right.set(speedRight);
	}

	public void climb(double speed) {
		climb(speed, speed);
	}

	public double leftAmps() {
		return left.getOutputCurrent();
	}

	public double rightAmps() {
		return right.getOutputCurrent();
	}

	public void setMode(IdleMode mode) throws IllegalArgumentException {
		if (mode != IdleMode.kBrake && mode != IdleMode.kCoast) {
			throw new IllegalArgumentException("Invalid mode");
		}

		left.setIdleMode(mode);
		right.setIdleMode(mode);
	}

	public void periodic() {
		SmartDashboard.putNumber("(Climber) Left Motor Amps", leftAmps());
		SmartDashboard.putNumber("(Climber) Right Motor Amps", rightAmps());
	}
}
