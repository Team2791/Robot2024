// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
	private CANSparkMax motor;

	public Turret() {
		motor = new CANSparkMax(RobotMap.turretMotor, MotorType.kBrushless);
	}

	public void setAngle(double angle, double tolerance) {
		boolean inRange = angle() < angle + tolerance && angle() > angle - tolerance;

		while (!inRange) {
			if (angle() < angle) motor.set(.1);
			else motor.set(-.1);
		}
	}

	public void setAngle(double angle) {
		setAngle(angle, 10);
	}

	public double angle() {
		return motor.getAbsoluteEncoder().getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("(Turret) Current Angle", angle());
	}
}
