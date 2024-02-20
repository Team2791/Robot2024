// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
	private final CANSparkMax intake;

	public Intake() {
		intake = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
	}

	public void setAngle(double angle, double speed) {
		while (intake.getAbsoluteEncoder().getPosition() < angle) {
			intake.set(speed);
		}

		intake.set(0);
	}

	public void setAngle(double angle) {
		setAngle(angle, .5);
	}

	public void takeIn() {
		intake.set(.5);
	}

	public void spitOut() {
		intake.set(-.5);
	}

	public void stop() {
		intake.set(0);
	}

	public double amps() {
		return intake.getOutputCurrent();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("(Intake) Current Angle", intake.getAbsoluteEncoder().getVelocity());
	}

}
