// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private CANSparkMax left;
	private CANSparkMax right;

	/** Creates a new Shooter. */
	public Shooter() {
		left = new CANSparkMax(Constants.Ids.Shoot.Left, MotorType.kBrushless);
		right = new CANSparkMax(Constants.Ids.Shoot.Right, MotorType.kBrushless);

		left.setIdleMode(IdleMode.kCoast);
		right.setIdleMode(IdleMode.kCoast);

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public void setShooter(double speed) {
		left.set(speed);
		right.set(speed);
	}

	public void setShooter(double leftSpeed, double rightSpeed) {
		left.set(leftSpeed);
		right.set(rightSpeed);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("(Shooter) Left Motor Speed", left.getEncoder().getVelocity());
		SmartDashboard.putNumber("(Shooter) Right Motor Speed", right.getEncoder().getVelocity());
	}
}
