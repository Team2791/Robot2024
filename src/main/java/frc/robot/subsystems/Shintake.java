// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shintake extends SubsystemBase {
	private final CANSparkMax intake;
	private final CANSparkMax top;
	private final CANSparkMax bottom;
	private final DigitalInput beam;

	public Shintake() {
		intake = new CANSparkMax(Constants.Ids.Note.Intake, MotorType.kBrushless);
		top = new CANSparkMax(Constants.Ids.Note.ShooterTop, MotorType.kBrushless);
		bottom = new CANSparkMax(Constants.Ids.Note.ShooterBottom, MotorType.kBrushless);
		beam = new DigitalInput(Constants.Ids.Note.BeamBreak);

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public void setIntake(double speed) {
		intake.set(speed);
	}

	public void setTop(double speed) {
		top.set(speed);
	}

	public void setBottom(double speed) {
		bottom.set(speed);
	}

	public boolean broken() {
		return !beam.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("(Shooter) Beam Broken?", broken());
		SmartDashboard.putNumber("(Shooter) Top", top.get());
		SmartDashboard.putNumber("(Shooter) Bottom", bottom.get());
		SmartDashboard.putNumber("(Shooter) Intake", intake.get());
	}

}
