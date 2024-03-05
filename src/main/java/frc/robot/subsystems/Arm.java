// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	private final CANSparkMax left;
	private final CANSparkMax right;
	private final CANSparkMax extension;

	private final AnalogPotentiometer pivpot;
	private final AnalogPotentiometer extpot;

	private double pivPoint;
	private double extPoint;

	private double lastPivPoint;
	private double lastExtPoint;

	public Arm() {
		right = new CANSparkMax(31, MotorType.kBrushless);
		left = new CANSparkMax(32, MotorType.kBrushless);
		extension = new CANSparkMax(33, MotorType.kBrushless);

		right.follow(left, true);

		extension.setIdleMode(IdleMode.kBrake);
		left.setIdleMode(IdleMode.kBrake);
		right.setIdleMode(IdleMode.kBrake);

		double pivrise = ArmConstants.kMaxAngle - ArmConstants.kMinAngle;
		double pivrun = ArmConstants.kMaxPot - ArmConstants.kMinPot;
		double pivslope = pivrise / pivrun;
		double pivintercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * pivslope);

		double extrise = 100;
		double extrun = ArmConstants.kMaxPot - ArmConstants.kMinPot;
		double extslope = extrise / extrun;
		double extintercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * pivslope);

		pivpot = new AnalogPotentiometer(1, pivslope, pivintercept);
		extpot = new AnalogPotentiometer(0, extslope, extintercept);

		pivPoint = pivpot.get();
		extPoint = extpot.get();

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	/** 
	 * Manual trapazoidal
	 * 
	 * Starts at zero speed, ramps to full over 1/3 of the distance
	 * then stays at full speed for 1/3 of the distance, then ramps to zero
	 * 
	 * @param current the current position
	 * @param lastGoal the last setpoint
	 * @param goal the current setpoint
	 * @param maxSpeed the maximum speed to ramp to
	 * @param zeroOffset when current == lastGoal, the output will be zero. Add this to the output to make it non-zero
	 * @return the speed to set the motor
	 */
	private static double trapazoidal(double current, double lastGoal, double goal, double maxSpeed,
			double zeroOffset) {
		double distBegin = Math.abs(current - lastGoal);
		double totalLen = Math.abs(goal - lastGoal);

		if (totalLen == 0)
			return 0;

		if (lastGoal < goal && current > goal)
			return 0;

		if (lastGoal > goal && current < goal)
			return 0;

		if (distBegin < (totalLen / 3)) {
			// line
			double x1 = 0;
			double y1 = zeroOffset;
			double x2 = totalLen / 3;
			double y2 = maxSpeed;

			// input
			double xeq = distBegin;

			// output
			double yeq = ((y2 - y1) / (x2 - x1)) * (xeq - x1) + y1;

			return yeq;
		} else if (distBegin > (2 * (totalLen / 3))) {
			// line
			double x1 = totalLen;
			double y1 = 0;
			double x2 = 2 * (totalLen / 3);
			double y2 = maxSpeed;

			// input
			double xeq = distBegin;

			// output
			double yeq = ((y2 - y1) / (x2 - x1)) * (xeq - x1) + y1;

			return yeq;
		} else {
			return maxSpeed;
		}
	}

	public void pivot(boolean up) {
		pivot(up ? 1 : -1);
	}

	public void pivot(double offset) {
		this.lastPivPoint = this.pivPoint;
		this.pivPoint += offset;

		this.pivPoint = Math.min(ArmConstants.kMaxAngle, this.pivPoint);
		this.pivPoint = Math.max(ArmConstants.kMinAngle, this.pivPoint);
	}

	public boolean atPivot() {
		double tolerance = 5; // degrees
		return Math.abs(getPivot() - pivPoint) < tolerance;
	}

	public double getPivot() {
		return pivpot.get();
	}

	public double getPivotPoint() {
		return pivPoint;
	}

	public void extend(boolean up) {
		extend(up ? 1 : -1);
	}

	public void extend(double offset) {
		this.lastExtPoint = this.extPoint;
		this.extPoint += offset;

		this.extPoint = Math.min(ArmConstants.kMaxExt, this.extPoint);
		this.extPoint = Math.max(ArmConstants.kMinExt, this.extPoint);
	}

	public boolean atExt() {
		double tolerance = 5;
		return Math.abs(getExt() - extPoint) < tolerance;
	}

	public double getExt() {
		return extpot.get();
	}

	public double getExtPoint() {
		return extPoint;
	}

	public void periodic() {
		double piv = trapazoidal(getPivot(), lastPivPoint, pivPoint, 0.15, 0.02);
		double ext = trapazoidal(getPivot(), lastPivPoint, pivPoint, 0.15, 0.02);

		left.set(piv);
		extension.set(ext);

		SmartDashboard.putNumber("(Arm) Pivot Setpoint", pivPoint);
		SmartDashboard.putNumber("(Arm) Extension Setpoint", extPoint);

		SmartDashboard.putNumber("(Arm) Pivot Position", getPivot());
		SmartDashboard.putNumber("(Arm) Extension Position", getExt());

		SmartDashboard.putNumber("(Arm) Pivot Power", piv);
		SmartDashboard.putNumber("(Arm) Extension Power", ext);
	}
}
