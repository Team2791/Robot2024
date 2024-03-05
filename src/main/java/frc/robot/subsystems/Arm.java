// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
	/** DO NOT USE (follows pivot automatically) */
	private final CANSparkMax pivotFollower = new CANSparkMax(31, MotorType.kBrushless);

	private final CANSparkMax pivot = new CANSparkMax(32, MotorType.kBrushless);
	private final CANSparkMax extend = new CANSparkMax(33, MotorType.kBrushless);

	private final PIDController pivotctl = new PIDController(1, 0, 0);

<<<<<<< HEAD
	private final AnalogPotentiometer pivotPot;
	private final AnalogPotentiometer extendPot;

	private double gravity;
	private double angle;

	public Arm() {
		pivotFollower.follow(pivot, true);
		pivot.setIdleMode(IdleMode.kBrake);
		pivotFollower.setIdleMode(IdleMode.kBrake);
=======
  private double Fg;
  public double setAngle;

  private double slope, intercept;
  private double extSlope, extIntercept;

  /** Creates a new Turret. */
  public Arm() {
    armLeft = new CANSparkMax(32, MotorType.kBrushless);
    armRight = new CANSparkMax(31, MotorType.kBrushless);
    armRight.follow(armLeft, true);
    extensionMotor = new CANSparkMax(33, MotorType.kBrushless);

    slope = (ArmConstants.kMaxAngle - ArmConstants.kMinAngle) / (ArmConstants.kMaxPot - ArmConstants.kMinPot);
    intercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * slope);
    armPot = new AnalogPotentiometer(1, slope, intercept);

    extSlope = 100 / (ArmConstants.kExtendMaxPot - ArmConstants.kExtendMinPot);
    extIntercept = - ArmConstants.kExtendMinPot * extSlope;
    extenpot = new AnalogPotentiometer(0, extSlope, extIntercept);
>>>>>>> refs/remotes/origin/swerveFieldCentric

		double slope = (ArmConstants.kMaxAngle - ArmConstants.kMinAngle)
				/ (ArmConstants.kMaxPot - ArmConstants.kMinPot);

		double intercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * slope);

		pivotPot = new AnalogPotentiometer(1, slope, intercept);
		extendPot = new AnalogPotentiometer(0);

		angle = pivotPot.get();

		CommandScheduler.getInstance().registerSubsystem(this);
	}


	public void setAngle(double angle) {
		pivot.setIdleMode(IdleMode.kCoast);
		pivotctl.setSetpoint(angle);

		this.angle = angle;

		while (!pivotctl.atSetpoint()) {
			gravity = Math.cos(angle);
			pivot.set(pivotctl.calculate(pivotPot.get()));
		}
	}

	public void moveUp() {
		pivot.set(-ArmConstants.kArmSpeedDown);

		angle = pivotPot.get();
	}

	public void moveDown() {
		pivot.set(ArmConstants.kArmSpeedUp);

		angle = pivotPot.get();
	}

	public void hold() {
		pivot.set(0);
	}

	public void coast() {
		pivot.setIdleMode(IdleMode.kBrake);
		pivotFollower.setIdleMode(IdleMode.kBrake);
	}

	public void brake() {
		pivot.setIdleMode(IdleMode.kBrake);
		pivotFollower.setIdleMode(IdleMode.kBrake);
	}

	public double getPivotPot() {
		return pivotPot.get();
	}

	public void manualExtend() {
		extend.set(.1);
	}

	public void manualRetract() {
		extend.set(-.1);
	}

	public double getExtensionPot() {
		return extendPot.get();
	}

	public void stopExtension() {
		extend.set(0);
	}

<<<<<<< HEAD
	public void periodic() {
		SmartDashboard.putNumber("Turret Pot", getPivotPot());
		SmartDashboard.putNumber("Extension Angle", getExtensionPot());
=======
  @Override
  public void periodic() {
    double armPot = getArmPot();
    double extPot = getExtensionPot();
    SmartDashboard.putNumber("Pivot Angle", armPot);
    SmartDashboard.putNumber("Raw pivot pot", (armPot - intercept) / slope);
    SmartDashboard.putNumber("Extension %", extPot);
    SmartDashboard.putNumber("Raw extension pot", (extPot - extIntercept) / extSlope);
    //SmartDashboard.putData("Left Turret PID", leftPID);
    //SmartDashboard.putData("Right Turret PID", rightPID);
>>>>>>> refs/remotes/origin/swerveFieldCentric

		// armLeft.set(leftPID.calculate(armPot.get(),setAngle)+Fg*Constants.ArmConstants.armLFF);
		// armRight.set(rightPID.calculate(turretpot.get(),setAngle)+Fg*Constants.ArmConstants.armRFF);
	}
}
