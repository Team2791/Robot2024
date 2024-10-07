// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShintakeConstants;
import frc.robotkt.constants.IOConstants;
import frc.robotkt.constants.PidConstants;

public class Shintake extends SubsystemBase {
    private final CANSparkMax left;
    private final CANSparkMax right;
    private final CANSparkMax intake;
    private final SparkLimitSwitch beam;

    private final SparkPIDController shooterctl;
    private double shooterSpeed = 0;

    public Shintake() {
        left = new CANSparkMax(IOConstants.Shintake.kLeftShooter, MotorType.kBrushless);
        right = new CANSparkMax(IOConstants.Shintake.kRightShooter, MotorType.kBrushless);
        intake = new CANSparkMax(IOConstants.Shintake.kIntake, MotorType.kBrushless);
        beam = right.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooterctl = right.getPIDController();

        left.setSmartCurrentLimit(ShintakeConstants.kCurrentLimit);
        right.setSmartCurrentLimit(ShintakeConstants.kCurrentLimit);
        left.setIdleMode(CANSparkMax.IdleMode.kCoast);
        right.setIdleMode(CANSparkMax.IdleMode.kCoast);

        left.follow(right, true);
        right.setInverted(true);

        shooterctl.setP(PidConstants.Shintake.kShooterP);
        shooterctl.setI(PidConstants.Shintake.kShooterI);
        shooterctl.setD(PidConstants.Shintake.kShooterD);
        shooterctl.setFF(PidConstants.Shintake.kShooterFF);
        shooterctl.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);

        ShuffleboardTab tab = Shuffleboard.getTab("Shintake");

        // Lambda expressions only need to be sent once
        tab.addNumber("Shooter Speed", this::getShooter);
        tab.addNumber("Intake Speed", this::getIntake);
        tab.addBoolean("Loaded", this::isLoaded);
    }

    public double getShooter() {
        return right.getEncoder().getVelocity();
    }

    public void setShooter(double speed) {
        shooterSpeed = speed;
        shooterctl.setReference(speed, ControlType.kVelocity);
    }

    public void stopShooter() {
        setShooter(0);
    }

    public boolean atShooterTarget() {
        return Math.abs(shooterSpeed - right.getEncoder().getVelocity()) < ShintakeConstants.kTolerance;
    }

    public double getIntake() {
        return intake.getEncoder().getVelocity();
    }

    public void setIntake(double speed) {
        intake.set(speed);
    }

    public void stopIntake() {
        setIntake(0);
    }

    public boolean isLoaded() {
        return !beam.isPressed();
    }
}