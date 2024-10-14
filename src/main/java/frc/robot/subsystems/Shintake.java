// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShintakeConstants;
import frc.robotkt.constants.IOConstants;

public class Shintake extends SubsystemBase {
    private final CANSparkMax left;
    private final CANSparkMax right;
    private final CANSparkMax intake;
    private final SparkLimitSwitch beam;

    private final double shooterSpeed = 0;

    public Shintake() {
        left = new CANSparkMax(IOConstants.Shintake.kLeftShooter, MotorType.kBrushless);
        right = new CANSparkMax(IOConstants.Shintake.kRightShooter, MotorType.kBrushless);
        intake = new CANSparkMax(IOConstants.Shintake.kIntake, MotorType.kBrushless);
        beam = right.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

        left.setSmartCurrentLimit(ShintakeConstants.kCurrentLimit);
        right.setSmartCurrentLimit(ShintakeConstants.kCurrentLimit);
        left.setIdleMode(CANSparkMax.IdleMode.kCoast);
        right.setIdleMode(CANSparkMax.IdleMode.kCoast);

        left.follow(right, false);
        right.setInverted(true);
        left.setInverted(true);

        ShuffleboardTab tab = Shuffleboard.getTab("Shintake");

        tab.addNumber("Shooter Speed", this::getShooter);
        tab.addNumber("Intake Speed", this::getIntake);
        tab.addBoolean("Loaded", this::isLoaded);
    }

    public double getShooter() {
        return right.getEncoder().getVelocity();
    }

    public void setShooter(double speed) {
        right.set(speed);
    }

    public void stopShooter() {
        setShooter(0);
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