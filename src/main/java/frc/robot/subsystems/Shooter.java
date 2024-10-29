// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robotkt.constants.IOConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final CANSparkMax kicker;

    public Shooter() {
        top = new CANSparkMax(IOConstants.Shooter.kTop, MotorType.kBrushless);
        bottom = new CANSparkMax(IOConstants.Shooter.kBottom, MotorType.kBrushless);
        kicker = new CANSparkMax(IOConstants.Shooter.kMiddle, MotorType.kBrushless);

        top.setIdleMode(CANSparkMax.IdleMode.kCoast);
        bottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
        kicker.setIdleMode(CANSparkBase.IdleMode.kCoast);

        bottom.follow(top, true);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        tab.addNumber("Top speed", this.top::get);
        tab.addNumber("Bottom speed", this.bottom::get);
        tab.addNumber("Kicker speed", this.kicker::get);
        tab.addNumber("Kicker current", this.kicker::getOutputCurrent);
    }

    public void setShooter() {
        top.set(ShooterConstants.Speeds.kShooter);
    }

    public void setIntake() {
        top.set(ShooterConstants.Speeds.kIntake);
        kicker.set(ShooterConstants.Speeds.kIntake);
    }

    public void kick() {
        kicker.set(ShooterConstants.Speeds.kKick);
    }

    public void stop() {
        top.set(0);
        kicker.set(0);
    }

    public double getKicker() {
        return kicker.getOutputCurrent();
    }
}