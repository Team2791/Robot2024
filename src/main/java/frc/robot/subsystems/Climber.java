// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotkt.constants.IOConstants;
import frc.robotkt.subsystems.Drivetrain;

public class Climber extends SubsystemBase {
    // Climbing motors
    private final CANSparkMax leftMotor = new CANSparkMax(IOConstants.Climber.kLeft, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(IOConstants.Climber.kRight, MotorType.kBrushless);

    // Linear actuators to lock the climber in place
    private final Servo leftLock = new Servo(IOConstants.Climber.kLeftLock);
    private final Servo rightLock = new Servo(IOConstants.Climber.kRightLock);

    private final AHRS gyro;

    public Climber(Drivetrain drivetrain) {
        gyro = drivetrain.getGyro();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftLock.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
        rightLock.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setOpenLoopRampRate(.1);
        rightMotor.setOpenLoopRampRate(.1);

        ShuffleboardTab tab = Shuffleboard.getTab("climber");

        // No need to update in periodic, shuffleboard saves the lambda
        tab.addNumber("Left position", this::getLeftPos);
        tab.addNumber("Right position", this::getRightPos);
    }

    /// A number between -1 to 1 representing the roll
    public double bias() {
        return (this.gyro.getRoll() / 180.0);
    }

    public void set(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public void set(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public double leftAmps() {
        return leftMotor.getOutputCurrent();
    }

    public double rightAmps() {
        return rightMotor.getOutputCurrent();
    }

    public void lock() {
        leftLock.setSpeed(1);
        rightLock.setSpeed(1);
    }

    public void unlock() {
        leftLock.setSpeed(-1);
        rightLock.setSpeed(-1);
    }

    public double getLeftPos() {
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightPos() {
        return rightMotor.getEncoder().getPosition();
    }

    public void setLeft(double speed) {
        leftMotor.set(speed);
    }

    public void setRight(double speed) {
        rightMotor.set(speed);
    }
}
