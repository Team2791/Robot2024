// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotkt.constants.IOConstants;
import frc.robotkt.subsystems.Drivetrain;
import kotlin.Pair;

class ClimberModule {
    final CANSparkMax motor;
    final RelativeEncoder encoder;
    final Servo servo;

    ClimberModule(int motor, int servo, boolean reversed) {
        this.motor = new CANSparkMax(motor, MotorType.kBrushless);
        this.servo = new Servo(servo);
        this.encoder = this.motor.getEncoder();

        this.motor.setInverted(reversed);
        this.motor.setIdleMode(IdleMode.kBrake);
        this.encoder.setPosition(0);
        this.servo.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
    }

    void set(double speed) {
        motor.set(speed);
    }

    void lock(boolean lock) {
        servo.setSpeed(lock ? 1 : 0);
    }

    void lock() {
        lock(true);
    }

    void unlock() {
        lock(false);
    }

    double speed() {
        return motor.get();
    }

    double position() {
        return encoder.getPosition();
    }

    void register(ShuffleboardTab tab, String prefix) {
        tab.addNumber(prefix + " position", this::position);
        tab.addNumber(prefix + " power", this::speed);
    }
}

public class Climber extends SubsystemBase {
    private final ClimberModule left = new ClimberModule(IOConstants.Climber.kLeft, IOConstants.Climber.kLeftLock, true);
    private final ClimberModule right = new ClimberModule(IOConstants.Climber.kRight, IOConstants.Climber.kRightLock, false);

    private final AHRS gyro;

    public Climber(Drivetrain drivetrain) {
        gyro = drivetrain.getGyro();

        ShuffleboardTab tab = Shuffleboard.getTab("climber");
        left.register(tab, "left");
        right.register(tab, "right");
    }

    /// A number between -1 to 1 representing the roll
    public double bias() {
        return (this.gyro.getRoll() / 180.0);
    }

    public void set(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed);
        right.set(rightSpeed);
    }

    public void set(double speed) {
        this.set(speed, speed);
    }

    public void setLeft(double speed) {
        left.set(speed);
    }

    public void setRight(double speed) {
        right.set(speed);
    }

    public void lock() {
        left.lock();
        right.lock();
    }

    public void unlock() {
        left.unlock();
        right.unlock();
    }

    /// (left, right)
    public Pair<Double, Double> speeds() {
        return new Pair<>(this.left.speed(), this.right.speed());
    }

    /// (left, right)
    public Pair<Double, Double> positions() {
        return new Pair<>(this.left.position(), this.right.position());
    }
}
