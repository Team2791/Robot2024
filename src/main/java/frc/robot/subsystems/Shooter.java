// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = new CANSparkMax(RobotMap.leftShooterMotor, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotMap.rightShooterMotor, MotorType.kBrushless);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setShooter(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void setShooter(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Motor Speed", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Motor Velocity", rightMotor.getEncoder().getVelocity());
  }

  


}
