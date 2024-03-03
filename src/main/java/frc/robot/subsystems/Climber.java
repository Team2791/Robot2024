// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ClimberActivate;

public class Climber extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private AnalogPotentiometer leftPot, rightPot;
  AHRS gyro;
  private Servo leftServo;
  private Servo rightServo;
  public static DriveSubsystem drivetrain;

  /** Creates a new Climber. */
  public Climber() {

    leftMotor = new CANSparkMax(41, MotorType.kBrushless);
    rightMotor = new CANSparkMax(42, MotorType.kBrushless);
    //rightMotor.follow(leftMotor, true);
    gyro = Robot.m_drivetrain.m_gyro;
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftServo = new Servo(3);
    rightServo = new Servo(4);

    leftServo.setBoundsMicroseconds(2000,0,0,0,1000);
    rightServo.setBoundsMicroseconds(2000,0,0,0,1000);
  }

  public double getRobotRoll(){
    return gyro.getRoll();
  }

  public double getBias(){
    if(getRobotRoll() > 0){ // if leaning right
      return 1.0;
    }
    else if(getRobotRoll()<0){ // if leaning left
      return -1.0;
    }
    else return 0.0;
  }

  public void climb(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void climb(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public double getLeftMotorCurrent(){
    return leftMotor.getOutputCurrent();
  }
  
  public double getRightMotorCurrent(){
    return rightMotor.getOutputCurrent();
  }
  
  public void setBrakeMode(){
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
  }

  public boolean lockedIN(){
    return leftServo.getAngle()>30;
  }

  public void lockIN(){
    leftServo.setSpeed(-1);
    rightServo.setSpeed(-1);
  }

  public void unlock(){
    leftServo.setSpeed(1);
    rightServo.setSpeed(1);
  }

  public double getLeftPot(){
    return leftPot.get();
  }

  public double getRightPot(){
    return rightPot.get();
  }

  public void setLeftMotor(double speed){
    leftMotor.set(speed);
  }

  public void setRightMotor(double speed){
    rightMotor.set(speed);
  }



  public void periodic(){
    // SmartDashboard.putNumber("Left Climb Motor Current", Robot.climber.getLeftMotorCurrent());
    // SmartDashboard.putNumber("Right Climb Motor Current", Robot.climber.getRightMotorCurrent());
    // SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
    // SmartDashboard.putBoolean("Climber Activation", ClimberActivate.isActive);
    SmartDashboard.putNumber("Left ncoder", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("right encoder", rightMotor.getEncoder().getPosition());
  }

}