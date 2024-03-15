// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shintake extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANSparkMax intakeMotor;
  private SparkLimitSwitch beamBrake;
  public PIDController speedController;
  double power;
  Timer timer = new Timer();
  /** Creates a new Shooter. */
  public Shintake() {
    leftMotor = new CANSparkMax(21, MotorType.kBrushless);
    rightMotor = new CANSparkMax(22, MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);
    intakeMotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
    speedController = new PIDController(Constants.ShintakeConstants.kShooterP, Constants.ShintakeConstants.kShooterI, Constants.ShintakeConstants.kShooterD);
    speedController.setTolerance(.01);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    beamBrake = rightMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    }

  // public void setShooter(double speed){
  //   leftMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),speed));
  //   rightMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),speed));
  // }

  // public void setShooter(double leftSpeed, double rightSpeed){
  //   leftMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),leftSpeed));
  //   rightMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),rightSpeed));
  // }

  public double getRPM(){
    return (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity())/2;
  }

  public void setShooter(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(-rightSpeed);
  }



  public void takeIn(){
    intakeMotor.set(-.55);
  }

  public void spitOut(){
    intakeMotor.set(.6);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void index(){
    intakeMotor.set(-.3);
  }

  public boolean isin(){ //false when note is out
    return !beamBrake.isPressed();
  }

  public void slowOut(){
    intakeMotor.set(.3);
  }

  public double getSpeedRight(){
    return rightMotor.getEncoder().getVelocity();


  }

  public double getSpeedLeft(){
    return leftMotor.getEncoder().getVelocity();
  }

  public double getIntakeCurrent(){
    return intakeMotor.getOutputCurrent();
  }




  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Motor actual velocity", leftMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Right Motor actual Velocity", rightMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Left motor set power", leftMotor.get());
    // SmartDashboard.putNumber("Right motor set power", rightMotor.get());
    // //SmartDashboard.putData("Speed PID", speedController);
    // SmartDashboard.putBoolean("Beam Break?", isin());
    // SmartDashboard.putNumber("left speed", getSpeedLeft());
    // SmartDashboard.putNumber("Right speed", getSpeedRight());
    // SmartDashboard.putNumber("RPM", getRPM());
  }

  


}