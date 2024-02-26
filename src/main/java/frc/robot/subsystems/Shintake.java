// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shintake extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private DigitalInput beamBrake;
  public PIDController speedController;
  double power;
  /** Creates a new Shooter. */
  public Shintake() {
    leftMotor = new CANSparkMax(RobotMap.leftShitakeMotor, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotMap.rightShitakeeMotor, MotorType.kBrushless);
    speedController = new PIDController(Constants.ShintakeConstants.kShooterP, Constants.ShintakeConstants.kShooterI, Constants.ShintakeConstants.kShooterD);
    speedController.setTolerance(.01);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    beamBrake = new DigitalInput(RobotMap.beamBrakeChannel);
  }

  public void setShooter(double speed){
    leftMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),speed));
    rightMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),speed));
  }

  public void setShooter(double leftSpeed, double rightSpeed){
    leftMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),leftSpeed));
    rightMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),rightSpeed));
  }

  public void takeIn(){
    while(!beamBrake.get()){
      leftMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),Constants.ShintakeConstants.intakeSpeed));
      rightMotor.set(speedController.calculate(leftMotor.getEncoder().getVelocity(),Constants.ShintakeConstants.intakeSpeed));
    }
    stop();

  }

  public void spitOut(){
    leftMotor.set(Constants.ShintakeConstants.outtakeSpeed);
    rightMotor.set(Constants.ShintakeConstants.outtakeSpeed);
  }

  public void stop(){
    leftMotor.set(0);
    rightMotor.set(0);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public boolean isItIn(){
    return beamBrake.get();
  }

  public void index(){
    while(!isItIn()){
    leftMotor.set(.1);
    rightMotor.set(.1);
    }
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Motor actual velocity", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Motor actual Velocity", rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left motor set power", leftMotor.get());
    SmartDashboard.putNumber("Right motor set power", rightMotor.get());
    //SmartDashboard.putData("Speed PID", speedController);
    SmartDashboard.putBoolean("Beam Break?", isItIn());
  }

  


}
