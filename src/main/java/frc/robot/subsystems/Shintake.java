// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
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
    leftMotor = new CANSparkMax(RobotMap.leftShoot, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotMap.rightShoot, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
    speedController = new PIDController(Constants.ShintakeConstants.kShooterP, Constants.ShintakeConstants.kShooterI, Constants.ShintakeConstants.kShooterD);
    speedController.setTolerance(.01);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    beamBrake = intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
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
    while(!beamBrake.isLimitSwitchEnabled()){
      intakeMotor.set(-.5);
    }
    timer.start();
    while(timer.get()<.1){
      intakeMotor.set(.1);
    }
    intakeMotor.set(0);

  }

  public void spitOut(){
    intakeMotor.set(.5);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void index(){
    intakeMotor.set(-.3);
  }

  public boolean inout(){
    return !beamBrake.isLimitSwitchEnabled();
  }



  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Left Motor actual velocity", leftMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Right Motor actual Velocity", rightMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Left motor set power", leftMotor.get());
    // SmartDashboard.putNumber("Right motor set power", rightMotor.get());
    // //SmartDashboard.putData("Speed PID", speedController);
    // SmartDashboard.putBoolean("Beam Break?", isItIn());
  }

  


}