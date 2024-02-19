// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  public CANSparkMax Intakemotor;
  /** Creates a new Intake. */
  public Intake() {
    Intakemotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);

  }


  public void setIntakeAngle(double angle, double speed){
    while(Intakemotor.getAbsoluteEncoder().getPosition()<angle){
    Intakemotor.set(speed);}

    Intakemotor.set(0);
  }

  public void takeIn(){
    Intakemotor.set(.5);
  }

  public void spitOut(){
    Intakemotor.set(-.5);
  }
  
  public void stop(){
    Intakemotor.set(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Angle", Intakemotor.getAbsoluteEncoder().getVelocity());
  }

}
