// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {

  CANSparkMax turningMotor;
  public static DriveSubsystem drivetrain;
  /** Creates a new Turret. */
  public Turret() {
    turningMotor = new CANSparkMax(RobotMap.turretMotor, MotorType.kBrushless);
  }

  public void setAngle(double angle){
    while(turningMotor.getAbsoluteEncoder().getPosition()!=angle+10 || turningMotor.getAbsoluteEncoder().getPosition()!=angle-10){
      if(turningMotor.getAbsoluteEncoder().getPosition()<angle)turningMotor.set(.1);
      else turningMotor.set(-.1);
    }
  }


  public double getAngle(){
    return turningMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle", getAngle());
    // This method will be called once per scheduler run
  }


}
