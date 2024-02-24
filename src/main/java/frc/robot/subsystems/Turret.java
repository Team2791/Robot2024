// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Turret extends SubsystemBase {
  PhotonCamera camera;

  private CANSparkMax turningMotor;
  private AnalogPotentiometer turretpot;
  /** Creates a new Turret. */
  public Turret() {
    turningMotor = new CANSparkMax(RobotMap.turretMotor, MotorType.kBrushless);
    turretpot = new AnalogPotentiometer(Constants.RobotConstants.turretPot,0,244);
    

  }

  public void setAngle(double angle){
    while(turretpot.get()!=angle+10 || turretpot.get()!=angle-10){
      if(turretpot.get()<angle)turningMotor.set(.1);
      else turningMotor.set(-.1);
    }
  }


  public double getAngle(){
    return turretpot.get();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle", getAngle());
    // This method will be called once per scheduler run
  }


}
