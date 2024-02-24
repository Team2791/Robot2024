// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Turret extends SubsystemBase {
  PhotonCamera camera;

  private CANSparkMax turretLeft;
  private CANSparkMax turretRight;

  private final PIDController leftPID;
  private final PIDController rightPID;
  private AnalogPotentiometer turretpot;
  private double Fg;
  public double setAngle;
  /** Creates a new Turret. */
  public Turret() {
    turretLeft = new CANSparkMax(RobotMap.turretLeft, MotorType.kBrushless);
    turretRight = new CANSparkMax(RobotMap.turretRight, MotorType.kBrushless);
    turretpot = new AnalogPotentiometer(Constants.RobotConstants.turretPot,90,244);

    leftPID = new PIDController(Constants.RobotConstants.turretLeftP, Constants.RobotConstants.turretLeftI, Constants.RobotConstants.turretLeftD);
    rightPID = new PIDController(Constants.RobotConstants.turretRightP, Constants.RobotConstants.turretRightI, Constants.RobotConstants.turretRightD);
    setAngle = turretpot.get();

    turretLeft.setIdleMode(IdleMode.kBrake);
    turretRight.setIdleMode(IdleMode.kBrake);

  }


  public void setAngle(double angle){
    turretLeft.setIdleMode(IdleMode.kCoast);
    turretRight.setIdleMode(IdleMode.kCoast);
    setAngle = angle;
    while(!leftPID.atSetpoint() && !rightPID.atSetpoint()){
      Fg = Math.cos(angle);
      turretLeft.set(leftPID.calculate(turretpot.get(),angle));
      turretRight.set(rightPID.calculate(turretpot.get(),angle));
    }

  }

  public void moveUp(){
    turretLeft.setIdleMode(IdleMode.kCoast);
    turretRight.setIdleMode(IdleMode.kCoast);
    turretLeft.set(.01);
    turretRight.set(.01);

    setAngle = turretpot.get();
  }

  public void moveDown(){
    turretLeft.setIdleMode(IdleMode.kCoast);
    turretRight.setIdleMode(IdleMode.kCoast);
    turretLeft.set(-.01);
    turretRight.set(-.01);
    setAngle = turretpot.get();
  }

  public void hold(){
    turretLeft.set(0);
    turretRight.set(0);
    turretLeft.setIdleMode(IdleMode.kBrake);
    turretRight.setIdleMode(IdleMode.kBrake);
  }


  public double getAngle(){
    return turretpot.get();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle", getAngle());
    SmartDashboard.putData("Left Turret PID", leftPID);
    SmartDashboard.putData("Right Turret PID", rightPID);

    turretLeft.set(leftPID.calculate(turretpot.get(),setAngle)+Fg*Constants.RobotConstants.turretLeftFF);
    turretRight.set(rightPID.calculate(turretpot.get(),setAngle)+Fg*Constants.RobotConstants.turretRightFF);

    // This method will be called once per scheduler run
  }


}
