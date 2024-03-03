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
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  PhotonCamera camera;

  private CANSparkMax armLeft;
  private CANSparkMax armRight;
  private CANSparkMax extensionMotor;

  private final PIDController leftPID;
  private final PIDController rightPID;
  private AnalogPotentiometer armPot;
  private AnalogPotentiometer extenpot;
  private PIDController pid;

  private double Fg;
  public double setAngle;
  /** Creates a new Turret. */
  public Arm() {
    armLeft = new CANSparkMax(32, MotorType.kBrushless);
    armRight = new CANSparkMax(31, MotorType.kBrushless);
    armRight.follow(armLeft, true);
    extensionMotor = new CANSparkMax(33, MotorType.kBrushless);

    double slope = (ArmConstants.kMaxAngle - ArmConstants.kMinAngle) / (ArmConstants.kMaxPot - ArmConstants.kMinPot);
    double intercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * slope);
    armPot = new AnalogPotentiometer(1, slope, intercept);

    extenpot = new AnalogPotentiometer(0);

    leftPID = new PIDController(1,0,0);
    rightPID = new PIDController(1,0,0);
    setAngle = (armPot.get()+armPot.get())/2;

    pid = new PIDController(0,0,0);

    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);

  }


  public void setAngle(double angle){
    armLeft.setIdleMode(IdleMode.kCoast);
    armRight.setIdleMode(IdleMode.kCoast);
    setAngle = angle;
    while(!pid.atSetpoint()){
      Fg = Math.cos(angle);
      armLeft.set(pid.calculate(armPot.get(),angle));
      armRight.set(pid.calculate(armPot.get(),angle));
    }

  }

  public void moveUp(){
    armLeft.set(-ArmConstants.kArmSpeedDown);

    setAngle = armPot.get();
  }

  public void moveDown(){
    armLeft.set(ArmConstants.kArmSpeedUp);
    setAngle = armPot.get();
  }

  public void hold(){
    armLeft.set(0);
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    armLeft.setIdleMode(IdleMode.kCoast);
    armRight.setIdleMode(IdleMode.kCoast);
  }


  public double getArmPot(){
    return armPot.get();
  }

  public void manualExtend(){
    extensionMotor.set(.1);
  }

  public void manualRetract(){
    extensionMotor.set(-.1);
  }

  public double getExtensionPot(){
    return extenpot.get();
  }

  public void stopExtension(){
    extensionMotor.set(0);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Pot", getArmPot());
    SmartDashboard.putNumber("Extension Angle", getExtensionPot());
    //SmartDashboard.putData("Left Turret PID", leftPID);
    //SmartDashboard.putData("Right Turret PID", rightPID);


    // armLeft.set(leftPID.calculate(armPot.get(),setAngle)+Fg*Constants.ArmConstants.armLFF);
    // armRight.set(rightPID.calculate(turretpot.get(),setAngle)+Fg*Constants.ArmConstants.armRFF);

    // This method will be called once per scheduler run
  }


}