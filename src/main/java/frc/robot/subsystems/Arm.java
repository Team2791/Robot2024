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
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  PhotonCamera camera;

  private CANSparkMax armLeft;
  private CANSparkMax armRight;
  public CANSparkMax extensionMotor;

  private final PIDController leftPID;
  private final PIDController rightPID;
  private AnalogPotentiometer armPot;
  private AnalogPotentiometer extenpot;
  private PIDController pid;
  public PIDController extensionPID;

  private double Fg;
  public double setExtension;

  private double slope, intercept;
  private double extSlope, extIntercept;

  /** Creates a new Turret. */
  public Arm() {
    armLeft = new CANSparkMax(32, MotorType.kBrushless);
    armRight = new CANSparkMax(31, MotorType.kBrushless);
    armRight.follow(armLeft, true);
    extensionMotor = new CANSparkMax(33, MotorType.kBrushless);
    extensionMotor.setIdleMode(IdleMode.kBrake);
    slope = (ArmConstants.kMaxAngle - ArmConstants.kMinAngle) / (ArmConstants.kMaxPot - ArmConstants.kMinPot);
    intercept = ArmConstants.kMinAngle - (ArmConstants.kMinPot * slope);
    armPot = new AnalogPotentiometer(1, slope, intercept);
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
    extSlope = 100 / (ArmConstants.kExtendMaxPot - ArmConstants.kExtendMinPot);
    extIntercept = - ArmConstants.kExtendMinPot * extSlope;
    extenpot = new AnalogPotentiometer(0, extSlope, extIntercept);

    leftPID = new PIDController(1,0,0);
    rightPID = new PIDController(1,0,0);
    setExtension = getExtensionPot();
    extensionPID = new PIDController(7,0,0);

    pid = new PIDController(0,0,0);

  }


  public void setAngle(double angle){
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
    while(!pid.atSetpoint()){
      Fg = Math.cos(angle);
      armLeft.set(pid.calculate(armPot.get(),angle));
      armRight.set(pid.calculate(armPot.get(),angle));
    }

  }

  public void moveUp(){

    armLeft.set(-ArmConstants.kArmSpeedUp);
  }

  public void moveDown(){

    armLeft.set(ArmConstants.kArmSpeedDown);

  }

  public void hold(){
    armLeft.set(0);
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
  }


  public double getArmPot(){
    return armPot.get();
  }

  public void manualExtend(){
    extensionMotor.set(.1);
    setExtension = getExtensionPot();
  }

  public void manualRetract(){
    extensionMotor.set(-.1);
    setExtension = getExtensionPot();
  }

  public void intakeOut(){
    extensionMotor.set(-.1);
    if(extenpot.get()<Constants.ArmConstants.extensionPosition);
  }

  public void intakein(){
    extensionMotor.set(.1);
  }

  public double getExtensionPot(){
    return extenpot.get();
  }

  public void stopExtension(){
    extensionMotor.set(0);
  }



  @Override
  public void periodic() {
    double armPot = getArmPot();
    double extPot = getExtensionPot();
    SmartDashboard.putNumber("Pivot Angle", armPot);
    SmartDashboard.putNumber("Raw pivot pot", (armPot - intercept) / slope);
    SmartDashboard.putNumber("Extension", extPot);
    SmartDashboard.putNumber("Raw extension pot", (extPot - extIntercept) / extSlope);

    


    //SmartDashboard.putData("Left Turret PID", leftPID);
    //SmartDashboard.putData("Right Turret PID", rightPID);


    // armLeft.set(leftPID.calculate(armPot.get(),setAngle)+Fg*Constants.ArmConstants.armLFF);
    // armRight.set(rightPID.calculate(turretpot.get(),setAngle)+Fg*Constants.ArmConstants.armRFF);

    // This method will be called once per scheduler run
  }


}