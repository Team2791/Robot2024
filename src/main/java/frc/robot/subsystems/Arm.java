// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  PhotonCamera camera;

  public CANSparkMax armLeft;
  public CANSparkMax armRight;
  public CANSparkMax extensionMotor;

  public final PIDController leftPID;
  private AnalogPotentiometer armPot;
  private AnalogPotentiometer extenpot;
  //private PIDController pid;
  public PIDController extensionPID;


  public double setExtension;
  public double setpoint;
  
  public double power;

  private double slope, intercept;
  private double extSlope, extIntercept;

  /** Creates a new Turret. */
  public Arm() {
    armLeft = new CANSparkMax(32, MotorType.kBrushless);
    armRight = new CANSparkMax(31, MotorType.kBrushless);
    armRight.follow(armLeft, true);
    
    // armLeft.getPIDController().setOutputRange(-.5, .5);
    // armRight.getPIDController().setOutputRange(-.5, .5);

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

    leftPID = new PIDController(.02,0,0);
    leftPID.setTolerance(1);

    setExtension = getExtensionPot();
    extensionPID = new PIDController(.7,0,0);

    setpoint = getArmPot();

  }


  public void setAngle(double angle){
    armLeft.setIdleMode(IdleMode.kBrake);
    power = leftPID.calculate(getArmPot(), angle);

    if(power>.5)power=.5;
    else if(power<-.5)power=-.5;

    armLeft.set(power);
  }

  public void moveUp(double speed){
    armLeft.set(-speed);
    //setpoint = getArmPot();
  }

  public void moveDown(double speed){
    armLeft.set(speed);
    //setpoint = getArmPot();

  }

  public void hold(){
    armLeft.set(0);
    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
  }


  public double getArmPot(){
    return armPot.get();
    //return armLeft.getEncoder().getPosition();
  }

  public void manualExtend(){
    extensionMotor.set(ArmConstants.kExtensionSpeed);
  }

  public void manualRetract(){
    extensionMotor.set(-ArmConstants.kRetractionSpeed);
  }

  public double getExtensionPot(){
    return extenpot.get();
  }

  public void stopExtension(){
    extensionMotor.set(0);
  }

  public double getRawPivotPot(){
    return (getArmPot()-intercept)/slope;
  }




  @Override
  public void periodic() {
    double armPot = getArmPot();
    double extPot = getExtensionPot();

    setpoint = getArmPot();
    SmartDashboard.putNumber("Pivot Angle", armPot);
    SmartDashboard.putNumber("Raw pivot pot", getRawPivotPot());
    SmartDashboard.putNumber("Extension", extPot);
    SmartDashboard.putNumber("Raw extension pot", (extPot - extIntercept) / extSlope);
    // if(leftPID.atSetpoint()){
    //   RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1);
    // }
    //SmartDashboard.putNumber("Encoder", armLeft.getEncoder().getPosition());
    
    //armLeft.set(leftPID.calculate(getArmPot(),setpoint)+Constants.ArmConstants.armLeftFF * Math.cos(Math.toRadians(getArmPot())));

  }


}