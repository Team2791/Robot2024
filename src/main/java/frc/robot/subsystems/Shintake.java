// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
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

  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;
  public CANSparkMax intakeMotor;
  private SparkLimitSwitch beamBrake;
  public PIDController speedController;
  double power;
  Timer timer = new Timer();
  /** Creates a new Shooter. */
  public Shintake() {
    shooterLeader = new CANSparkMax(21, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(22, MotorType.kBrushless);
    shooterLeader.setIdleMode(IdleMode.kCoast);
    shooterFollower.setIdleMode(IdleMode.kCoast);
    shooterFollower.follow(shooterLeader, true);

    shooterLeader.setSmartCurrentLimit(40);
    shooterFollower.setSmartCurrentLimit(40);

    beamBrake = shooterFollower.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);




    intakeMotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);

    

    
    }

  public void setShooter(final double output){
    shooterLeader.set(output);
  }

  public void setShooterPID(final double setpoint){
    if (setpoint == 0) {
        setShooter(setpoint);
    } else {
        shooterLeader.getPIDController().setReference(setpoint, ControlType.kVelocity);
    }
  }

  public double getRPM(){
    return shooterLeader.getEncoder().getVelocity();
  }


    public void takeIn(){
      intakeMotor.set(-.6);
    }

    public void spitOut(){
      intakeMotor.set(.6);
    }

    public void stopIntake(){
      intakeMotor.set(0);
    }


  public boolean isin(){ //false when note is out
    return !beamBrake.isPressed();
  }

  public void slowOut(){
    intakeMotor.set(.3);
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
    SmartDashboard.putNumber("RPM", getRPM());
  }

  


}