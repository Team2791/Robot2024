// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotMap;


// public class Arm extends SubsystemBase {
//   PhotonCamera camera;

//   private CANSparkMax armLeft;
//   private CANSparkMax armRight;
//   private CANSparkMax extensionMotor;

//   // private final PIDController leftPID;
//   // private final PIDController rightPID;
//   private AnalogPotentiometer armLeftPot;
//   private AnalogPotentiometer armRightPot;
//   private AnalogPotentiometer extensionPot;
//   private PIDController pid;

//   private double Fg;
//   public double setAngle;
//   /** Creates a new Turret. */
//   public Arm() {
//     armLeft = new CANSparkMax(RobotMap.armLeft, MotorType.kBrushless);
//     armRight = new CANSparkMax(RobotMap.armRight, MotorType.kBrushless);
//     extensionMotor = new CANSparkMax(RobotMap.extension, MotorType.kBrushless);

//     armLeftPot = new AnalogPotentiometer(Constants.ArmConstants.LeftArmPot,90,244);
//     armRightPot = new AnalogPotentiometer(Constants.ArmConstants.RightArmPot, 90,244);

//     //leftPID = new PIDController(Constants.ArmConstants.armLP, Constants.ArmConstants.armLI, Constants.ArmConstants.armLD);
//     //rightPID = new PIDController(Constants.ArmConstants.armRP, Constants.ArmConstants.armRI, Constants.ArmConstants.armRD);
//     setAngle = (armLeftPot.get()+armRightPot.get())/2;

//     pid = new PIDController(0,0,0);

//     armLeft.setIdleMode(IdleMode.kBrake);
//     armRight.setIdleMode(IdleMode.kBrake);

//   }


//   public void setAngle(double angle){
//     armLeft.setIdleMode(IdleMode.kCoast);
//     armRight.setIdleMode(IdleMode.kCoast);
//     setAngle = angle;
//     while(!pid.atSetpoint()){
//       Fg = Math.cos(angle);
//       armLeft.set(pid.calculate(armLeftPot.get(),angle));
//       armRight.set(pid.calculate(armRightPot.get(),angle));
//     }

//   }

//   public void moveUp(){
//     armLeft.setIdleMode(IdleMode.kBrake);
//     armRight.setIdleMode(IdleMode.kBrake);
//     armLeft.set(.01);
//     armRight.set(.01);

//     setAngle = (armLeftPot.get()+armRightPot.get())/2;
//   }

//   public void moveDown(){
//     armLeft.setIdleMode(IdleMode.kCoast);
//     armRight.setIdleMode(IdleMode.kCoast);
//     armLeft.set(-.01);
//     armRight.set(-.01);
//     setAngle = (armLeftPot.get() + armRightPot.get())/2;
//   }

//   public void hold(){
//     armLeft.set(0);
//     armRight.set(0);
//     armLeft.setIdleMode(IdleMode.kBrake);
//     armRight.setIdleMode(IdleMode.kBrake);
//   }


//   public double getAngle(){
//     return (armLeftPot.get()+armRightPot.get())/2;
//   }

//   public void manualExtend(){
//     extensionMotor.set(.1);
//   }

//   public void manualRetract(){
//     extensionMotor.set(-.1);
//   }

//   public double getExtensionAngle(){
//     return extensionPot.get();
//   }



//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Turret Angle", getAngle());
//     SmartDashboard.putNumber("Extension Angle", getExtensionAngle());
    
//     //SmartDashboard.putData("Left Turret PID", leftPID);
//     //SmartDashboard.putData("Right Turret PID", rightPID);


//     armLeft.set(leftPID.calculate(turretpot.get(),setAngle)+Fg*Constants.ArmConstants.armLFF);
//     armRight.set(rightPID.calculate(turretpot.get(),setAngle)+Fg*Constants.ArmConstants.armRFF);

//     // This method will be called once per scheduler run
//   }


// }