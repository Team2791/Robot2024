// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCommands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;

// public class ClimberActivate extends Command {
//   public static boolean isActive = false;

//   Timer timer = new Timer();
//   /** Creates a new ClimberOut. */
//   public ClimberActivate() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {


//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer.start();
//     while(timer.get()<10){
//     Robot.climber.climb(.3);}
//     isActive = true;

//     Robot.climber.climb(0);
//     Robot.climber.setBrakeMode();


//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isActive;
//   }
// }
