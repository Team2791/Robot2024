// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCommands;


// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;

// public class ClimbRelease extends Command {
//   public Timer timer;
//   private boolean isFinished;
//   /** Creates a new ClimbRelease. */
//   public ClimbRelease() {

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Robot.climber.unlock();
//     timer = new Timer();
//     Robot.climber.climb(.01);
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     while (timer.get()<5.0){
//       Robot.climber.climb(0);
//       isFinished = true;

//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     isFinished = true;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isFinished;
//   }
// }
