// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;

public class PhotonAim extends CommandBase {
	PhotonCamera camera, camera2;
	PIDController pid;
	Command thing;
	double output;
	double pval;
	int ci;
	/** Creates a new PhotonAim. */
	public PhotonAim(PhotonCamera c,double p) {
		pval = p;
		pid = new PIDController(0.04, 0, 0); //.6
		camera = c;
		
		addRequirements(Robot.drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		pid.setTolerance(1);
		pid.setSetpoint(-5);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		var result = camera.getLatestResult();
		if (result != null && result.hasTargets()) {
			output = pid.calculate(result.getBestTarget().getYaw()); // -8
			if (output > 0.4)
				output = 0.4;
			if (output < -0.4)
				output = -0.4;

			Robot.drivetrain.arcadeDrive(0, output);
		
		}

		if (result == null) {
			output = 0;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.setMotors(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pid.atSetpoint();
	}
}
