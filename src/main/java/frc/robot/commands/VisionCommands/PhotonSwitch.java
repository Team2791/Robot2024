// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class PhotonSwitch extends CommandBase {
	private int cameraIndex;
	/** Creates a new Switch. */
	PhotonCamera cam;

	public PhotonSwitch(PhotonCamera c) {
		cam = c;
		addRequirements(Robot.drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		cameraIndex = cam.getPipelineIndex();
		if (cameraIndex == 1) {
			Robot.led.setColor(128, 0, 128);
			cameraIndex++;
		} 
		else if(cameraIndex== 2){
			Robot.led.setColor(255, 255, 255);
		}
		else {
			Robot.led.setColor(255, 255, 0);
			cameraIndex++;
		}

		cam.setPipelineIndex(cameraIndex);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return cam.getPipelineIndex() == cameraIndex;
	}
}
