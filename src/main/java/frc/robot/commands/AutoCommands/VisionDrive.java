// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;

public class VisionDrive extends CommandBase {
	PhotonCamera camera, camera2;
	PIDController pid;
	Command thing;
	double output;
	Timer timer = new Timer();
	double set, goal;
	boolean tag;
	private TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints(120, 25);
	private ProfiledPIDController drivePID = new ProfiledPIDController(0.42, 0, 0, constraint);

	/** Creates a new PhotonAim. */
	public VisionDrive(PhotonCamera c, PhotonCamera c2, double setpoint, boolean tag, double dis) {
		pid = new PIDController(0.04, 0, 0);
		camera = c;
		camera2 = c2;
		camera.setPipelineIndex(0);
		set = setpoint;
		this.tag = tag;
		goal = dis;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		pid.setSetpoint(set);
		pid.setTolerance(1);
		timer.start();
		drivePID.setGoal(goal);
		Robot.drivetrain.resetEncoders();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		var result = camera.getLatestResult();
		var result2 = camera2.getLatestResult();

		if (result != null && result.hasTargets()) {
			output = pid.calculate(result.getBestTarget().getYaw()); // -8
			double thrust = pid.calculate(Robot.drivetrain.getAveragePosition());

			if (thrust > 0.7)
				thrust = 0.7;
			if (thrust < -0.7)
				thrust = -0.7;
			if (output > 0.4)
				output = 0.4;
			if (output < -0.4)
				output = -0.4;

			Robot.drivetrain.arcadeDrive(thrust, output);

			Robot.led.setColor(255, 0, 0);

		}

		// Robot.led.setColor(255, 255, 255);

		//

		if (result == null) {
			output = 0;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.led.setColor(0, 255, 0);
		Robot.drivetrain.setMotors(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {

		return pid.atSetpoint() || timer.get() > 1.8;
	}
}
