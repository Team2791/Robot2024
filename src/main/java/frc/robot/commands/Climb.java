// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.RGBLED;

public class Climb extends Command {
	private boolean done = false;

	private final Climber climber;
	private final AHRS gyro;
	private final RGBLED led;

	private final double multPosRoll;
	private final double multNegRoll;

	/**
	 * Creates a new Climb.
	 */
	public Climb(Climber climber, AHRS gyro, RGBLED led, boolean up) {
		this.climber = climber;
		this.gyro = gyro;
		this.led = led;

		if (up) {
			this.multNegRoll = 0.1;
			this.multPosRoll = 0.5;
		} else {
			this.multNegRoll = -0.5;
			this.multPosRoll = -0.1;
		}
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		this.climber.climb(this.multPosRoll);

		double roll = this.gyro.getRoll();
		boolean enoughAmpsLeft = this.climber.leftAmps() > Constants.Subsystem.ClimberCurrent;
		boolean enoughAmpsRight = this.climber.rightAmps() > Constants.Subsystem.ClimberCurrent;

		if (enoughAmpsLeft && enoughAmpsRight) {
			this.led.setColor(255, 0, 0);

			switch ((int) Math.signum(roll)) {
				case 1:
					this.climber.climb(roll * this.multNegRoll, roll * this.multPosRoll);
					break;
				case -1:
					this.climber.climb(roll * this.multPosRoll, roll * this.multNegRoll);
					break;
				default:
					this.climber.climb(roll * this.multNegRoll);
					break;
			}
		}

		// TODO: figure out how to set done
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.climber.setMode(IdleMode.kBrake);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
