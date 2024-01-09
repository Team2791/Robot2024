// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds

		public static final class Driving {
			/**
			 * The max speed that the robot can drive in meters per second.
			 */
			public static final double kMaxSpeed = 3;

			/**
			 * The max angular speed in radians per second.
			 */
			public static final double kMaxAngularSpeed = 2 * Math.PI;


			// @formatter:off
			static final Translation2d[] wheels = {
				new Translation2d(Chassis.kWheelBase / 2, Chassis.kTrackWidth / 2), 
				new Translation2d(Chassis.kWheelBase / 2, -Chassis.kTrackWidth / 2),
				new Translation2d(-Chassis.kWheelBase / 2, Chassis.kTrackWidth / 2), 
				new Translation2d(-Chassis.kWheelBase / 2, -Chassis.kTrackWidth / 2),
			};
			// @formatter:on

			public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(wheels);
		}

		public static final class Slew {
			/**
			 * Radians per second
			 */
			public static final double kDirection = 1.2;

			/**
			 * Percent per second (1 = 100%)
			 */
			public static final double kMagnitude = 1.8;

			/**
			 * Percent per second (1 = 100%)
			 */
			public static final double kRotational = 2.0;
		}

		public static final class Chassis {
			/**
			 * meters
			 */
			public static final double kTrackWidth = Units.inchesToMeters(21.5);

			/**
			 * meters
			 */
			public static final double kWheelBase = Units.inchesToMeters(21.5);

			public static final double kNwAngularOffset = -Math.PI / 2;
			public static final double kNeAngularOffset = 0;
			public static final double kSwAngularOffset = Math.PI;
			public static final double kSeAngularOffset = Math.PI / 2;
		}

		public static final class CanIds {
			public static final int kNwDrive = 10;
			public static final int kNeDrive = 20;
			public static final int kSwDrive = 30;
			public static final int kSeDrive = 40;

			public static final int kNwTurn = 15;
			public static final int kNeTurn = 25;
			public static final int kSwTurn = 35;
			public static final int kSeTurn = 45;
		}

		public static final boolean kGyroReversed = false;
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth
		// will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

		public static final double kDrivingP = .5; // .04
		public static final double kDrivingI = 0.0001;
		public static final double kDrivingD = 0.01;
		public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 0.2; // 1
		public static final double kTurningI = 0.0001;
		public static final double kTurningD = .01;
		public static final double kTurningFF = 0;
		public static final double kTurningMinOutput = -1;
		public static final double kTurningMaxOutput = 1;

		public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
		public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

		public static final int kDrivingMotorCurrentLimit = 40; // amps
		public static final int kTurningMotorCurrentLimit = 15; // amps
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final double kDriveDeadband = 0.05;
	}

	public static final class AutoConstants {
		public static final class Movement {
			/**
			* meters per second
			*/
			public static final double kMaxSpeed = 3;

			/**
			 * meters per second squared
			 */
			public static final double kMaxAccel = 3;

			/**
			 * radians per second
			 */
			public static final double kMaxAngularSpeed = Math.PI;

			/**
			 * radians per second squared
			 */
			public static final double kMaxAngularAccel = Math.PI;
		}

		public static final class Controller {
			public static final double kPX = 1;
			public static final double kPY = 1;
			public static final double kPTheta = 1;

			public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(Movement.kMaxSpeed, Movement.kMaxAccel);
		}
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}
}
