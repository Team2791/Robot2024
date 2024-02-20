// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class Drive {
		public static final class Dimensions {
			/** Meters */
			public static final double TrackWidth = Units.inchesToMeters(21.5);

			/** Meters */
			public static final double WheelBase = Units.inchesToMeters(21.5);

			/** Radians */
			public static final double AngularOffsetFrontLeft = -Math.PI / 2;

			/** Radians */
			public static final double AngularOffsetFrontRight = 0;

			/** Radians */
			public static final double AngularOffsetBackLeft = Math.PI;

			/** Radians */
			public static final double AngularOffsetBackRight = Math.PI / 2;

			/** Meters */
			public static final double DriveBaseRadius = 0.39;
		}

		public static final class Slew {
			/** Radians per second */
			public static final double Direction = 1.2;

			/** Percent per second (1 = 100%) */
			public static final double Magnitude = 1.8;

			/** Percent per second (1 = 100%) */
			public static final double Rotational = 2.0;
		}

		public static final class Limits {
			/** Meters per second */
			public static final double MaxSpeed = 4.8;

			/** Radians per second */
			public static final double MaxAngularSpeed = 2 * Math.PI;
		}

		public static final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
		    new Translation2d(Dimensions.WheelBase / 2, Dimensions.TrackWidth / 2),
		    new Translation2d(Dimensions.WheelBase / 2, -Dimensions.TrackWidth / 2),
		    new Translation2d(-Dimensions.WheelBase / 2, Dimensions.TrackWidth / 2),
		    new Translation2d(-Dimensions.WheelBase / 2, -Dimensions.TrackWidth / 2)
		);

		public static final boolean GyroReversed = false;
	}

	public static final class Ids {
		public static final class DrivingMotor {
			public static final int FrontLeft = 10;
			public static final int RearLeft = 30;
			public static final int FrontRight = 20;
			public static final int RearRight = 40;
		}

		public static final class TurningMotor {
			public static final int FrontLeft = 15;
			public static final int RearLeft = 35;
			public static final int FrontRight = 25;
			public static final int RearRight = 45;
		}
	}


	public static final class Module {
		public static final class Gear {
			/**
			 * The MAXSwerve module can have either a 12, 13, or 14 tooth pinion gear.
			 * This changes the drive speed of the module (more teeth = faster).
			 */
			public static final int DrivingMotorPinionTeeth = 14;
			public static final int BevelGearTeeth = 45;
			public static final int FirstSpurGearTeeth = 22;
			public static final int BevelPinionTeeth = 15;

			public static final double DriveMotorReduction = ((double) (BevelGearTeeth * FirstSpurGearTeeth)) / ((double) (DrivingMotorPinionTeeth * BevelPinionTeeth));
		}

		public static final class Limits {
			/** Meters per second */
			public static final double DrivingMotorFreeSpeed = 5676.0 / 60.0;

			/** Rotations per second */
			public static final double DrivingWheelFreeSpeed = (Gear.DriveMotorReduction * Constants.Drive.Dimensions.DriveBaseRadius) / DrivingMotorFreeSpeed;

			/** Amps */
			public static final int DrivingMotorCurrent = 40;

			/** Amps */
			public static final int TurningMotorCurrent = 15;
		}

		public static final class Encoder {
			/** Meters */
			public static final double DrivingPositionFactor = (Constants.Drive.Dimensions.DriveBaseRadius * Math.PI) / Gear.DriveMotorReduction;

			/** Meters per second */
			public static final double DrivingVelocityFactor = ((Constants.Drive.Dimensions.DriveBaseRadius * Math.PI) / Gear.DriveMotorReduction) / 60.0;

			/** Radians */
			public static final double TurningPositionFactor = 2 * Math.PI;

			/** Radians per second */
			public static final double TurningVelocityFactor = (2 * Math.PI) / 60.0;

			/** Radians */
			public static final double TurningPositionPIDMin = 0;

			/** Radians */
			public static final double TurningPositionPIDMax = TurningPositionFactor;

			/** Radians */
			public static final boolean TurningInverted = true;
		}

		public static final class PID {
			public static final class Driving {
				public static final double P = 0.15;
				public static final double I = 0.0001;
				public static final double D = 0.01;
				public static final double FF = 1 / Limits.DrivingWheelFreeSpeed;
				public static final double MinOutput = -1;
				public static final double MaxOutput = 1;
			}

			public static final class Turning {
				public static final double P = 4;
				public static final double I = 0.0001;
				public static final double D = 0.02;
				public static final double FF = 0;
				public static final double MinOutput = -1;
				public static final double MaxOutput = 1;
			}
		}

		public static final class Idle {
			public static final IdleMode Driving = IdleMode.kBrake;
			public static final IdleMode Turning = IdleMode.kBrake;
		}
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final double kDriveDeadband = 0.05;
	}

	public static class VisionConstants {
		public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
		    new Translation3d(-0.3425, 0.0, -0.233),
		    new Rotation3d()
		);
		public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
	}

	public static final class AutoConstants {

		public static final double kRotationP = 0.0000001;
		public static final double kRotationI = 0;
		public static final double kRotationD = 0;

		public static final double kTranslationP = 3.35;
		public static final double kTranslationI = .8;
		public static final double kTranslationD = .1;

		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
		    kMaxAngularSpeedRadiansPerSecond,
		    kMaxAngularSpeedRadiansPerSecondSquared
		);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

	public static final class GameConstants {
		public static final double climbVoltage = 5;
		public static final double CAMERA_HEIGHT_METERS = .1;
		public static final double TARGET_HEIGHT_METERS = 3;
		public static final double CAMERA_PITCH_RADIANS = 0;
		public static final double SpeakerHeight = 2;
		public static final double ShooterHeight = .5;
	}
}
