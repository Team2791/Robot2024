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
 * It's not a mess now thank angad for his valiant effort
 */
public final class Constants {
	public static final class Circle {
		public static final double Tau = Math.PI * 2;
	}

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
			public static final double AngularOffsetRearLeft = Math.PI;

			/** Radians */
			public static final double AngularOffsetRearRight = Math.PI / 2;

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
			public static final double MaxAngularSpeed = Circle.Tau;
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
		public static final class Drive {
			public static final int FrontLeft = 10;
			public static final int RearLeft = 30;
			public static final int FrontRight = 20;
			public static final int RearRight = 40;
		}

		public static final class Turn {
			public static final int FrontLeft = 15;
			public static final int RearLeft = 35;
			public static final int FrontRight = 25;
			public static final int RearRight = 45;
		}

		public static final class Climb {
			public static final int Left = 50;
			public static final int Right = 60;
		}

		public static final class Shoot {
			public static final int Left = 70;
			public static final int Right = 80;
		}

		public static final int Turret = 90;
		public static final int Intake = 100;
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
			public static final double TurningPositionFactor = Circle.Tau;

			/** Radians per second */
			public static final double TurningVelocityFactor = Circle.Tau / 60.0;

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

	public static final class Controller {
		public static final int Port = 0;
		public static final double Deadband = 0.05;
	}

	public static class Vision {
		public static final Transform3d CameraToRobot = new Transform3d(
		    new Translation3d(-0.3425, 0.0, -0.233),
		    new Rotation3d()
		);

		public static final Transform3d RobotToCamera = CameraToRobot.inverse();

		/** Meters */
		public static final double CameraHeight = 0.1;

		/** Meters */
		public static final double TargetHeight = 3;

		/** Radians */
		public static final double CameraPitch = 0;
	}

	public static final class Auto {
		public static final class PID {
			public static final class Rotation {
				public static final double P = 0.0000001;
				public static final double I = 0;
				public static final double D = 0;
			}

			public static final class Translation {
				public static final double P = 3.35;
				public static final double I = .8;
				public static final double D = .1;
			}
		}

		public static final class Limits {
			/** Meters per second */
			public static final double MaxSpeed = 3;

			/** Meters per second squared */
			public static final double MaxAcceleration = 3;

			/** Radians per second */
			public static final double MaxAngularSpeed = Math.PI;

			/** Radians per second squared */
			public static final double MaxAngularAcceleration = Math.PI;
		}

		public static final class Profile {
			public static final double XController = 1;
			public static final double YController = 1;
			public static final double ThetaController = 1;
		}

		public static final TrapezoidProfile.Constraints ThetaControllerConstraints = new TrapezoidProfile.Constraints(
		    Limits.MaxAngularSpeed,
		    Limits.MaxAngularAcceleration
		);
	}

	/** Miscellaneous subsystem constants */
	public static final class Subsystem {
		/** meters */
		public static final double ShooterHeight = 0.5;

		/** amps */
		public static final double ClimberCurrent = 5;
	}

	public static final class Game {
		/** meters */
		public static final double SpeakerHeight = 2;
	}

	public static final class Led {
		public static final int Length = 60;

		public static final int ShootSize = 3;
		public static final int ShootSizeHole = 17;

		public static final int FullRainbowLength = 3 * 256;
		public static final int RainbowStep = FullRainbowLength / Length;

		public static final double RotatingSpeed = 0.2;
		public static final int ShiftSize = 1;
	}
}
