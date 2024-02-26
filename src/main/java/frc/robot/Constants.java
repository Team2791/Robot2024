// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8;
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

		public static final double kDirectionSlewRate = 1.2; // radians per second
		public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
		public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

		// Chassis configuration (meters)
		public static final double kTrackWidth = Units.inchesToMeters(21.5); // comp bot 24.5
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(21.5); //comp bot 24.5
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		// SPARK MAX CAN IDs (DONE)
		public static final int kFrontLeftDrivingCanId = 10;
		public static final int kRearLeftDrivingCanId = 30;
		public static final int kFrontRightDrivingCanId = 20;
		public static final int kRearRightDrivingCanId = 40;

		public static final int kFrontLeftTurningCanId = 15;
		public static final int kRearLeftTurningCanId = 35;
		public static final int kFrontRightTurningCanId = 25;
		public static final int kRearRightTurningCanId = 45;

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
		public static final double kWheelDiameterMeters = 0.0762; //0.08636 on new bot
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)/ kDrivingMotorReduction;

		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction; // meters
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction) / 60.0; // meters per second

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

		public static final double kDrivingP = .15; // .04
		public static final double kDrivingI = 0.0001;
		public static final double kDrivingD = 0.01;
		public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 4; // 1
		public static final double kTurningI = 0.0001;
		public static final double kTurningD = .02;
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
		public static final int kOperatorControllerPort = 1;
		public static final double kDriveDeadband = 0.05;
	}
	public static class VisionConstants {
		/** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
		public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(-0.06, 0.2, -0.2127),new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(-3.0)));
		
		public static final double FIELD_LENGTH_METERS = 16.54;
		public static final double FIELD_WIDTH_METERS = 8.21;
	
		// Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
		public static final Pose2d FLIPPING_POSE = new Pose2d(new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),new Rotation2d(Math.PI));
	
		/** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
		public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

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
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

	public static final class ClimberConstants{
		public static final double climbVoltage = 5;
		public static final double climbLocked = 90;
		public static final int leftClimbPot = 3;
		public static final int rightClimbPot = 4;
	}

		public static final class ArmConstants{

			public static final int armPot = 6;

			public static final double armLP = 1;
			public static final double armLI = 0;
			public static final double armLD = 0;
			public static final double armLFF = 0;

			public static final double armRP = 0;
			public static final double armRI = 0;
			public static final double armRD = 0;
			public static final double armRFF = 1;

			
			public static final double SpeakerHeight = 2.045;
			public static final double ShooterHeight = .5;

		}

		public static final class ShintakeConstants{


			public static final double kShooterP = 1;
			public static final double kShooterI = 1;
			public static final double kShooterD = 1;

			public static final double speakerSpeed =25;
			public static final double ampSpeed = 10;
			public static final double intakeSpeed = 1;
			public static final double outtakeSpeed = -1;
		}



		public static final class AprilTagCommandsConstants{

			public static final double CAMERA_HEIGHT_METERS = .01;
			public static final double CAMERA_PITCH_RADIANS = 0;

			public static final double kATrotateP = .001;
			public static final double kATrotateI = .0001;
			public static final double kATrotateD = .00004;


			public static final double kATtranslateP = .001;
			public static final double kATtranslateI = .0001;
			public static final double kATtranslateD = .00004;
		}
}


		
