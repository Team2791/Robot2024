// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
// BOTTOM EXTENSION = 0.3
// ZERO PIVOT =

public final class Constants {
	// Driving Constants
	public static final double kCreep = 0.55; // 0.525
	public static final double TURN_FACTOR = -.59;
	public static final double MaxSpeed = .97;

	// Cone Intake Presets
	public static final double ConeIntakePivotFront = 75;
	public static final double ConeIntakePivotBack = -83.5;
	public static final double FlangeIntakePivotFront = 84;
	public static final double FlangeIntakePivotBack = -93;

	// Cone Scoring Presets
	public static final double MidConeScoringPivotFront = 41;
	public static final double MidConeScoringPivotBack = -49;
	public static final double ConeScoringPivotFront = 38;
	public static final double ConeScoringPivotBack = -47;

	// Cube Intake/Scoring Presets
	public static final double CubeIntakePivotFront = 83;
	public static final double CubeIntakePivotBack = -89.5;
	public static final double CubeScoringPivotFront = 43;
	public static final double CubeScoringPivotBack = -52;

	// Substation Presets
	public static final double NewStationPivotFront = 29;
	public static final double NewStationPivotBack = -36.5;

	// Extension Presets
	public static final double NewStationExtension = 0.2;
	public static final double ConeUpperExtension = 29;
	public static final double ConeMidExtension = 13;
	public static final double CubeExtension = 23;

	// PID Constants
	public static final double ExtendKP = 0.32;// .35
	public static final double ExtendKI = 0;
	public static final double ExtendKD = 0.005; // 0.002
	public static final double PivotKP = 0.05;
	public static final double PivotKI = 0;
	public static final double PivotKD = 0.005;

	public static final double IntakeMaxCurrent = 47; // 17 old
	public static final double newPivotOffset = 1;
	public static final double newExtendOffset = 4;

	// Speeds(Pivot/Extension)
	public static final double PivotMaxSpeed = 0.53;
	public static final double ExtendMaxSpeed = 1;
	public static final double IntakeSpeed = 0.65;
	public static final double OutakeSpeed = -0.65;
	public static final double IntakeRotionSpeed = 0.38;
	public static final double ManualPivotSpeed = 0.16;

	// Limits
	public static final double maxPivot = 100;
	public static final double minPivot = -100;
	public static final double minExtension = 0.4;
	public static final double maxExtension = 40;

	// Variables
	public static boolean ManualPivot;
	public static boolean ManualExtend;
	public static boolean mode = true;
	public static boolean orientation;
	public static int Jonah;
	public static boolean Intaking = false;

	// Pathing constants
	public static final double WheelDiameter = Units.inchesToMeters(6);
	public static final double GearRatio = 1.0 / 9.0;
	public static final double TrackWidth = Units.inchesToMeters(24.25);
	public static final double pconversion = WheelDiameter * Math.PI * GearRatio;
	public static final double vconversion = (WheelDiameter * Math.PI * GearRatio) / 60;
	public static double kP = 1.6825;
	public static double ks = 0.09002;
	public static double kv = 2.3225;
	public static double ka = 0.38092;
}
