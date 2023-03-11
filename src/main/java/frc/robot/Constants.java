// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int SECOND_DRIVER_CONTROLLER_PORT = 1;

	public static final double LOOP_TIME = TimedRobot.kDefaultPeriod;

	public static final String CAMERA_NAME = "USB_Camera";

	// Used to correct any weird odometry rotations
	public static final Transform2d FIELD_ROBOT =
			new Transform2d(
					new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)),
					new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)));

	// TODO: Measure height of limelight
	public static final Transform3d ROBOT_TO_CAMERA =
			new Transform3d(new Translation3d(0.0, -0.088, 1.067), new Rotation3d(new Quaternion()));

	public static final Transform2d SCORING_OFFSET =
			new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d());
	public static final Pose2d PICKUP_POSE =
			new Pose2d(new Translation2d(13.5, 7.0), Rotation2d.fromDegrees(45.0));

	public static final double SWERVE_X_KP = 10.0;
	public static final double SWERVE_X_KI = 0.0;
	public static final double SWERVE_X_KD = 0.0;

	public static final double SWERVE_Y_KP = 10.0;
	public static final double SWERVE_Y_KI = 0.0;
	public static final double SWERVE_Y_KD = 0.0;

	public static final double SWERVE_ROT_KP = 5.0;
	public static final double SWERVE_ROT_KI = 0.0;
	public static final double SWERVE_ROT_KD = 0.0;

	public static final boolean SWERVE_DISABLED = false;

	public static class Swerve {
		public static final int pigeonID = 0;
		public static final boolean invertGyro = false;

		public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
				COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(18.25); //TODO: This must be tuned to specific robot
		public static final double wheelBase = Units.inchesToMeters(30.25); //TODO: This must be tuned to specific robot
		public static final double wheelCircumference = chosenModule.wheelCircumference;

		public static final Translation2d[] swerveModuleTranslations = {
				new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
		};

		/* Swerve Kinematics
		 * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(swerveModuleTranslations);

		/* Module Gear Ratios */
		public static final double driveGearRatio = chosenModule.driveGearRatio;
		public static final double angleGearRatio = chosenModule.angleGearRatio;

		/* Motor Inverts */
		public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
		public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

		/* Angle Encoder Invert */
		public static final boolean canCoderInvert = chosenModule.canCoderInvert;

		/* Swerve Current Limiting */
		public static final int angleContinuousCurrentLimit = 25;
		public static final int anglePeakCurrentLimit = 40;
		public static final double anglePeakCurrentDuration = 0.1;
		public static final boolean angleEnableCurrentLimit = true;

		public static final int driveContinuousCurrentLimit = 35;
		public static final int drivePeakCurrentLimit = 60;
		public static final double drivePeakCurrentDuration = 0.1;
		public static final boolean driveEnableCurrentLimit = true;

		/* These values are used by the drive falcon to ramp in open loop and closed loop driving.
		 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		/* Angle Motor PID Values */
		public static final double angleKP = chosenModule.angleKP;
		public static final double angleKI = chosenModule.angleKI;
		public static final double angleKD = chosenModule.angleKD;
		public static final double angleKF = chosenModule.angleKF;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.0;
		public static final double driveKF = 0.0;

		/* Drive Motor Characterization Values
		 * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
		public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
		public static final double driveKV = (1.51 / 12);
		public static final double driveKA = (0.27 / 12);

		/* Swerve Profiling Values */
		/** Meters per Second */
		public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
		/** Radians per Second */
		public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

		/* Neutral Modes */
		public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
		public static final NeutralMode driveNeutralMode = NeutralMode.Coast;

		/* Module Specific Constants */
		/* Front Left Module - Module 0 */
		public static final class FrontLeftMod { //TODO: This must be tuned to specific robot
			public static final int driveMotorID = 13;
			public static final int angleMotorID = 12;
			public static final int canCoderID = 12;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(99.49 - 180.0);
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/* Front Right Module - Module 1 */
		public static final class FrontRightMod { //TODO: This must be tuned to specific robot
			public static final int driveMotorID = 11;
			public static final int angleMotorID = 10;
			public static final int canCoderID = 10;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.29);
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/* Back Left Module - Module 2 */
		public static final class BackLeftMod { //TODO: This must be tuned to specific robot
			public static final int driveMotorID = 17;
			public static final int angleMotorID = 16;
			public static final int canCoderID = 16;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(192.04);
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/* Back Right Module - Module 3 */
		public static final class BackRightMod { //TODO: This must be tuned to specific robot
			public static final int driveMotorID = 15;
			public static final int angleMotorID = 14;
			public static final int canCoderID = 14;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.67);
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}
	}

	public static final class Auto {
		public static final PIDConstants kTranslationPID = new PIDConstants(0.0, 0.0, 0.0);
		public static final PIDConstants kRotationPID = new PIDConstants(0.0, 0.0, 0.0);
	}
}
