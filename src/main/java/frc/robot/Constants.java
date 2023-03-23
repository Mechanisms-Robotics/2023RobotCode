// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Constants {
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int SECOND_DRIVER_CONTROLLER_PORT = 1;

	public static final double LOOP_TIME = TimedRobot.kDefaultPeriod;

	public static final String CAMERA_NAME = "HD_Webcam_C615";

	// Used to correct any weird odometry rotations
	public static final Transform2d FIELD_ROBOT =
			new Transform2d(
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));

	// TODO: Measure height of limelight
	public static final Transform3d ROBOT_TO_CAMERA =
			new Transform3d(
					new Translation3d(1.125, 0.0, 0.7112),
					new Rotation3d(0.0, 0.0, Math.toRadians(180.0 + 5.0)));

	public static final Transform2d SCORING_OFFSET =
			new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d());
	public static final Pose2d PICKUP_POSE =
			new Pose2d(new Translation2d(13.5, 7.0), Rotation2d.fromDegrees(45.0));

	public static final double SWERVE_X_KP = RobotBase.isReal() ? 0.02 : 10.0;
	public static final double SWERVE_X_KI = 0.0;
	public static final double SWERVE_X_KD = 0.0;

	public static final double SWERVE_Y_KP = SWERVE_X_KP;
	public static final double SWERVE_Y_KI = 0.0;
	public static final double SWERVE_Y_KD = 0.0;

	public static final double SWERVE_ROT_KP = RobotBase.isReal() ? 1.75 : 2.5;
	public static final double SWERVE_ROT_KI = 0.0;
	public static final double SWERVE_ROT_KD = 0.0;

	public static final class Auto {
		public static final PIDConstants kTranslationPID = new PIDConstants(SWERVE_X_KP, 0.0, 0.0);
		public static final PIDConstants kRotationPID = new PIDConstants(SWERVE_ROT_KP, 0.0, 0.0);
	}

	public static final boolean SWERVE_DISABLED = false;
}
