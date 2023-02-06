// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Constants {
	public static final int DRIVER_CONTROLLER_PORT = 0;

	public static final double LOOP_TIME = TimedRobot.kDefaultPeriod;

	public static final String CAMERA_NAME = "limelight";

	// Used to correct any weird odometry rotations
	public static final Transform2d FIELD_ROBOT =
			new Transform2d(
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));

	// TODO: Measure height of limelight
	public static final Transform3d ROBOT_TO_CAMERA =
			new Transform3d(new Translation3d(0.0, 0.0, 0.8), new Rotation3d(new Quaternion()));

	public static final Transform2d SCORING_OFFSET =
			new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d());

	public static final double SWERVE_X_KP = 10.0;
	public static final double SWERVE_X_KI = 0.0;
	public static final double SWERVE_X_KD = 0.0;

	public static final double SWERVE_Y_KP = 10.0;
	public static final double SWERVE_Y_KI = 0.0;
	public static final double SWERVE_Y_KD = 0.0;

	public static final double SWERVE_ROT_KP = 5.0;
	public static final double SWERVE_ROT_KI = 0.0;
	public static final double SWERVE_ROT_KD = 0.0;
}
