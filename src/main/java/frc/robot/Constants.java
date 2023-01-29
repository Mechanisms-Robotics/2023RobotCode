// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Constants {
	public static class OperatorConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
	}

	public static final double loopTime = TimedRobot.kDefaultPeriod;

	public static final Transform2d fieldRobot =
			new Transform2d(
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
					new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
}
