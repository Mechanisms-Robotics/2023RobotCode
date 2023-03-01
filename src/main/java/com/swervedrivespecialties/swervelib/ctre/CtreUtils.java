package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class CtreUtils {
	private CtreUtils() {}

	public static void checkCtreError(ErrorCode errorCode, String message) {
		if (RobotBase.isReal() && errorCode != ErrorCode.OK) {
			DriverStation.reportError(
					String.format("%s: %s", message, errorCode.toString()), false);

			SmartDashboard.putString(
					"pigeon error", String.format("%s: %s", message, errorCode.toString()));
		}
	}
}
