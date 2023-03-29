package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;

public class ObstacleAvoidance {
	private static final double BORDER_1_X = 2.95;
	private static final double BORDER_2_X = 4.875;

	private static final double BORDER_1_Y = 2.75;

	private static final double ACTUAL_X_OFFSET = -0.4;

	private static final Pose2d UPPER_RIGHT_CONTROL_POINT =
			new Pose2d(
					new Translation2d(
							5.50 + ACTUAL_X_OFFSET,
							4.75
									+ ((DriverStation.getAlliance() == Alliance.Red)
											? Constants.RED_Y_OFFSET
											: 0.0)),
					Rotation2d.fromDegrees(180.0));
	private static final Pose2d UPPER_LEFT_CONTROL_POINT =
			new Pose2d(
					new Translation2d(
							2.625 + ACTUAL_X_OFFSET,
							4.75
									+ ((DriverStation.getAlliance() == Alliance.Red)
											? Constants.RED_Y_OFFSET
											: 0.0)),
					Rotation2d.fromDegrees(180.0 + 90.0));

	private static final Pose2d LOWER_RIGHT_CONTROL_POINT =
			new Pose2d(
					new Translation2d(
							5.50 + ACTUAL_X_OFFSET,
							0.75
									+ ((DriverStation.getAlliance() == Alliance.Red)
											? Constants.RED_Y_OFFSET
											: 0.0)),
					Rotation2d.fromDegrees(180.0));
	private static final Pose2d LOWER_LEFT_CONTROL_POINT =
			new Pose2d(
					new Translation2d(
							2.625 + ACTUAL_X_OFFSET,
							0.75
									+ ((DriverStation.getAlliance() == Alliance.Red)
											? Constants.RED_Y_OFFSET
											: 0.0)),
					Rotation2d.fromDegrees(180.0 - 90.0));

	private static final Pose2d HP_CONTROL_POINT =
			new Pose2d(
					new Translation2d(10.5 + ACTUAL_X_OFFSET, 6.0), Rotation2d.fromDegrees(20.0));

	private static final Pose2d[] CONTROL_POINTS = {
		UPPER_LEFT_CONTROL_POINT,
		UPPER_RIGHT_CONTROL_POINT,
		LOWER_RIGHT_CONTROL_POINT,
		LOWER_LEFT_CONTROL_POINT,
		HP_CONTROL_POINT
	};

	public static PathPlannerTrajectory generateTrajectoryAvoidObstacles(
			Pose2d goalPose, double translationVelocity, double rotationVelocity, Swerve swerve) {
		ArrayList<PathPoint> points = new ArrayList<>();
		SmartDashboard.putNumber("Goal X", goalPose.getX());
		SmartDashboard.putNumber("Goal Y", goalPose.getY());

		double curX = swerve.getPose().getX();
		double curY = swerve.getPose().getY();

		double goalX = goalPose.getX();
		double goalY = goalPose.getY();

		if (curY > goalY) {
			points.add(
					new PathPoint(
							swerve.getPose().getTranslation(),
							Rotation2d.fromDegrees(-90.0),
							swerve.getGyroHeading()));
		} else {
			points.add(
					new PathPoint(
							swerve.getPose().getTranslation(),
							Rotation2d.fromDegrees(90.0),
							swerve.getGyroHeading()));
		}

		if (goalX < BORDER_2_X && BORDER_2_X < curX) {
			if (BORDER_1_Y < curY) {
				points.add(
						new PathPoint(
								UPPER_RIGHT_CONTROL_POINT.getTranslation(),
								UPPER_RIGHT_CONTROL_POINT.getRotation(),
								goalPose.getRotation()));
			} else {
				points.add(
						new PathPoint(
								LOWER_RIGHT_CONTROL_POINT.getTranslation(),
								LOWER_RIGHT_CONTROL_POINT.getRotation(),
								goalPose.getRotation()));
			}
		}

		if (goalX < BORDER_1_X && BORDER_1_X < curX) {
			if (BORDER_1_Y < curY) {
				points.add(
						new PathPoint(
								UPPER_LEFT_CONTROL_POINT.getTranslation(),
								UPPER_LEFT_CONTROL_POINT.getRotation(),
								goalPose.getRotation()));
			} else {
				points.add(
						new PathPoint(
								LOWER_LEFT_CONTROL_POINT.getTranslation(),
								LOWER_LEFT_CONTROL_POINT.getRotation(),
								goalPose.getRotation()));
			}
		}

		if (goalX > curX) {
			if (curX > BORDER_1_X) {
				if (curY > BORDER_1_Y) {
					points.add(
							new PathPoint(
									UPPER_LEFT_CONTROL_POINT.getTranslation(),
									UPPER_LEFT_CONTROL_POINT
											.getRotation()
											.rotateBy(Rotation2d.fromDegrees(180.0)),
									new Rotation2d()));
				} else {
					points.add(
							new PathPoint(
									LOWER_LEFT_CONTROL_POINT.getTranslation(),
									LOWER_LEFT_CONTROL_POINT
											.getRotation()
											.rotateBy(Rotation2d.fromDegrees(180.0)),
									new Rotation2d()));
				}
			}

			if (curX > BORDER_2_X) {
				if (curY > BORDER_1_Y) {
					points.add(
							new PathPoint(
									UPPER_RIGHT_CONTROL_POINT.getTranslation(),
									UPPER_RIGHT_CONTROL_POINT
											.getRotation()
											.rotateBy(Rotation2d.fromDegrees(180.0)),
									new Rotation2d()));
				} else {
					points.add(
							new PathPoint(
									LOWER_RIGHT_CONTROL_POINT.getTranslation(),
									LOWER_RIGHT_CONTROL_POINT
											.getRotation()
											.rotateBy(Rotation2d.fromDegrees(180.0)),
									new Rotation2d()));
				}
			}

			if (curX > HP_CONTROL_POINT.getX() && curY < HP_CONTROL_POINT.getY()) {
				points.add(
						new PathPoint(
								HP_CONTROL_POINT.getTranslation(),
								HP_CONTROL_POINT.getRotation(),
								goalPose.getRotation()));
			}
		}

		points.add(
				new PathPoint(
						goalPose.getTranslation(),
						Rotation2d.fromDegrees(-90.0),
						goalPose.getRotation()));

		PathPlannerTrajectory trajectory =
				PathPlanner.generatePath(
						new PathConstraints(translationVelocity, rotationVelocity), true, points);

		return trajectory;
	}

	public static Pose2d[] getControlPoints() {
		return CONTROL_POINTS;
	}
}
