package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;

public class ObstacleAvoidance {
	private static final double BORDER_1_X = 2.95;
	private static final double BORDER_2_X = 4.875;

	private static final double BORDER_Y = 2.75;

	private static final Pose2d UPPER_RIGHT_CONTROL_POINT =
			new Pose2d(new Translation2d(5.50, 4.75), Rotation2d.fromDegrees(180.0));
	private static final Pose2d UPPER_LEFT_CONTROL_POINT =
			new Pose2d(new Translation2d(2.25, 4.75), Rotation2d.fromDegrees(180.0));

	private static final Pose2d LOWER_RIGHT_CONTROL_POINT =
			new Pose2d(new Translation2d(5.50, 0.75), Rotation2d.fromDegrees(180.0));
	private static final Pose2d LOWER_LEFT_CONTROL_POINT =
			new Pose2d(new Translation2d(2.25, 0.75), Rotation2d.fromDegrees(180.0));

	public static PathPlannerTrajectory generateTrajectoryAvoidObstacles(
			Pose2d goalPose, double translationVelocity, double rotationVelocity, Swerve swerve) {
		ArrayList<PathPoint> points = new ArrayList<>();
		points.add(PathPoint.fromCurrentHolonomicState(swerve.getPose(), swerve.getVelocity()));

		double curX = swerve.getPose().getX();
		double curY = swerve.getPose().getY();

		if (BORDER_2_X < curX) {
			if (BORDER_Y < curY) {
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

		if (BORDER_1_X < curX) {
			if (BORDER_Y < curY) {
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

		points.add(
				new PathPoint(
						goalPose.getTranslation(),
						Rotation2d.fromDegrees(180.0),
						goalPose.getRotation()));

		PathPlannerTrajectory trajectory =
				PathPlanner.generatePath(
						new PathConstraints(translationVelocity, rotationVelocity), true, points);

		return trajectory;
	}
}
