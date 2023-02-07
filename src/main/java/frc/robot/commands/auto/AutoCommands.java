package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ObstacleAvoidance;

public final class AutoCommands {
	public static CommandBase resetPoseCommand(PathPlannerTrajectory trajectory, Swerve swerve) {
		SmartDashboard.putNumber("Initial X", trajectory.getInitialPose().getX());
		SmartDashboard.putNumber("Initial Y", trajectory.getInitialPose().getY());

		return new InstantCommand(
				() ->
						swerve.setPose(
								trajectory.getInitialPose(),
								trajectory.getInitialState().holonomicRotation));
	}

	public static CommandBase resetPoseCommand(Pose2d pose, Swerve swerve) {
		SmartDashboard.putNumber("Initial X", pose.getX());
		SmartDashboard.putNumber("Initial Y", pose.getY());

		return new InstantCommand(() -> swerve.setPose(pose, pose.getRotation()));
	}

	public static CommandBase generateTrajectoryCommand(
			Pose2d goalPose, double translationVelocity, double rotationVelocity, Swerve swerve) {
		return generateTrajectoryCommand(
				goalPose,
				Rotation2d.fromDegrees(180.0),
				translationVelocity,
				rotationVelocity,
				swerve);
	}

	public static CommandBase generateTrajectoryCommand(
			Pose2d goalPose,
			Rotation2d goalHeading,
			double translationVelocity,
			double rotationVelocity,
			Swerve swerve) {
		return new InstantCommand(
				() -> {
					PathPlannerTrajectory trajectory =
							PathPlanner.generatePath(
									new PathConstraints(translationVelocity, rotationVelocity),
									PathPoint.fromCurrentHolonomicState(
											swerve.getPose(), swerve.getVelocity()),
									new PathPoint(
											goalPose.getTranslation(),
											goalHeading,
											goalPose.getRotation()));

					swerve.setTrajectory(trajectory);
				});
	}

	public static CommandBase followTrajectoryCommand(
			PathPlannerTrajectory trajectory, boolean isFirstPath, Swerve swerve) {
		return Commands.sequence(
				new ConditionalCommand(
						resetPoseCommand(trajectory, swerve),
						new InstantCommand(),
						() -> isFirstPath),
				new InstantCommand(() -> swerve.setRunningTrajectory(true)),
				new PPSwerveControllerCommand(
						trajectory,
						swerve::getPose,
						new PIDController(
								Constants.SWERVE_X_KP,
								Constants.SWERVE_X_KI,
								Constants.SWERVE_X_KD),
						new PIDController(
								Constants.SWERVE_Y_KP,
								Constants.SWERVE_Y_KI,
								Constants.SWERVE_Y_KD),
						new PIDController(
								Constants.SWERVE_ROT_KP,
								Constants.SWERVE_ROT_KI,
								Constants.SWERVE_ROT_KD),
						swerve::drive,
						true,
						swerve),
				new InstantCommand(() -> swerve.setRunningTrajectory(false)));
	}

	public static CommandBase driveToCommand(Pose2d goalPose, Swerve swerve) {
		return new InstantCommand(
				() -> {
					CommandScheduler.getInstance()
							.schedule(
									AutoCommands.generateTrajectoryCommand(
													goalPose, 2.0, 2.0, swerve)
											.andThen(
													new FunctionalCommand(
															() -> {
																CommandScheduler.getInstance()
																		.schedule(
																				AutoCommands
																						.followTrajectoryCommand(
																								swerve
																										.getTrajectory(),
																								false,
																								swerve));
															},
															() -> {},
															(interrupted) -> {},
															() -> true,
															swerve)));
				});
	}

	public static CommandBase driveToAvoidObstaclesCommand(Pose2d goalPose, Swerve swerve) {
		return new InstantCommand(
						() -> {
							PathPlannerTrajectory trajectory =
									ObstacleAvoidance.generateTrajectoryAvoidObstacles(
											goalPose, 3.0, 3.0, swerve);
							swerve.setTrajectory(trajectory);
						})
				.andThen(
						new FunctionalCommand(
								() -> {
									CommandScheduler.getInstance()
											.schedule(
													followTrajectoryCommand(
															swerve.getTrajectory(), false, swerve));
								},
								() -> {},
								(interrupted) -> {},
								() -> true,
								swerve));
	}
}
