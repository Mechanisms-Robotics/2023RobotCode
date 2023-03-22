package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ObstacleAvoidance;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoBuilder extends SwerveAutoBuilder {

	private final Swerve swerve;
	private final Superstructure superstructure;

	private PathPlannerTrajectory trajectory;
	private boolean isRunning = false;

	public AutoBuilder(
			Supplier<Pose2d> poseSupplier,
			Consumer<Pose2d> resetPose,
			PIDConstants translationConstants,
			PIDConstants rotationConstants,
			Consumer<ChassisSpeeds> outputChassisSpeeds,
			boolean useAllianceColor,
			Map<String, Command> eventMap,
			Swerve swerve,
			Superstructure superstructure) {
		super(
				poseSupplier,
				resetPose,
				translationConstants,
				rotationConstants,
				outputChassisSpeeds,
				eventMap,
				useAllianceColor,
				swerve);

		this.swerve = swerve;
		this.superstructure = superstructure;
	}

	public CommandBase followPath(PathPlannerTrajectory trajectory, boolean isFirstPath) {
		return Commands.sequence(
				new ConditionalCommand(
						this.resetPose(trajectory), Commands.none(), () -> isFirstPath),
				Commands.runOnce(() -> setTrajectory(trajectory)),
				super.fullAuto(trajectory),
				Commands.runOnce(() -> setRunning(false)));
	}

	public CommandBase generateTrajectoryCommand(
			Pose2d goalPose, double translationVelocity, double rotationVelocity) {
		return generateTrajectoryCommand(
				goalPose, Rotation2d.fromDegrees(180.0), translationVelocity, rotationVelocity);
	}

	public CommandBase generateTrajectoryCommand(
			Pose2d goalPose,
			Rotation2d goalHeading,
			double translationVelocity,
			double rotationVelocity) {
		return Commands.run(
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
					System.out.println(trajectory);
					followPath(trajectory, false);
				});
	}

	public CommandBase driveToCommand(
			Pose2d goalPose, Rotation2d goalHeading, double translationVel, double rotationVel) {
		return Commands.sequence(
				Commands.runOnce(
						() ->
								setTrajectory(
										PathPlanner.generatePath(
												new PathConstraints(translationVel, rotationVel),
												PathPoint.fromCurrentHolonomicState(
														swerve.getPose(), swerve.getVelocity()),
												new PathPoint(
														goalPose.getTranslation(),
														goalHeading,
														goalPose.getRotation())))),
				Commands.runOnce(
						() ->
								CommandScheduler.getInstance()
										.schedule(followPath(trajectory, false))));
	}

	public CommandBase followTrajectoryCommand(
			PathPlannerTrajectory trajectory, boolean isFirstPath, Swerve swerve) {
		return Commands.sequence(
				new ConditionalCommand(
						super.resetPose(trajectory), new InstantCommand(), () -> isFirstPath),
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

	public CommandBase driveToAvoidObstaclesCommand(Pose2d goalPose, Swerve swerve) {
		return new InstantCommand(
						() -> {
							PathPlannerTrajectory trajectory =
									ObstacleAvoidance.generateTrajectoryAvoidObstacles(
											goalPose, 1.5, 1.5, swerve);
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

	public void setTrajectory(PathPlannerTrajectory trajectory) {
		this.trajectory = trajectory;
		setRunning(true);
	}

	public void setRunning(boolean isRunning) {
		this.isRunning = isRunning;
	}

	public boolean getRunning() {
		return this.isRunning;
	}
}
