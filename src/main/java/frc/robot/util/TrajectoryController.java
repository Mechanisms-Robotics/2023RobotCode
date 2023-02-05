package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** This class is used for controlling the swerve drive along a predefined trajectory. */
public class TrajectoryController {
	private static final double X_GAIN = 1.25 * 3.0;
	private static final double Y_GAIN = X_GAIN;
	private static final double THETA_GAIN = 2.0;
	private static final TrapezoidProfile.Constraints HEADING_PROFILE_CONSTRAINTS =
			new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI);

	private static final double ALLOWABLE_POSITION_ERROR = 0.1; // meters
	private static final double ALLOWABLE_ROTATION_ERROR = 1.0; // degrees

	private final Timer timer = new Timer();
	private PathPlannerTrajectory trajectory;
	private final SwerveDriveKinematics kinematics;
	private final HolonomicDriveController controller;
	private boolean isFinished = true;

	/**
	 * Construction a Trajectory Controller
	 *
	 * @param kinematics The kinematics controller for the swerve drive.
	 */
	public TrajectoryController(SwerveDriveKinematics kinematics) {
		this.kinematics = kinematics;

		ProfiledPIDController thetaController =
				new ProfiledPIDController(THETA_GAIN, 0.0, 0.0, HEADING_PROFILE_CONSTRAINTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI); // Thanks mendax1234
		thetaController.setTolerance(0.25);

		this.controller =
				new HolonomicDriveController(
						new PIDController(X_GAIN, 0.0, 0.0, Constants.LOOP_TIME),
						new PIDController(Y_GAIN, 0.0, 0.0, Constants.LOOP_TIME),
						thetaController);

		this.controller.setTolerance(
				new Pose2d(
						new Translation2d(ALLOWABLE_POSITION_ERROR, ALLOWABLE_POSITION_ERROR),
						Rotation2d.fromDegrees(ALLOWABLE_ROTATION_ERROR)));
	}

	/**
	 * Start a trajectory. The calculate functions needs to be called periodically after this
	 * function is called.
	 *
	 * @param trajectory The trajectory to follow.
	 */
	public void startTrajectory(PathPlannerTrajectory trajectory) {
		isFinished = false;
		timer.reset();
		timer.start();
		this.trajectory = trajectory;
	}

	/**
	 * Get the current trajectory
	 *
	 * @return The current trajectory
	 */
	public PathPlannerTrajectory getTrajectory() {
		return trajectory;
	}

	/**
	 * Call this function periodically after calling start trajectory. It returns the chassis speeds
	 * needed to follow the trajectory.
	 *
	 * @param currentPose The current pose of the robot
	 * @return The chassis speeds need to follow this trajectory.
	 */
	public ChassisSpeeds calculate(Pose2d currentPose) {
		if (isFinished()) {
			return new ChassisSpeeds();
		}

		final double currentTime = timer.get();
		final var desiredState =
				(PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
		desiredState.poseMeters = desiredState.poseMeters.transformBy(Constants.FIELD_ROBOT);
		desiredState.holonomicRotation.rotateBy(Constants.FIELD_ROBOT.getRotation());
		return controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
	}

	/**
	 * Return weather the trajectory is finished or not. This is based on weather or not enough time
	 * has passed not where the robot is.
	 *
	 * @return True if the trajectory is finish
	 */
	public boolean isFinished() {
		if (trajectory == null) {
			return true;
		}

		if (timer.hasElapsed(trajectory.getTotalTimeSeconds())) {
			isFinished = true;
		}

		if (timer.hasElapsed(trajectory.getTotalTimeSeconds() - 1.0) && controller.atReference()) {
			return true;
		}

		return isFinished;
	}

	/** Stop any currently running trajectory. */
	public void stop() {
		isFinished = true;
		timer.stop();
		timer.reset();
	}
}
