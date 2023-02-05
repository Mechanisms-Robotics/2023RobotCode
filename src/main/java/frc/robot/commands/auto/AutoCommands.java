package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public final class AutoCommands {

	public static final double MAX_VEL = 1.7;
	public static final double MAX_ACCEL = 1.0; // m/s^2

	private static final PathPlannerTrajectory MOVE_FORWARD_PATH =
			PathPlanner.loadPath("MoveForward", MAX_VEL, MAX_ACCEL);

	public static CommandBase resetPoseCommand(PathPlannerTrajectory trajectory, Swerve swerve) {
		SmartDashboard.putNumber("Initial X", trajectory.getInitialPose().getX());
		SmartDashboard.putNumber("Initial Y", trajectory.getInitialPose().getY());

		return new InstantCommand(
				() ->
						swerve.setPose(
								trajectory.getInitialPose(),
								trajectory.getInitialState().holonomicRotation));
	}

	public static CommandBase followPathCommand(PathPlannerTrajectory trajectory, Swerve swerve) {
		return Commands.sequence(
				new FunctionalCommand(
						() -> swerve.followTrajectory(trajectory),
						() -> {},
						interrupted -> {},
						swerve::isTrajectoryFinished,
						swerve),
				new InstantCommand(swerve::stop, swerve));
	}

	public static CommandBase moveForwardCommand(Swerve swerve) {
		return Commands.parallel(
				resetPoseCommand(MOVE_FORWARD_PATH, swerve),
				followPathCommand(MOVE_FORWARD_PATH, swerve));
	}
}
