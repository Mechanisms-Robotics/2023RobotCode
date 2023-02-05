package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

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
}
