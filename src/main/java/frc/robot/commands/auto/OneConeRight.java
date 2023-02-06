package frc.robot.commands.auto;

import static frc.robot.commands.auto.AutoCommands.followPathCommand;
import static frc.robot.commands.auto.AutoCommands.resetPoseCommand;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class OneConeRight {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 1.0; // m/s^2

	private static final PathPlannerTrajectory ONE_CONE_RIGHT =
			PathPlanner.loadPath("1ConeRight", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneConeRight(Swerve swerveSubsystem) {
		return Commands.parallel(
				resetPoseCommand(ONE_CONE_RIGHT, swerveSubsystem),
				followPathCommand(ONE_CONE_RIGHT, swerveSubsystem));
	}
}
