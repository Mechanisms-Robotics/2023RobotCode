package frc.robot.commands.auto;

import static frc.robot.commands.auto.AutoCommands.followPathCommand;
import static frc.robot.commands.auto.AutoCommands.resetPoseCommand;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class OneConeTwoCubesLeft {
	public static final double MAX_VEL = 4.5; // m/s
	public static final double MAX_ACCEL = 4.5; // m/s^2

	private static final PathPlannerTrajectory ONE_CONE_TWO_CUBES_LEFT =
			PathPlanner.loadPath("1Cone2CubesLeft", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneConeTwoCubesLeft(Swerve swerveSubsystem) {
		return Commands.parallel(
				resetPoseCommand(ONE_CONE_TWO_CUBES_LEFT, swerveSubsystem),
				followPathCommand(ONE_CONE_TWO_CUBES_LEFT, swerveSubsystem));
	}
}
