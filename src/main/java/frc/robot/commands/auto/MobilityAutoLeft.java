package frc.robot.commands.auto;

import static frc.robot.commands.auto.AutoCommands.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class MobilityAutoLeft {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 1.0; // m/s^2

	private static final PathPlannerTrajectory MOBILITY_AUTO_LEFT =
			PathPlanner.loadPath("MobilityAutoLeft", MAX_VEL, MAX_ACCEL);

	public static CommandBase mobilityAutoLeftCommand(Swerve swerveSubsystem) {
		return Commands.parallel(
				resetPoseCommand(MOBILITY_AUTO_LEFT, swerveSubsystem),
				followPathCommand(MOBILITY_AUTO_LEFT, swerveSubsystem));
	}
}
