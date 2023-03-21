package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TwoElementWall {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 2.0; // m/s^2

	private static final PathPlannerTrajectory TWO_ELEMENT_WALL =
			PathPlanner.loadPath("2ElementWall", MAX_VEL, MAX_ACCEL);

	public static CommandBase twoElementWall(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(TWO_ELEMENT_WALL, true);
	}
}
