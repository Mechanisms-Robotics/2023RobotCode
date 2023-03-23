package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OneElementBalanceWall {
	public static final double MAX_VEL = 1.5; // m/s
	public static final double MAX_ACCEL = 1.5; // m/s^2

	private static final PathPlannerTrajectory ONE_ELEMENT_BALANCE_WALL =
			PathPlanner.loadPath("1ElementBalanceWall", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneElementBalanceWall(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(ONE_ELEMENT_BALANCE_WALL, true);
	}
}
