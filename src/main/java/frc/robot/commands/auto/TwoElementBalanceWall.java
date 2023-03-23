package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TwoElementBalanceWall {
	public static final double MAX_VEL = 1.5; // m/s
	public static final double MAX_ACCEL = 1.5; // m/s^2

	private static final PathPlannerTrajectory TWO_ELEMENT_BALANCE_WALL =
			PathPlanner.loadPath("2ElementBalanceWall", MAX_VEL, MAX_ACCEL);

	public static CommandBase twoElementBalanceWall(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(TWO_ELEMENT_BALANCE_WALL, true);
	}
}
