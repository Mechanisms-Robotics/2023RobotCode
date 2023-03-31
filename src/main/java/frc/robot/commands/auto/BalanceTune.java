package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceTune {
	public static final double MAX_VEL = 1.5; // m/s
	public static final double MAX_ACCEL = 2.0; // m/s^2

	private static final PathPlannerTrajectory BALANCE_TUNE =
			PathPlanner.loadPath("BalanceTune", MAX_VEL, MAX_ACCEL);

	public static CommandBase balanceTune(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(BALANCE_TUNE, true);
	}
}
