package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OneElementGrabBalanceHP {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 2.0; // m/s^2

	private static final PathPlannerTrajectory ONE_ELEMENT_GRAB_BALANCE_HP =
			PathPlanner.loadPath("1ElementGrabBalanceHP", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneElementGrabBalanceHP(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(ONE_ELEMENT_GRAB_BALANCE_HP, true);
	}
}
