package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TwoElementGrabBalanceHP {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 2.0; // m/s^2

	private static final PathPlannerTrajectory TWO_ELEMENT_GRAB_BALANCE_HP =
			PathPlanner.loadPath("2ElementGrabBalanceHP", MAX_VEL, MAX_ACCEL);

	public static CommandBase twoElementGrabBalanceHP(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(TWO_ELEMENT_GRAB_BALANCE_HP, true);
	}
}
