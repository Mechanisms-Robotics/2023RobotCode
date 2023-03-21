package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThreeElementHP {
	public static final double MAX_VEL = 3.0; // m/s
	public static final double MAX_ACCEL = 2.5; // m/s^2

	private static final PathPlannerTrajectory THREE_ELEMENT_HP =
			PathPlanner.loadPath("3ElementHP", MAX_VEL, MAX_ACCEL);

	public static CommandBase threeElementHP(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(THREE_ELEMENT_HP, true);
	}
}
