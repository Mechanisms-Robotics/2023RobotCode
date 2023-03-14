package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAuto {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 1.0; // m/s^2

	private static final PathPlannerTrajectory TEST =
			PathPlanner.loadPath("Test", MAX_VEL, MAX_ACCEL);

	public static CommandBase testAuto(AutoBuilder autoBuilder) {
		return autoBuilder.followPath(TEST, true);
	}
}
