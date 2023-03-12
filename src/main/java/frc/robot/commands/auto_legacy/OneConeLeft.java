package frc.robot.commands.auto_legacy;

import static frc.robot.commands.auto_legacy.AutoCommands.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class OneConeLeft {
	public static final double MAX_VEL = 2.0; // m/s
	public static final double MAX_ACCEL = 1.0; // m/s^2

	private static final PathPlannerTrajectory ONE_CONE_LEFT =
			PathPlanner.loadPath("1ConeLeft", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneConeLeft(Swerve swerveSubsystem) {
		return followTrajectoryCommand(ONE_CONE_LEFT, true, swerveSubsystem);
	}
}
