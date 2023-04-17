package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Swerve;

public class TwoElementGrabHP {
	public static final double MAX_VEL = 3.5; // m/s
	public static final double MAX_ACCEL = 3.5; // m/s^2

	private static final PathPlannerTrajectory TWO_ELEMENT_HP =
			PathPlanner.loadPath("2ElementHP", MAX_VEL, MAX_ACCEL);
	private static final PathPlannerTrajectory TWO_ELEMENT_GRAB_HP =
			PathPlanner.loadPath("2ElementGrabHP", MAX_VEL, MAX_ACCEL);

	public static CommandBase twoElementGrabHP(AutoBuilder autoBuilder, Swerve swerve) {
		return new SequentialCommandGroup(
				autoBuilder.followPath(TWO_ELEMENT_HP, true),
				new WaitUntilCommand(() -> !swerve.getRunningTrajectory()),
				autoBuilder.followPath(TWO_ELEMENT_GRAB_HP, false));
	}
}
