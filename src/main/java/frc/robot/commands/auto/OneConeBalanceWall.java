package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Element;
import frc.robot.subsystems.Swerve;

public class OneConeBalanceWall {
	public static final double MAX_VEL = 1.375; // m/s
	public static final double MAX_ACCEL = 1.375; // m/s^2

	private static final PathPlannerTrajectory ONE_CONE_BALANCE_WALL =
			PathPlanner.loadPath("1ConeBalanceWall", MAX_VEL, MAX_ACCEL);

	public static CommandBase oneConeBalanceWall(
			AutoBuilder autoBuilder, Superstructure superstructure) {
		return Commands.sequence(
				new InstantCommand(() -> superstructure.setElement(Element.Cone)),
				new InstantCommand(() -> superstructure.setNode(2, 0)),
				new InstantCommand(superstructure::idle),
				new WaitCommand(0.75),
				new InstantCommand(superstructure::prep),
				new WaitCommand(2.25),
				new InstantCommand(superstructure::score),
				new WaitCommand(0.5),
				new InstantCommand(superstructure::idle),
				autoBuilder.followPath(ONE_CONE_BALANCE_WALL, true),
				new WaitCommand(1.0));
	}
}
