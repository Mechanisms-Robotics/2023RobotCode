package frc.robot.commands.auto;

import static frc.robot.commands.auto.AutoCommands.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class OneConeLeft {
  public static final double MAX_VEL = 2.0; // m/s
  public static final double MAX_ACCEL = 1.0; // m/s^2

  private static final PathPlannerTrajectory ONE_CONE_LEFT =
      PathPlanner.loadPath("1ConeLeft", MAX_VEL, MAX_ACCEL);

  public static CommandBase oneConeLeft(Swerve swerveSubsystem) {
    return Commands.parallel(
        resetPoseCommand(ONE_CONE_LEFT, swerveSubsystem),
        followPathCommand(ONE_CONE_LEFT, swerveSubsystem));
  }
}
