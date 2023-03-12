package frc.robot.commands.auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.ScoreCommand;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Element;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class OneConeOneCubeHP {
  public static final double MAX_VEL = 2.0; // m/s
  public static final double MAX_ACCEL = 1.0; // m/s^2

  private static final PathPlannerTrajectory ONE_CONE_ONE_CUBE_HP =
      PathPlanner.loadPath("1Cone1CubeHP", MAX_VEL, MAX_ACCEL);

  public static CommandBase oneConeOneCubeLeft(AutoBuilder autoBuilder, Superstructure superstructure, Intake intake) {
    return Commands.sequence(
        new InstantCommand(() -> superstructure.setElement(Element.Cone)),
        new InstantCommand(() -> superstructure.setNode(2, 0)),
        new InstantCommand(superstructure::idle),
        new WaitCommand(0.5),
        new InstantCommand(superstructure::prep),
        new WaitCommand(2.0),
        new InstantCommand(superstructure::score),
        new WaitCommand(0.5),
        new InstantCommand(superstructure::idle),
        new InstantCommand(() -> superstructure.setElement(Element.Cube)),
        new DeployIntakeCommand(intake),
        new InstantCommand(superstructure::intake),
        autoBuilder.followPath(ONE_CONE_ONE_CUBE_HP, true),
        new InstantCommand(superstructure::prep),
        new WaitCommand(2.0),
        new InstantCommand(superstructure::score),
        new WaitCommand(0.5),
        new InstantCommand(superstructure::idle)
    );
  }
}
