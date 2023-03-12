package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.superstructure.ScoreCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoBuilder extends SwerveAutoBuilder {

  private final Swerve swerve;
  private final Superstructure superstructure;

  private PathPlannerTrajectory trajectory;
  private boolean isRunning = false;

  public AutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      boolean useAllianceColor,
      Map<String, Command> eventMap,
      Swerve swerve,
      Superstructure superstructure) {
    super(poseSupplier, resetPose, translationConstants, rotationConstants, outputChassisSpeeds, eventMap, useAllianceColor, swerve);

    this.swerve = swerve;
    this.superstructure = superstructure;
  }

  public CommandBase followPath(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return Commands.sequence(
        new ConditionalCommand(
            this.resetPose(trajectory),
            Commands.none(),
            () -> isFirstPath
        ),
        Commands.runOnce(() -> setTrajectory(trajectory)),
        super.followPathWithEvents(trajectory),
        Commands.runOnce(() -> setRunning(false))
    );
  }

  public CommandBase followPathWithEvents(List<PathPlannerTrajectory> pathGroup, boolean isFirstPath) {
    return Commands.sequence(
        new ConditionalCommand(
            this.resetPose(pathGroup.get(0)),
            Commands.none(),
            () -> isFirstPath
        ),
        Commands.runOnce(() -> setTrajectory(pathGroup.get(0))),
        super.followPathGroupWithEvents(pathGroup),
        Commands.runOnce(() -> setRunning(false))
    );
  }

  public CommandBase generateTrajectoryCommand(
      Pose2d goalPose, double translationVelocity, double rotationVelocity) {
    return generateTrajectoryCommand(
        goalPose,
        Rotation2d.fromDegrees(180.0),
        translationVelocity,
        rotationVelocity);
  }

  public CommandBase generateTrajectoryCommand(
      Pose2d goalPose,
      Rotation2d goalHeading,
      double translationVelocity,
      double rotationVelocity) {
    return Commands.run(() -> {
      PathPlannerTrajectory trajectory =
          PathPlanner.generatePath(
              new PathConstraints(translationVelocity, rotationVelocity),
              PathPoint.fromCurrentHolonomicState(
                  swerve.getPose(), swerve.getVelocity()),
              new PathPoint(
                  goalPose.getTranslation(),
                  goalHeading,
                  goalPose.getRotation()));
      System.out.println(trajectory);
      followPath(trajectory, false);
    });
  }

  public CommandBase driveToCommand(Pose2d goalPose, Rotation2d goalHeading, double translationVel, double rotationVel) {
    return Commands.sequence(
        Commands.runOnce(() -> setTrajectory(
            PathPlanner.generatePath(
                new PathConstraints(translationVel, rotationVel),
                PathPoint.fromCurrentHolonomicState(
                    swerve.getPose(), swerve.getVelocity()),
                new PathPoint(
                    goalPose.getTranslation(),
                    goalHeading,
                    goalPose.getRotation()))
        )),
        Commands.runOnce(() -> CommandScheduler.getInstance().schedule(followPath(trajectory, false)))

    );
  }

  public CommandBase autoBalance() {
    return driveToCommand(
        new Pose2d(
            new Translation2d(3.53, swerve.getPose().getY()),
            Rotation2d.fromDegrees(0.0)
        ), new Rotation2d(0.0), 2.0, 2.0
    );
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory;
    setRunning(true);
  }

  public void setRunning(boolean isRunning) {
    this.isRunning = isRunning;
  }

  public boolean getRunning() {
    return this.isRunning;
  }

}