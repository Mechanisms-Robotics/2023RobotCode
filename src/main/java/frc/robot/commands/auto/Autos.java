package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Autos {

    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    static {
        EVENT_MAP.put("marker0", Commands.none());
        EVENT_MAP.put("marker1", Commands.none());
    }

    public static final PathPlannerTrajectory MOBILITY_AUTO_LEFT = PathPlanner.loadPath("MobilityAutoLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory MOBILITY_AUTO_RIGHT = PathPlanner.loadPath("MobilityAutoRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_LEFT = PathPlanner.loadPath("1ConeLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_ONE_CUBE_LEFT = PathPlanner.loadPath("1Cone1CubeLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_ONE_CUBE_RIGHT = PathPlanner.loadPath("1Cone1CubeRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_RIGHT = PathPlanner.loadPath("1ConeRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_TWO_CUBES_LEFT = PathPlanner.loadPath("1Cone2CubesLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_TWO_CUBES_RIGHT = PathPlanner.loadPath("1Cone2CubesRight", new PathConstraints(2.0, 1.0));

    public static Command followPath(PathPlannerTrajectory trajectory, SwerveAutoBuilder swerveAutoBuilder, Swerve swerve, boolean isFirstPath) {
        return Commands.sequence(
                new ConditionalCommand(swerveAutoBuilder.resetPose(trajectory), Commands.none(), () -> isFirstPath),
                Commands.run(() -> swerve.setRunningTrajectory(true)),
                swerveAutoBuilder.followPath(trajectory),
                Commands.run(() -> swerve.setRunningTrajectory(false))
        );
    }

    public static CommandBase generateTrajectoryCommand(
            Pose2d goalPose, double translationVelocity, double rotationVelocity, SwerveAutoBuilder swerveAutoBuilder, Swerve swerve) {
        return generateTrajectoryCommand(
                goalPose,
                Rotation2d.fromDegrees(180.0),
                swerveAutoBuilder,
                translationVelocity,
                rotationVelocity,
                swerve);
    }

    public static CommandBase generateTrajectoryCommand(
            Pose2d goalPose,
            Rotation2d goalHeading,
            SwerveAutoBuilder swerveAutoBuilder,
            double translationVelocity,
            double rotationVelocity,
            Swerve swerve) {
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

            swerveAutoBuilder.followPath(trajectory);
        });
    }

    public static CommandBase driveToCommand(Pose2d goalPose, SwerveAutoBuilder swerveAutoBuilder, Swerve swerve) {
        return new InstantCommand(
                () -> {
                    CommandScheduler.getInstance()
                            .schedule(
                                    Autos.generateTrajectoryCommand(
                                                    goalPose, 2.0, 2.0, swerveAutoBuilder, swerve)
                                            .andThen(
                                                    new FunctionalCommand(
                                                            () -> {
                                                                CommandScheduler.getInstance()
                                                                        .schedule(
                                                                                Autos
                                                                                        .followPath(
                                                                                                swerve
                                                                                                        .getTrajectory(),
                                                                                                swerveAutoBuilder,
                                                                                                swerve,
                                                                                                false));
                                                            },
                                                            () -> {},
                                                            (interrupted) -> {},
                                                            () -> true,
                                                            swerve)));
                });
    }

}
