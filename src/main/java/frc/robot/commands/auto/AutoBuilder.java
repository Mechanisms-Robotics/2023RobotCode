package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Swerve;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoBuilder extends SwerveAutoBuilder {

    private final Swerve swerve;
    private PathPlannerTrajectory trajectory;
    private boolean isRunning = false;

    public AutoBuilder(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            SwerveDriveKinematics kinematics,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Map<String, Command> eventMap,
            boolean useAllianceColor,
            Swerve swerve) {
        super(poseSupplier, resetPose, kinematics, translationConstants, rotationConstants, outputModuleStates, eventMap, useAllianceColor, swerve);
        this.swerve = swerve;
    }

    public CommandBase followPath(PathPlannerTrajectory trajectory, boolean isFirstPath) {
        return Commands.sequence(
                new ConditionalCommand(
                        this.resetPose(trajectory),
                        Commands.none(),
                        () -> isFirstPath
                ),
                Commands.runOnce(() -> setTrajectory(trajectory)),
                super.followPath(trajectory),
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
