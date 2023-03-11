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

public final class Autos {

    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    static {
        EVENT_MAP.put("marker0", Commands.print("marker0"));
        EVENT_MAP.put("marker1", Commands.print("marker1"));
    }

    public static final PathPlannerTrajectory MOBILITY_AUTO_LEFT = PathPlanner.loadPath("MobilityAutoLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory MOBILITY_AUTO_RIGHT = PathPlanner.loadPath("MobilityAutoRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_LEFT = PathPlanner.loadPath("1ConeLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_ONE_CUBE_LEFT = PathPlanner.loadPath("1Cone1CubeLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_ONE_CUBE_RIGHT = PathPlanner.loadPath("1Cone1CubeRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_RIGHT = PathPlanner.loadPath("1ConeRight", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_TWO_CUBES_LEFT = PathPlanner.loadPath("1Cone2CubesLeft", new PathConstraints(2.0, 1.0));
    public static final PathPlannerTrajectory ONE_CONE_TWO_CUBES_RIGHT = PathPlanner.loadPath("1Cone2CubesRight", new PathConstraints(2.0, 1.0));

}
