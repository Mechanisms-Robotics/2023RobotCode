package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;
import java.util.function.Supplier;

public class DriveToCommand extends CommandBase {
	private static final double RAMP_X_BORDER = 3.0;
	private static final double RAMP_Y_BORDER = 2.7;

	private static final Translation2d TOP_RIGHT_RAMP_CONTROL_POINT = new Translation2d(5.6, 4.875);
	private static final Translation2d TOP_LEFT_RAMP_CONTROL_POINT = new Translation2d(2.25, 4.75);
	private static final Translation2d BOTTOM_RIGHT_RAMP_CONTROL_POINT = new Translation2d(5.6, 0.75);
	private static final Translation2d BOTTOM_LEFT_RAMP_CONTROL_POINT = new Translation2d(2.25, 0.75);

	private final Swerve m_swerveSubsystem;
	private Pose2d m_goalPose;
	private Supplier<Pose2d> m_goalPoseSupplier;
	private final double m_translationalVelocity;
	private final double m_angularVelocity;

	public DriveToCommand(
			Swerve swerveSubsystem,
			Supplier<Pose2d> goalPoseSupplier,
			double translationalVelocity,
			double angularVelocity) {
		m_swerveSubsystem = swerveSubsystem;
		m_goalPoseSupplier = goalPoseSupplier;
		m_translationalVelocity = translationalVelocity;
		m_angularVelocity = angularVelocity;

		addRequirements(swerveSubsystem);
	}

	public DriveToCommand(
			Swerve swerveSubsystem,
			Pose2d goalPose,
			double translationalVelocity,
			double angularVelocity) {
		m_swerveSubsystem = swerveSubsystem;
		m_goalPose = goalPose;
		m_translationalVelocity = translationalVelocity;
		m_angularVelocity = angularVelocity;

		addRequirements(swerveSubsystem);
	}

	@Override
	public void initialize() {
		if (m_goalPoseSupplier != null) {
			m_goalPose = m_goalPoseSupplier.get();
		}

		ArrayList<PathPoint> pathPoints = new ArrayList<>();

		pathPoints.add(
				PathPoint.fromCurrentHolonomicState(
						m_swerveSubsystem.getPose(), m_swerveSubsystem.getVelocity()));

		if (m_swerveSubsystem.getPose().getX() > RAMP_X_BORDER
				&& m_goalPose.getX() < RAMP_X_BORDER) {
			if (m_swerveSubsystem.getPose().getY() > RAMP_Y_BORDER) {
				pathPoints.add(
						new PathPoint(
								TOP_RIGHT_RAMP_CONTROL_POINT,
								TOP_RIGHT_RAMP_CONTROL_POINT
										.minus(m_swerveSubsystem.getPose().getTranslation())
										.getAngle(),
								m_goalPose.getRotation()));
				pathPoints.add(
						new PathPoint(
								TOP_LEFT_RAMP_CONTROL_POINT,
								TOP_LEFT_RAMP_CONTROL_POINT
										.minus(m_swerveSubsystem.getPose().getTranslation())
										.getAngle(),
								m_goalPose.getRotation()));
			} else {
				pathPoints.add(
						new PathPoint(
								BOTTOM_RIGHT_RAMP_CONTROL_POINT,
								BOTTOM_RIGHT_RAMP_CONTROL_POINT
										.minus(m_swerveSubsystem.getPose().getTranslation())
										.getAngle(),
								m_goalPose.getRotation()));
				pathPoints.add(
						new PathPoint(
								BOTTOM_LEFT_RAMP_CONTROL_POINT,
								BOTTOM_LEFT_RAMP_CONTROL_POINT
										.minus(m_swerveSubsystem.getPose().getTranslation())
										.getAngle(),
								m_goalPose.getRotation()));
			}
		}

		pathPoints.add(
				new PathPoint(
						m_goalPose.getTranslation(),
						m_goalPose.minus(m_swerveSubsystem.getPose()).getTranslation().getAngle(),
						m_goalPose.getRotation()));

		PathPlannerTrajectory trajectory =
				PathPlanner.generatePath(
						new PathConstraints(m_translationalVelocity, m_angularVelocity),
						pathPoints);

		m_swerveSubsystem.followTrajectory(trajectory);
	}

	@Override
	public boolean isFinished() {
		return m_swerveSubsystem.isTrajectoryFinished();
	}

	@Override
	public void end(boolean interrupted) {
		m_swerveSubsystem.drive(new ChassisSpeeds());
	}
}
