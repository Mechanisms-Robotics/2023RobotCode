package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class DriveToCommand extends CommandBase {
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

		PathPlannerTrajectory trajectory =
				PathPlanner.generatePath(
						new PathConstraints(m_translationalVelocity, m_angularVelocity),
						PathPoint.fromCurrentHolonomicState(
								m_swerveSubsystem.getPose(), m_swerveSubsystem.getVelocity()),
						new PathPoint(
								m_goalPose.getTranslation(),
								m_goalPose.minus(m_swerveSubsystem.getPose()).getTranslation().getAngle(),
								m_goalPose.getRotation()));

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
