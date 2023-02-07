package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.Supplier;

public class GoalTracker extends SubsystemBase {
	private static final Translation2d LEFT_LOW_POSITION = new Translation2d(1.16, 0.48);
	private static final Translation2d UP_OFFSET = new Translation2d(0.0, 0.55);

	private final Field2d m_field;

	private final ArrayList<Pose2d> m_goalPoses = new ArrayList<>();

	private final Supplier<Pose2d> m_poseSupplier;
	private Pose2d m_closestGoalPosition = new Pose2d(LEFT_LOW_POSITION, new Rotation2d());

	public GoalTracker(Field2d field, Supplier<Pose2d> poseSupplier) {
		initGoalPositions();
		field.getObject("dGoals").setPoses(m_goalPoses);

		m_field = field;
		m_poseSupplier = poseSupplier;
	}

	private void initGoalPositions() {
		for (int i = 0; i < 9; i++) {
			m_goalPoses.add(
					new Pose2d(LEFT_LOW_POSITION.plus(UP_OFFSET.times(i)), new Rotation2d()));
		}
	}

	public Pose2d getClosestGoal() {
		return m_closestGoalPosition;
	}

	@Override
	public void periodic() {
		for (int i = 0; i < 9; i++) {
			double distance_best =
					m_closestGoalPosition.minus(m_poseSupplier.get()).getTranslation().getNorm();
			double distance_i =
					m_goalPoses.get(i).minus(m_poseSupplier.get()).getTranslation().getNorm();

			if (distance_i < distance_best) {
				m_closestGoalPosition = m_goalPoses.get(i);
			}
		}

		m_field.getObject("eClosest Goal").setPose(m_closestGoalPosition);
	}
}
