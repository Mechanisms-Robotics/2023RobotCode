package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.Supplier;

public class GoalTracker extends SubsystemBase {
	private static final Translation2d LEFT_LOW_POSITION = new Translation2d(1.16, 0.48);
	private static final Translation2d UP_OFFSET = new Translation2d(0.0, 0.565);

	private final Field2d m_field;

	private final ArrayList<Pose2d> m_goalPoses = new ArrayList<>();

	private final Supplier<Pose2d> m_poseSupplier;
	private Pose2d m_targetGoalPosition = new Pose2d(LEFT_LOW_POSITION, new Rotation2d());
	private final int[] m_targetNode = {0, 0};

	public enum TrackingMode {
		BestGoal,
		ClosestGoal
	}

	private TrackingMode m_trackingMode = TrackingMode.BestGoal;

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

	public void setTargetGoal(int col) {
		m_targetGoalPosition = m_goalPoses.get(col);
		m_targetNode[0] = (int) SmartDashboard.getNumber("TargetRow", 0);
		m_targetNode[1] = col;
	}

	public Pose2d getTargetGoal() {
		return m_targetGoalPosition;
	}

	public int[] getTargetNode() {
		return m_targetNode;
	}

	public void setTrackingMode(TrackingMode trackingMode) {
		m_trackingMode = trackingMode;
	}

	public TrackingMode getTrackingMode() {
		return m_trackingMode;
	}

	@Override
	public void periodic() {
		if (m_trackingMode == TrackingMode.BestGoal) {
			setTargetGoal((int) SmartDashboard.getNumber("TargetCol", 0));
		} else {
			int closest_goal = 0;
			double closest_distance = 1000.0; // meters

			for (int i = 0; i < 9; i++) {
				double distance =
						m_goalPoses
								.get(i)
								.getTranslation()
								.getDistance(m_poseSupplier.get().getTranslation());

				if (distance < closest_distance) {
					closest_goal = i;
					closest_distance = distance;
				}
			}

			setTargetGoal(closest_goal);
		}

		m_field.getObject("eTarget Goal").setPose(m_targetGoalPosition);
	}
}
