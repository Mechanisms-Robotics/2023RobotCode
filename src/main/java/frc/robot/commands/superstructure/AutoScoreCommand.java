// package frc.robot.commands.superstructure;
//
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.auto.AutoBuilder;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Swerve;
// import frc.robot.util.GoalTracker;
//
// public class AutoScoreCommand extends CommandBase {
//	private static final Transform2d LINEUP_OFFSET =
//			new Transform2d(new Translation2d(1.25, 0.0), new Rotation2d());
//	private static final Transform2d PLACE_OFFSET =
//			new Transform2d(new Translation2d(0.75, 0.0), new Rotation2d());
//
//	private enum ScoreState {
//		DrivingTo,
//		Prepping,
//		Placing,
//		Done
//	}
//
//	private final AutoBuilder m_autoBuilder;
//	private final Swerve m_swerve;
//	private final GoalTracker m_goalTracker;
//	private final Superstructure m_superstructure;
//
//	private ScoreState m_state;
//
//	public AutoScoreCommand(
//			AutoBuilder autoBuilder,
//			Swerve swerve,
//			GoalTracker goalTracker,
//			Superstructure superstructure) {
//		m_autoBuilder = autoBuilder;
//		m_swerve = swerve;
//		m_goalTracker = goalTracker;
//		m_superstructure = superstructure;
//
//		addRequirements(m_superstructure);
//	}
//
//	@Override
//	public void execute() {
//		if (m_state == null) {
//			if (!m_swerve.getRunningTrajectory()) {
//				CommandScheduler.getInstance()
//						.schedule(
//								m_autoBuilder.driveToAvoidObstaclesCommand(
//										m_goalTracker.getTargetGoal().transformBy(LINEUP_OFFSET),
//										m_swerve));
//			} else {
//				m_state = ScoreState.DrivingTo;
//			}
//		} else if (m_state == ScoreState.DrivingTo) {
//			if (!m_swerve.getRunningTrajectory()) {
//				m_state = ScoreState.Prepping;
//
//				m_superstructure.prep();
//			}
//		} else if (m_state == ScoreState.Prepping) {
//			if (m_superstructure.atPosition() || RobotBase.isSimulation()) {
//				if (!m_swerve.getRunningTrajectory()) {
//					CommandScheduler.getInstance()
//							.schedule(
//									m_autoBuilder.driveToAvoidObstaclesCommand(
//											m_goalTracker.getTargetGoal().transformBy(PLACE_OFFSET),
//											m_swerve));
//				} else {
//					m_state = ScoreState.Placing;
//				}
//			}
//		} else if (m_state == ScoreState.Placing) {
//			if (!m_swerve.getRunningTrajectory()) {
//				m_state = ScoreState.Done;
//
//				m_superstructure.score();
//			}
//		}
//	}
//
//	@Override
//	public boolean isFinished() {
//		if (m_state != null) SmartDashboard.putString("ScoreState", m_state.toString());
//
//		return m_state == ScoreState.Done;
//	}
// }
