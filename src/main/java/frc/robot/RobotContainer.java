package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Auto;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.OneConeBalanceHP;
import frc.robot.commands.auto.OneConeOneCubeHP;
import frc.robot.commands.goalTracker.SetTrackingMode;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.HPStationIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.AutoScoreCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.OuttakeCommand;
import frc.robot.commands.superstructure.ScoreCommand;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.commands.swerve.TornadoCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.Element;
import frc.robot.util.GoalTracker;
import frc.robot.util.GoalTracker.TrackingMode;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
	public final Swerve m_swerve = new Swerve();
	public final Intake m_intake = new Intake();
	public final Arm m_arm = new Arm();
	public final Gripper m_gripper = new Gripper();
	public final Feeder m_feeder = new Feeder();
	public final Conveyor m_conveyor = new Conveyor();
	public final GoalTracker m_goalTracker =
			new GoalTracker(m_swerve.getField(), m_swerve::getPose);

	public final Superstructure m_superstructure =
			new Superstructure(m_intake, m_feeder, m_conveyor, m_arm, m_gripper);

	private final CommandXboxController m_driverController =
			new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController m_secondDriverController =
			new CommandXboxController(Constants.SECOND_DRIVER_CONTROLLER_PORT);

	private final SendableChooser<CommandBase> autoChooser;

	private final AutoBuilder m_autoBuilder;

	public RobotContainer() {
		configureBindings();
		configureDefaultCommands();

		var events = buildEventMap();

		m_autoBuilder = new AutoBuilder(
				m_swerve::getPose,
				m_swerve::resetOdometry,
				Auto.kTranslationPID, // translation
				Auto.kRotationPID, // rotation
				m_swerve::drive,
				true,
				events,
				m_swerve,
				m_superstructure
		);

		autoChooser = new SendableChooser<CommandBase>();

		autoChooser.addOption(
				"1Cone1CubeHP", OneConeOneCubeHP.oneConeOneCubeLeft(m_autoBuilder, m_superstructure, m_intake));
		autoChooser.addOption("1ConeBalanceHP", OneConeBalanceHP.oneConeBalanceHP(m_autoBuilder, m_swerve, m_superstructure, m_intake));

		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		m_driverController.back().onTrue(new InstantCommand(m_swerve::zeroGyro));

		m_driverController.leftBumper().onTrue(new RetractIntakeCommand(m_intake));
		m_driverController.rightBumper().onTrue(new DeployIntakeCommand(m_intake));

		m_driverController.leftTrigger().onTrue(new HPStationIntakeCommand(m_intake));
		m_driverController.rightTrigger().toggleOnFalse(new IntakeCommand(m_superstructure));

		m_driverController
				.a()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(
										() -> {
											CommandScheduler.getInstance()
													.schedule(
															new AutoScoreCommand(
																	m_swerve,
																	m_goalTracker,
																	m_superstructure));
										}),
								new ScoreCommand(m_superstructure, m_secondDriverController.a()),
								m_superstructure::getAutoScore));
		m_driverController.y().toggleOnTrue(new OuttakeCommand(m_superstructure));

		m_secondDriverController
				.leftBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.BestGoal));

		m_secondDriverController
				.rightBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.ClosestGoal));

		m_secondDriverController.leftTrigger().onTrue(new ConditionalCommand(
				new InstantCommand(() -> {
					m_superstructure.setElement(Element.Cone);
				}),
				new InstantCommand(() -> {}),
						() -> (!m_superstructure.getAutoScore() || m_goalTracker.getTrackingMode() == TrackingMode.ClosestGoal)
		));

		m_secondDriverController.rightTrigger().onTrue(new ConditionalCommand(
				new InstantCommand(() -> {
					m_superstructure.setElement(Element.Cube);
				}),
				new InstantCommand(() -> {}),
				() -> (!m_superstructure.getAutoScore() || m_goalTracker.getTrackingMode() == TrackingMode.ClosestGoal)
		));

		m_secondDriverController
				.y()
				.onTrue(
						new InstantCommand(
								() ->
										m_superstructure.setAutoScore(
												!m_superstructure.getAutoScore())));

		m_secondDriverController.povDown().onTrue(new InstantCommand(m_superstructure::idle));

		m_secondDriverController
				.povLeft()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(() -> {}),
								new InstantCommand(
										() -> {
											m_superstructure.setNode(0, 0);
										}),
								() ->
										m_superstructure.getAutoScore()
												&& !(m_goalTracker.getTrackingMode()
														== TrackingMode.ClosestGoal)));

		m_secondDriverController
				.povRight()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(() -> {}),
								new InstantCommand(
										() -> {
											m_superstructure.setNode(1, 0);
										}),
								() ->
										m_superstructure.getAutoScore()
												&& !(m_goalTracker.getTrackingMode()
														== TrackingMode.ClosestGoal)));

		m_secondDriverController
				.povUp()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(() -> {}),
								new InstantCommand(
										() -> {
											m_superstructure.setNode(2, 0);
										}),
								() ->
										m_superstructure.getAutoScore()
												&& !(m_goalTracker.getTrackingMode()
														== TrackingMode.ClosestGoal)));
	}

	private void configureDefaultCommands() {
		if (!Constants.SWERVE_DISABLED) {
			m_swerve.setDefaultCommand(
					new DriveCommand(
							m_swerve,
							() -> -m_driverController.getLeftY() * m_swerve.getMaxVelocity(),
							() -> -m_driverController.getLeftX() * m_swerve.getMaxVelocity(),
							() -> -m_driverController.getRightX() * Swerve.ANGULAR_VELOCITY_RANGE));
		}
	}

	private HashMap<String, Command> buildEventMap() {
		var events = new HashMap<String, Command>();

		events.put("score", new ScoreCommand(m_superstructure, 2, Element.Cone));
		events.put("idle", new InstantCommand(m_superstructure::idle));
		events.put("deploy", new DeployIntakeCommand(m_intake));
		events.put("retract", new RetractIntakeCommand(m_intake));

		return events;
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
