package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Auto;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.MobilityAutoHP;
import frc.robot.commands.auto.MobilityAutoWall;
import frc.robot.commands.auto.OneElementBalanceHP;
import frc.robot.commands.auto.OneElementBalanceWall;
import frc.robot.commands.auto.ThreeElementHP;
import frc.robot.commands.auto.ThreeElementWall;
import frc.robot.commands.auto.TwoElementBalanceHP;
import frc.robot.commands.auto.TwoElementBalanceWall;
import frc.robot.commands.auto.TwoElementHP;
import frc.robot.commands.auto.TwoElementWall;
import frc.robot.commands.goalTracker.SetTrackingMode;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.HPStationIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.states.arm.Scoring;
import frc.robot.states.arm.Stowed;
import frc.robot.states.intake.Intaking;
import frc.robot.states.intake.Outtaking;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.Element;
import frc.robot.util.GoalTracker;
import frc.robot.util.GoalTracker.TrackingMode;
import frc.robot.util.LEDWrapper;
import java.util.HashMap;

public class RobotContainer {
	private SendableChooser<Boolean> m_swerveDisabledChooser = null;

	public final Swerve m_swerve = new Swerve(m_swerveDisabledChooser);
	public final Intake m_intake = new Intake();
	public final Arm m_arm = new Arm();
	public final Gripper m_gripper = new Gripper();
	public final Feeder m_feeder = new Feeder();
	public final Conveyor m_conveyor = new Conveyor();

	public final GoalTracker m_goalTracker =
			new GoalTracker(m_swerve.getField(), m_swerve::getPose);

	public final LEDWrapper m_ledWrapper = new LEDWrapper();

	public final Superstructure m_superstructure =
			new Superstructure(m_intake, m_feeder, m_conveyor, m_arm, m_gripper, m_ledWrapper);

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

		m_autoBuilder =
				new AutoBuilder(
						m_swerve::getPose,
						m_swerve::resetOdometry,
						Auto.kTranslationPID, // translation
						Auto.kRotationPID, // rotation
						m_swerve::drive,
						true,
						events,
						m_swerve,
						m_superstructure);

		autoChooser = new SendableChooser<CommandBase>();

		autoChooser.addOption("MobilityAutoHP", MobilityAutoHP.mobilityAutoHP(m_autoBuilder));
		autoChooser.addOption("MobilityAutoWall", MobilityAutoWall.mobilityAutoWall(m_autoBuilder));
		autoChooser.setDefaultOption(
				"1ElementBalanceHP", OneElementBalanceHP.oneElementBalanceHP(m_autoBuilder));
		autoChooser.addOption(
				"1ElementBalanceWall", OneElementBalanceWall.oneElementBalanceWall(m_autoBuilder));
		autoChooser.addOption("2ElementHP", TwoElementHP.twoElementHP(m_autoBuilder));
		autoChooser.addOption("2ElementWall", TwoElementWall.twoElementWall(m_autoBuilder));
		autoChooser.addOption(
				"2ElementBalanceHP", TwoElementBalanceHP.twoElementBalanceHP(m_autoBuilder));
		autoChooser.addOption(
				"2ElementBalanceWall", TwoElementBalanceWall.twoElementBalanceWall(m_autoBuilder));
		autoChooser.addOption("3ElementHP", ThreeElementHP.threeElementHP(m_autoBuilder));
		autoChooser.addOption("3ElementWall", ThreeElementWall.threeElementWall(m_autoBuilder));

		SmartDashboard.putData("Auto Chooser", autoChooser);

		m_swerveDisabledChooser = new SendableChooser<>();
		m_swerveDisabledChooser.setDefaultOption("Enabled", false);
		m_swerveDisabledChooser.addOption("Disabled", true);

		SmartDashboard.putData("Swerve Chooser", m_swerveDisabledChooser);
	}

	private void configureBindings() {
		m_driverController.back().onTrue(new InstantCommand(m_swerve::zeroGyro));

		m_driverController.leftBumper().onTrue(new RetractIntakeCommand(m_intake));
		m_driverController.rightBumper().onTrue(new DeployIntakeCommand(m_intake));

		m_driverController.leftTrigger().onTrue(new HPStationIntakeCommand(m_intake));

		m_driverController
				.rightTrigger()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(m_superstructure::intake),
								new InstantCommand(m_superstructure::idle),
								() ->
										m_superstructure.getIntakeState().getClass()
												!= Intaking.class));

		m_driverController
				.y()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(m_superstructure::outtake),
								new InstantCommand(m_superstructure::idle),
								() ->
										m_superstructure.getIntakeState().getClass()
												!= Outtaking.class));

		//		m_driverController
		//				.a()
		//				.onTrue(
		//						new ConditionalCommand(
		//								new InstantCommand(
		//										() -> {
		//											CommandScheduler.getInstance()
		//													.schedule(
		//															new AutoScoreCommand(
		//																	m_autoBuilder,
		//																	m_swerve,
		//																	m_goalTracker,
		//																	m_superstructure));
		//										}),
		//								Commands.none(),
		//								m_superstructure::getAutoScore));

		m_driverController
				.x()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(() -> m_swerve.setClimbMode(false)),
								new InstantCommand(() -> m_swerve.setClimbMode(true)),
								m_swerve::getClimbMode));

		m_secondDriverController
				.a()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(m_superstructure::close),
								new ConditionalCommand(
										new InstantCommand(m_superstructure::score),
										new ConditionalCommand(
												new InstantCommand(m_superstructure::idle),
												Commands.none(),
												() ->
														m_superstructure.getArmState().getClass()
																== Scoring.class),
										() ->
												m_superstructure.getArmState().getClass()
														== Stowed.class),
								() -> m_superstructure.getArmState().getClass() == Stowed.class && m_superstructure.getArmState().isOpen()));

		m_secondDriverController
				.leftBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.BestGoal));

		m_secondDriverController
				.rightBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.ClosestGoal));

		m_secondDriverController
				.leftTrigger()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(
										() -> {
											m_superstructure.setElement(Element.Cone);
										}),
								new InstantCommand(() -> {}),
								() ->
										(!m_superstructure.getAutoScore()
												|| m_goalTracker.getTrackingMode()
														== TrackingMode.ClosestGoal)));

		m_secondDriverController
				.rightTrigger()
				.onTrue(
						new ConditionalCommand(
								new InstantCommand(
										() -> {
											m_superstructure.setElement(Element.Cube);
										}),
								new InstantCommand(() -> {}),
								() ->
										(!m_superstructure.getAutoScore()
												|| m_goalTracker.getTrackingMode()
														== TrackingMode.ClosestGoal)));

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
		return events;
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
