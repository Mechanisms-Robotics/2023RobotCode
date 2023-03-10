package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.MobilityAutoLeft;
import frc.robot.commands.auto.MobilityAutoRight;
import frc.robot.commands.auto.OneConeBalanceLeft;
import frc.robot.commands.auto.OneConeBalanceRight;
import frc.robot.commands.auto.OneConeLeft;
import frc.robot.commands.auto.OneConeOneCubeLeft;
import frc.robot.commands.auto.OneConeOneCubeRight;
import frc.robot.commands.auto.OneConeRight;
import frc.robot.commands.goalTracker.SetTrackingMode;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.HPStationIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.OuttakeCommand;
import frc.robot.commands.superstructure.ScoreCommand;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.GoalTracker;
import frc.robot.util.GoalTracker.TrackingMode;

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

	public RobotContainer() {
		configureBindings();
		configureDefaultCommands();

		autoChooser = new SendableChooser<CommandBase>();

		autoChooser.addOption(
				"MobilityAutoLeft", MobilityAutoLeft.mobilityAutoLeftCommand(m_swerve));
		autoChooser.addOption(
				"MobilityAutoRight", MobilityAutoRight.mobilityAutoRightCommand(m_swerve));
		autoChooser.addOption("1ConeLeft", OneConeLeft.oneConeLeft(m_swerve));
		autoChooser.addOption("1ConeRight", OneConeRight.oneConeRight(m_swerve));
		autoChooser.addOption("1ConeBalanceLeft", OneConeBalanceLeft.oneConeBalanceLeft(m_swerve));
		autoChooser.addOption(
				"1ConeBalanceRight", OneConeBalanceRight.oneConeBalanceRight(m_swerve));
		autoChooser.addOption("1Cone1CubeLeft", OneConeOneCubeLeft.oneConeOneCubeLeft(m_swerve));
		autoChooser.addOption("1Cone1CubeRight", OneConeOneCubeRight.oneConeOneCubeRight(m_swerve));

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
						new InstantCommand(
								() -> {
									CommandScheduler.getInstance()
											.schedule(
													new ScoreCommand(
															m_swerve,
															m_goalTracker,
															m_superstructure));
								}));
		m_driverController.y().toggleOnTrue(new OuttakeCommand(m_superstructure));

		m_secondDriverController
				.leftBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.BestGoal));

		m_secondDriverController
				.rightBumper()
				.onTrue(new SetTrackingMode(m_goalTracker, TrackingMode.ClosestGoal));
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

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
