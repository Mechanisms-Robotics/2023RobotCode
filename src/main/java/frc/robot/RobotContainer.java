package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.MobilityAutoLeft;
import frc.robot.commands.auto.MobilityAutoRight;
import frc.robot.commands.auto.OneConeLeft;
import frc.robot.commands.auto.OneConeOneCubeLeft;
import frc.robot.commands.auto.OneConeOneCubeRight;
import frc.robot.commands.auto.OneConeRight;
import frc.robot.commands.auto.OneConeTwoCubesLeft;
import frc.robot.commands.auto.OneConeTwoCubesRight;
import frc.robot.commands.conveyor.ConveyCommand;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.AprilTagTracker;
import frc.robot.util.GoalTracker;

public class RobotContainer {
	public final Swerve m_swerveSubsystem = new Swerve();
	public final Intake m_intakeSubsystem = new Intake();
	//	public final Arm m_armSubsystem = new Arm();
	//	public final Gripper m_gripperSubsystem = new Gripper();
	public final Feeder m_feederSubsystem = new Feeder();
	public final Conveyor m_conveyorSubsystem = new Conveyor();
	private final GoalTracker m_goalTracker =
			new GoalTracker(m_swerveSubsystem.getField(), m_swerveSubsystem::getPose);

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
				"MobilityAutoLeft", MobilityAutoLeft.mobilityAutoLeftCommand(m_swerveSubsystem));
		autoChooser.addOption(
				"MobilityAutoRight", MobilityAutoRight.mobilityAutoRightCommand(m_swerveSubsystem));
		autoChooser.addOption("1ConeLeft", OneConeLeft.oneConeLeft(m_swerveSubsystem));
		autoChooser.addOption("1ConeRight", OneConeRight.oneConeRight(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone1CubeLeft", OneConeOneCubeLeft.oneConeOneCubeLeft(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone1CubeRight", OneConeOneCubeRight.oneConeOneCubeRight(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone2CubesLeft", OneConeTwoCubesLeft.oneConeTwoCubesLeft(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone2CubesRight", OneConeTwoCubesRight.oneConeTwoCubesRight(m_swerveSubsystem));

		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		m_driverController.back().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro));

		//		m_driverController
		//				.a()
		//				.onTrue(
		//						new FunctionalCommand(
		//								() -> {
		//									CommandScheduler.getInstance()
		//											.schedule(
		//													AutoCommands.driveToAvoidObstaclesCommand(
		//															m_goalTracker
		//																	.getClosestGoal()
		//																	.plus(Constants.SCORING_OFFSET),
		//															m_swerveSubsystem));
		//								},
		//								() -> {},
		//								(interrupted) -> {},
		//								() -> true,
		//								m_swerveSubsystem));

		//		m_driverController
		//				.y()
		//				.onTrue(
		//						new FunctionalCommand(
		//								() -> {
		//									CommandScheduler.getInstance()
		//											.schedule(
		//													AutoCommands.driveToAvoidObstaclesCommand(
		//															Constants.PICKUP_POSE,
		//															m_swerveSubsystem));
		//								},
		//								() -> {},
		//								(interrupted) -> {},
		//								() -> true,
		//								m_swerveSubsystem));

		//		m_driverController
		//				.b()
		//				.onTrue(
		//						new InstantCommand(
		//								() -> {
		//									CommandScheduler.getInstance()
		//											.requiring(m_swerveSubsystem)
		//											.cancel();
		//								}));

		//		m_driverController.leftBumper().toggleOnTrue(new
		// InstantCommand(m_intakeSubsystem::toggleBrakeMode));

		//		m_driverController.rightBumper().toggleOnTrue(new
		// InstantCommand(m_intakeSubsystem::intake));

		//		m_driverController.rightBumper().onTrue(new InstantCommand(m_intakeSubsystem::retract));
		//		m_driverController.leftBumper().onTrue(new InstantCommand(m_intakeSubsystem::deploy));
		//		m_driverController.x().onTrue(new
		// InstantCommand(m_intakeSubsystem::setToGamePieceStation));
		//		m_driverController.b().onTrue(new InstantCommand(m_gripperSubsystem::toggle));

		m_driverController.rightBumper().onTrue(new InstantCommand(m_intakeSubsystem::deploy));
		m_driverController.leftBumper().onTrue(new InstantCommand(m_intakeSubsystem::retract));
		m_driverController
				.leftTrigger()
				.onTrue(new InstantCommand(m_intakeSubsystem::setToGamePieceStation));

		m_driverController
				.rightTrigger()
				.toggleOnTrue(
						Commands.parallel(new FunctionalCommand(
								() -> {
									m_intakeSubsystem.intake();
									m_feederSubsystem.feed();
								},
								() -> {},
								(interrupted) -> {
									m_intakeSubsystem.stop();
									m_feederSubsystem.stop();
								},
								() -> false,
								m_intakeSubsystem,
								m_feederSubsystem),
								new ConveyCommand(m_conveyorSubsystem.conveyorSensor::get, m_conveyorSubsystem))
						);

		m_driverController
				.a()
				.toggleOnTrue(
						new FunctionalCommand(
								() -> {
									m_intakeSubsystem.outtake();
									m_feederSubsystem.unjam();
									m_conveyorSubsystem.unjam();
								},
								() -> {},
								(interrupted) -> {
									m_intakeSubsystem.stop();
									m_feederSubsystem.stop();
									m_conveyorSubsystem.stop();
								},
								() -> false,
								m_intakeSubsystem,
								m_feederSubsystem,
								m_conveyorSubsystem));

		// UNCOMMENT
		//		m_secondDriverController.b().onTrue(new InstantCommand(m_gripperSubsystem::cone));
		//		m_secondDriverController.x().onTrue(new InstantCommand(m_gripperSubsystem::cube));
		//		m_secondDriverController.y().onTrue(new InstantCommand(m_gripperSubsystem::open));
		//
		//		m_secondDriverController.povDown().onTrue(new InstantCommand(m_armSubsystem::stow));
		//		m_secondDriverController.povLeft().onTrue(new InstantCommand(m_armSubsystem::low));
		//		m_secondDriverController.povRight().onTrue(new InstantCommand(m_armSubsystem::mid));
		//		m_secondDriverController.povUp().onTrue(new InstantCommand(m_armSubsystem::high));

		//		m_driverController.leftBumper().onTrue(new FunctionalCommand(
		//				m_intakeSubsystem::retract,
		//				m_intakeSubsystem::retract,
		//				i -> {},
		//				() -> false,
		//				m_intakeSubsystem
		//		));
		//
		//		m_driverController.rightBumper().onTrue(new FunctionalCommand(
		//				m_intakeSubsystem::deploy,
		//				m_intakeSubsystem::deploy,
		//				i -> {},
		//				() -> false,
		//				m_intakeSubsystem
		//		));

		//		m_driverController
		//				.y()
		//				.toggleOnTrue(
		//						new FunctionalCommand(
		//								() -> {
		//									m_intakeSubsystem.intake();
		//																		m_conveyorSubsystem.convey();
		//									m_feederSubsystem.feed();
		//								},
		//								() -> {},
		//								interrupted -> {
		//									m_feederSubsystem.stop();
		//																		m_conveyorSubsystem.stop();
		//									m_intakeSubsystem.stop();
		//								},
		//								() -> false,
		//								m_feederSubsystem,
		//								m_intakeSubsystem,
		//								m_conveyorSubsystem));

		//		m_driverController
		//				.a()
		//				.toggleOnFalse(
		//						new FunctionalCommand(
		//								() -> {
		//																		m_intakeSubsystem.unjam();
		//									m_feederSubsystem.unjam();
		//									m_intakeSubsystem.outtake();
		//																		m_conveyorSubsystem.unjam();
		//								},
		//								() -> {},
		//								interrupted -> {
		//									m_feederSubsystem.stop();
		//									m_intakeSubsystem.stop();
		//																		m_conveyorSubsystem.stop();
		//								},
		//								() -> false,
		//																m_intakeSubsystem,
		//								m_feederSubsystem,
		//								m_conveyorSubsystem));

		AprilTagTracker.setAllianceSide(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
	}

	private void configureDefaultCommands() {
		if (!Constants.SWERVE_DISABLED) {
			m_swerveSubsystem.setDefaultCommand(
					new DriveCommand(
							m_swerveSubsystem,
							() -> -m_driverController.getLeftY() * Swerve.VELOCITY_RANGE,
							() -> -m_driverController.getLeftX() * Swerve.VELOCITY_RANGE,
							() -> -m_driverController.getRightX() * Swerve.ANGULAR_VELOCITY_RANGE));

		}

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
