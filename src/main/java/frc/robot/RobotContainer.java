package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.CubeArmCommand;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.auto.MobilityAutoLeft;
import frc.robot.commands.auto.MobilityAutoRight;
import frc.robot.commands.auto.OneConeBalanceLeft;
import frc.robot.commands.auto.OneConeBalanceRight;
import frc.robot.commands.auto.OneConeLeft;
import frc.robot.commands.auto.OneConeOneCubeLeft;
import frc.robot.commands.auto.OneConeOneCubeRight;
import frc.robot.commands.auto.OneConeRight;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.AprilTagTracker;
import frc.robot.util.GoalTracker;

public class RobotContainer {
	public final Swerve m_swerveSubsystem = new Swerve();
	public final Intake m_intakeSubsystem = new Intake();
	public final Arm m_armSubsystem = new Arm();
	public final Gripper m_gripperSubsystem = new Gripper();
	public final Feeder m_feederSubsystem = new Feeder();
	public final Conveyor m_conveyorSubsystem = new Conveyor();
	private final GoalTracker m_goalTracker =
			new GoalTracker(m_swerveSubsystem.getField(), m_swerveSubsystem::getPose);

	private final Superstructure m_superstructure = new Superstructure(m_intakeSubsystem, m_feederSubsystem, m_conveyorSubsystem, m_armSubsystem);

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
		autoChooser.addOption("1ConeBalanceLeft", OneConeBalanceLeft.oneConeBalanceLeft(m_swerveSubsystem));
		autoChooser.addOption("1ConeBalanceRight", OneConeBalanceRight.oneConeBalanceRight(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone1CubeLeft", OneConeOneCubeLeft.oneConeOneCubeLeft(m_swerveSubsystem));
		autoChooser.addOption(
				"1Cone1CubeRight", OneConeOneCubeRight.oneConeOneCubeRight(m_swerveSubsystem));

		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		m_driverController.back().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro));

				m_driverController
						.y()
						.onTrue(
								new FunctionalCommand(
										() -> {
											CommandScheduler.getInstance()
													.schedule(
															AutoCommands.driveToAvoidObstaclesCommand(
																	m_goalTracker
																			.getClosestGoal()
																			.plus(Constants.SCORING_OFFSET),
																	m_swerveSubsystem));
										},
										() -> {},
										(interrupted) -> {},
										() -> true,
										m_swerveSubsystem));

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
						Commands.parallel(

            new FunctionalCommand(
								m_superstructure::feed,
                () -> {},
                (interrupted) -> {
                  m_superstructure.idle();
                },
								m_superstructure::isPositioned,
                m_superstructure),
								new CubeArmCommand(m_intakeSubsystem, m_armSubsystem, m_gripperSubsystem, m_conveyorSubsystem::getDebouncedSensor, m_superstructure::isPositioned)));

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

//		m_secondDriverController.b().onTrue(new CubeArmCommand(m_armSubsystem, m_gripperSubsystem, m_conveyorSubsystem::getDebouncedSensor, m_superstructure::isPositioned));

		m_secondDriverController.y().onTrue(new InstantCommand(() -> {
			m_superstructure.setElement(Superstructure.Element.Cube);
		})); // Set intake and feeder mode to cube

		m_secondDriverController.x().onTrue(new InstantCommand(() -> {
			m_superstructure.setElement(Superstructure.Element.Cone);
		})); // Set intake and feeder mode to cone

		// UNCOMMENT
		//		m_secondDriverController.b().onTrue(new InstantCommand(m_gripperSubsystem::cone));
		//		m_secondDriverController.x().onTrue(new InstantCommand(m_gripperSubsystem::cube));
		//		m_secondDriverController.y().onTrue(new InstantCommand(m_gripperSubsystem::open));
		//
		m_secondDriverController.povDown().onTrue(
				Commands.parallel(
						Commands.runOnce(m_armSubsystem::stow),
						Commands.runOnce(m_intakeSubsystem::retract)
				)
		);
		m_secondDriverController.povLeft().onTrue(
				Commands.parallel(
						Commands.runOnce(m_armSubsystem::low),
						Commands.runOnce(m_intakeSubsystem::retract)
				)
		);
		m_secondDriverController.povRight().onTrue(
				Commands.parallel(
						Commands.runOnce(m_armSubsystem::mid),
						Commands.runOnce(m_intakeSubsystem::retract)
				)
		);
		m_secondDriverController.povUp().onTrue(
				Commands.parallel(
						Commands.runOnce(m_armSubsystem::high),
						Commands.runOnce(m_intakeSubsystem::retract)
				)
		);

//		m_secondDriverController.povDown().onTrue(new InstantCommand(m_armSubsystem::stow));
//		m_secondDriverController.povLeft().onTrue(new InstantCommand(m_armSubsystem::low));
//		m_secondDriverController.povRight().onTrue(new InstantCommand(m_armSubsystem::mid));
//		m_secondDriverController.povUp().onTrue(new InstantCommand(m_armSubsystem::high));

		m_secondDriverController.
				a().onTrue(
				Commands.sequence(
					Commands.runOnce(m_gripperSubsystem::open, m_gripperSubsystem),
					Commands.waitSeconds(0.5),
					Commands.runOnce(m_armSubsystem::stow, m_armSubsystem)
				)
		);

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
				  () -> -m_driverController.getLeftY() * m_swerveSubsystem.getMaxVelocity(),
				  () -> -m_driverController.getLeftX() * m_swerveSubsystem.getMaxVelocity(),
				  () -> -m_driverController.getRightX() * Swerve.ANGULAR_VELOCITY_RANGE));
		}

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
