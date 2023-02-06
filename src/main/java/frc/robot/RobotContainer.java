package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.MobilityAutoLeft;
import frc.robot.commands.auto.MobilityAutoRight;
import frc.robot.commands.auto.OneConeLeft;
import frc.robot.commands.auto.OneConeOneCubeLeft;
import frc.robot.commands.auto.OneConeOneCubeRight;
import frc.robot.commands.auto.OneConeRight;
import frc.robot.commands.auto.OneConeTwoCubesLeft;
import frc.robot.commands.auto.OneConeTwoCubesRight;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.commands.swerve.DriveToCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AprilTagTracker;
import org.photonvision.common.hardware.VisionLEDMode;

public class RobotContainer {
	private final AprilTagTracker m_aprilTagTracker = new AprilTagTracker();
	private final Swerve m_swerveSubsystem = new Swerve(m_aprilTagTracker);
	private final CommandXboxController m_driverController =
			new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

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

		m_driverController
				.a()
				.onTrue(new DriveToCommand(m_swerveSubsystem, Constants.CENTER_OF_FIELD, 2.0, 1.0));
	}

	private void configureDefaultCommands() {
		m_swerveSubsystem.setDefaultCommand(
				new DriveCommand(
						m_swerveSubsystem,
						() -> -m_driverController.getLeftY() * Swerve.MAX_VELOCITY,
						() -> -m_driverController.getLeftX() * Swerve.MAX_VELOCITY,
						() -> -m_driverController.getRightX() * Swerve.MAX_ANGULAR_VELOCITY));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setLimelightLEDMode(VisionLEDMode mode) {
		m_aprilTagTracker.getCamera().setLED(mode);
	}
}
