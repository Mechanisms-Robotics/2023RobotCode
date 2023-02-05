package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.swerve.DriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AprilTags;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class RobotContainer {
	private AprilTags aprilTags = new AprilTags();
	private final Swerve m_swerveSubsystem = new Swerve(aprilTags);
	private final CommandXboxController m_driverController =
			new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

	public RobotContainer() {
		configureBindings();
		configureDefaultCommands();
	}

	private void configureBindings() {
		m_driverController.back().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro));
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
		return AutoCommands.moveForwardCommand(m_swerveSubsystem);
	}

	public void setLimelightLEDMode(VisionLEDMode mode) {
		aprilTags.getCamera().setLED(mode);
	}
}
