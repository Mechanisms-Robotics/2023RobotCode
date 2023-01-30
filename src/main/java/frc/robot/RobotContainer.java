package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final CommandXboxController driverController =
			new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

	private final Swerve swerve = new Swerve();

	public RobotContainer() {
		swerve.zeroHeading();

		configureBindings();
		configureDefaultCommands();
	}

	private void configureBindings() {}

	private void configureDefaultCommands() {
		swerve.setDefaultCommand(
				new DriveTeleopCommand(
						driverController::getLeftX,
						driverController::getLeftY,
						driverController::getRightX,
						true,
						swerve));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
