package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Superstructure;

public class IntakeCommand extends FunctionalCommand {
	public IntakeCommand(Superstructure superstructure) {
		super(
				superstructure::intake,
				() -> {},
				(interrupted) -> {
					superstructure.idle();
				},
				() -> false,
				superstructure);
	}
}
